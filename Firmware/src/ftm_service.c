/* ftm_service.c - Fitness Machine Service module */

/*
 * Copyright (c) 2025 Jernej Pangerc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ftm_service.h"
#include "trainers/kinetic_road_machine.h"
#include <math.h>
#include <zephyr/drivers/retained_mem.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#if IS_ENABLED(CONFIG_USE_FTMS)
#define RETAINED_CRC_OFFSET offsetof(indoor_bike_data_t, crc)
#define RETAINED_CHECKED_SIZE (RETAINED_CRC_OFFSET + sizeof(indoor_bike_data.crc))

const static struct device* retained_mem_device = DEVICE_DT_GET(DT_ALIAS(retainedmemdevice));

indoor_bike_data_t indoor_bike_data;

float inst_speed;
uint16_t floor, ceil, power;

static void ibdc_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value)
{
  send_notify_flag = value;
}

static ssize_t read_fm_feature(struct bt_conn* conn,
                               const struct bt_gatt_attr* attr,
                               void* buf,
                               uint16_t len,
                               uint16_t offset)
{
  uint32_t fm_feature[2];
  fm_feature[0] = FMF_POWER_MEASURMENT; /* fitness machine features */
  fm_feature[1] = 0; /* target setting features */
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &fm_feature, sizeof(fm_feature));
}

// Fitness Machine Service Declaration
BT_GATT_SERVICE_DEFINE(ftm_svc,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_FMS),
                       BT_GATT_CHARACTERISTIC(BT_UUID_GATT_FMF, BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ, read_fm_feature, NULL, NULL),
                       BT_GATT_CHARACTERISTIC(BT_UUID_GATT_IBD, BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_NONE, NULL, NULL, NULL),
                       BT_GATT_CCC(ibdc_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

void notify_service(void)
{
  uint16_t data_len = sizeof(indoor_bike_data) - sizeof(indoor_bike_data.crc);
  bt_gatt_notify(NULL, &ftm_svc.attrs[3], &indoor_bike_data, data_len);
}

/* This function is called from ISR, so it must be quick. */
void update_service_data(uint16_t time_ms)
{
  inst_speed = ((float)(M_PI * CONFIG_ROLLER_DIAMETER) / (float)time_ms * (float)3.6);  // km/h

  floor = (int16_t)(inst_speed / 10);  // floor to nearest 10 and get index
  ceil = floor + 1;  // ceil to nearest 10 and get index

  /* Use liner interpolation to calculate power between defined values. */
  /* y = (y0(x1-x) + y1(x-x0)) / (x1-x0) */
  /* y0 = powere_curve[floor] */
  /* y1 = powere_curve[ceil] */
  /* x0 = floor*10 (floor speed rounded to 10) */
  /* x1 = ceil*10 (ceil speed rounded to 10) */
  /* x = inst_speed */
  /* x1-x0 is alway 10, since we have steps of 10 km/h. */
  power = (uint16_t)((power_curve[floor] * (ceil * 10 - inst_speed) + power_curve[ceil] * (inst_speed - floor * 10)) / 10);

  indoor_bike_data.inst_power = power;
}

void retained_validate(void)
{
  retained_mem_read(retained_mem_device, 0, (uint8_t*)&indoor_bike_data, sizeof(indoor_bike_data));

  /* The residue of a CRC is what you get from the CRC over the
   * message catenated with its CRC.  This is the post-final-xor
   * residue for CRC-32 (CRC-32/ISO-HDLC) which Zephyr calls
   * crc32_ieee.
   */
  const uint32_t residue = 0x2144df1c;
  uint32_t crc = crc32_ieee((const uint8_t*)&indoor_bike_data, RETAINED_CHECKED_SIZE);
  bool valid = (crc == residue);

  /* If the CRC isn't valid, reset the retained data. */
  if(!valid)
  {
    memset(&indoor_bike_data, 0, sizeof(indoor_bike_data));
  }
}

void retained_update(void)
{
  uint32_t crc = crc32_ieee((const uint8_t*)&indoor_bike_data, RETAINED_CRC_OFFSET);

  indoor_bike_data.crc = sys_cpu_to_le32(crc);

  retained_mem_write(retained_mem_device, 0, (uint8_t*)&indoor_bike_data, sizeof(indoor_bike_data));
}

void init_service_data(void)
{
  indoor_bike_data.flags = IBD_FLAG_INST_SPEED_NOT_PRESENT | IBD_FLAG_INST_POWER;
}
#endif  // IS_ENABLED(CONFIG_USE_FTMS)