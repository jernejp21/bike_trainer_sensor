/* main.c - Application main entry point */

/*
 * Copyright (c) 2025 Jernej Pangerc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ftm_service.h"
#include <zephyr/drivers/retained_mem.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#if IS_ENABLED(CONFIG_USE_FTMS)
#define RETAINED_CRC_OFFSET offsetof(indoor_bike_data_t, crc)
#define RETAINED_CHECKED_SIZE (RETAINED_CRC_OFFSET + sizeof(indoor_bike_data.crc))

const static struct device* retained_mem_device = DEVICE_DT_GET(DT_ALIAS(retainedmemdevice));

indoor_bike_data_t indoor_bike_data;

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
  fm_feature[0] = FMF_AVG_SPEED | FMF_ELAPSED_TIME | FMF_TOTAL_DIST; /* fitness machine features */
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

void update_service_data(uint16_t time)
{
  
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
  indoor_bike_data.flags = IBD_FLAG_AVG_SPEED | IBD_FLAG_ELAPSED_TIME | IBD_FLAG_TOTAL_DIST;  // Wheel Revolution Data Present
}
#endif  // IS_ENABLED(CONFIG_USE_FTMS)