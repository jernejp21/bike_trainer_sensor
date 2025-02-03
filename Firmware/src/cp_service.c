/* cp_service.c - Cycling Power Service module */

/*
 * Copyright (c) 2025 Jernej Pangerc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "cp_service.h"
#include <zephyr/drivers/retained_mem.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>

#if IS_ENABLED(CONFIG_USE_CPS)

#define RETAINED_CRC_OFFSET offsetof(cp_measurement_t, crc)
#define RETAINED_CHECKED_SIZE (RETAINED_CRC_OFFSET + sizeof(cpm_data.crc))

const static struct device* retained_mem_device = DEVICE_DT_GET(DT_ALIAS(retainedmemdevice));

cp_measurement_t cpm_data;

static void cpmc_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value)
{
  send_notify_flag = value;
}

static ssize_t read_feature(struct bt_conn* conn,
                            const struct bt_gatt_attr* attr,
                            void* buf,
                            uint16_t len,
                            uint16_t offset)
{
  uint32_t value = CPF_CRANK_REV | CPF_DIST_SYS_NOT_FOR_DIST;  // CP Feature - Wheel Revolution Data Supported
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &value, sizeof(value));
}

// Cycling Speed and Cadence Service Declaration
BT_GATT_SERVICE_DEFINE(csc_svc,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_CPS),
                       BT_GATT_CHARACTERISTIC(BT_UUID_GATT_CPS_CPF, BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ, read_feature, NULL, NULL),
                       BT_GATT_CHARACTERISTIC(BT_UUID_GATT_CPS_CPM, BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_NONE, NULL, NULL, NULL),
                       BT_GATT_CCC(cpmc_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

void notify_service(void)
{
  uint16_t data_len = sizeof(cpm_data) - sizeof(cpm_data.crc);
  bt_gatt_notify(NULL, &csc_svc.attrs[3], &cpm_data, data_len);
}

void update_service_data(uint16_t time)
{
  cpm_data.crank_revolution++;
  cpm_data.last_crank_event_time += time;
}

void retained_validate(void)
{
  retained_mem_read(retained_mem_device, 0, (uint8_t*)&cpm_data, sizeof(cpm_data));

  /* The residue of a CRC is what you get from the CRC over the
   * message catenated with its CRC.  This is the post-final-xor
   * residue for CRC-32 (CRC-32/ISO-HDLC) which Zephyr calls
   * crc32_ieee.
   */
  const uint32_t residue = 0x2144df1c;
  uint32_t crc = crc32_ieee((const uint8_t*)&cpm_data, RETAINED_CHECKED_SIZE);
  bool valid = (crc == residue);

  /* If the CRC isn't valid, reset the retained data. */
  if(!valid)
  {
    memset(&cpm_data, 0, sizeof(cpm_data));
  }
}

void retained_update(void)
{
  uint32_t crc = crc32_ieee((const uint8_t*)&cpm_data, RETAINED_CRC_OFFSET);

  cpm_data.crc = sys_cpu_to_le32(crc);

  retained_mem_write(retained_mem_device, 0, (uint8_t*)&cpm_data, sizeof(cpm_data));
}

void init_service_data(void)
{
  cpm_data.flags = CPM_FLAG_CRANK_REV;  // Wheel Revolution Data Present
}
#endif  // IS_ENABLED(CONFIG_USE_CSC)