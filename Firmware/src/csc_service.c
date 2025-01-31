/* main.c - Application main entry point */

/*
 * Copyright (c) 2025 Jernej Pangerc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "csc_service.h"
#include <zephyr/sys/crc.h>
#include <zephyr/drivers/retained_mem.h>
#include <zephyr/sys/byteorder.h>

#if IS_ENABLED(CONFIG_USE_CSC)

#define RETAINED_CRC_OFFSET offsetof(csc_measurement_t, crc)
#define RETAINED_CHECKED_SIZE (RETAINED_CRC_OFFSET + sizeof(cscm_data.crc))

const static struct device* retained_mem_device = DEVICE_DT_GET(DT_ALIAS(retainedmemdevice));

csc_measurement_t cscm_data;

static void cscmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    send_notify_flag = value;
}

static ssize_t read_feature(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr,
                            void *buf,
                            uint16_t len,
                            uint16_t offset)
{
    uint16_t value = 1; // CSC Feature - Wheel Revolution Data Supported
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &value, sizeof(value));
}

static ssize_t write_sc(struct bt_conn *conn,
                        const struct bt_gatt_attr *attr,
                        const void *buf,
                        uint16_t len, uint16_t offset, uint8_t flags)
{
    cscm_data.wheel_revolution = *((uint32_t *)buf);
    return len;
}

// Cycling Speed and Cadence Service Declaration
BT_GATT_SERVICE_DEFINE(csc_svc,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_CSC),
                       BT_GATT_CHARACTERISTIC(BT_UUID_CSC_MEASUREMENT, BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_NONE, NULL, NULL, NULL),
                       BT_GATT_CCC(cscmc_ccc_cfg_changed,
                                   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
                       BT_GATT_CHARACTERISTIC(BT_UUID_CSC_FEATURE, BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ, read_feature, NULL, NULL),
                       BT_GATT_CHARACTERISTIC(BT_UUID_SC_CONTROL_POINT, BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE, NULL, write_sc, NULL), );

void notify_service(void)
{
    bt_gatt_notify(NULL, &csc_svc.attrs[1], &cscm_data, sizeof(cscm_data));
}

void update_service_data(uint16_t time)
{
    cscm_data.wheel_revolution++;
    cscm_data.last_event_time += time;
}

void retained_validate(void)
{
  retained_mem_read(retained_mem_device, 0, (uint8_t*)&cscm_data, sizeof(cscm_data));

  /* The residue of a CRC is what you get from the CRC over the
   * message catenated with its CRC.  This is the post-final-xor
   * residue for CRC-32 (CRC-32/ISO-HDLC) which Zephyr calls
   * crc32_ieee.
   */
  const uint32_t residue = 0x2144df1c;
  uint32_t crc = crc32_ieee((const uint8_t*)&cscm_data, RETAINED_CHECKED_SIZE);
  bool valid = (crc == residue);

  /* If the CRC isn't valid, reset the retained data. */
  if(!valid)
  {
    memset(&cscm_data, 0, sizeof(cscm_data));
  }
}

void retained_update(void)
{
  uint32_t crc = crc32_ieee((const uint8_t*)&cscm_data, RETAINED_CRC_OFFSET);

  cscm_data.crc = sys_cpu_to_le32(crc);

  retained_mem_write(retained_mem_device, 0, (uint8_t*)&cscm_data, sizeof(cscm_data));
}

void init_service_data(void)
{
    cscm_data.flags = 1;  // Wheel Revolution Data Present
}
#endif //IS_ENABLED(CONFIG_USE_CSC)