/* main.c - Application main entry point */

/*
 * Copyright (c) 2025 Jernej Pangerc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/uuid.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

typedef struct __attribute__((packed))
{
  uint8_t flags;
  uint32_t wheel_revolution;
  uint16_t last_event_time;
} csc_measurement_t;

csc_measurement_t test_data;

static const struct bt_data advertisement[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                  BT_UUID_16_ENCODE(BT_UUID_CSC_VAL)),
};

static const struct bt_data scan_response[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void cscmc_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value)
{
}

static ssize_t read_feature(struct bt_conn* conn,
                            const struct bt_gatt_attr* attr,
                            void* buf,
                            uint16_t len,
                            uint16_t offset)
{
  uint16_t value = 1;
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &value, sizeof(value));

  return 0;
}

static ssize_t write_sc(struct bt_conn* conn,
                        const struct bt_gatt_attr* attr,
                        const void* buf,
                        uint16_t len, uint16_t offset, uint8_t flags)
{
  return len;
}

/* Cycling Speed and Cadence Service Declaration */
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

static void connected(struct bt_conn* conn, uint8_t err)
{
  if(err)
  {
    printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
  }
  else
  {
    printk("Connected\n");
  }
}

static void disconnected(struct bt_conn* conn, uint8_t reason)
{
  printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void bt_ready(void)
{
  int err;

  printk("Bluetooth initialized\n");

  err = bt_le_adv_start(BT_LE_ADV_CONN, advertisement, ARRAY_SIZE(advertisement), scan_response, ARRAY_SIZE(scan_response));
  if(err)
  {
    printk("Advertising failed to start (err %d)\n", err);
    return;
  }

  printk("Advertising successfully started\n");
}

int main(void)
{
  LOG_INF("Start of main.");
  int err;

  err = bt_enable(NULL);
  if(err)
  {
    printk("Bluetooth init failed (err %d)\n", err);
    return 0;
  }

  bt_ready();

  test_data.flags = 1;
  test_data.wheel_revolution = 1;

  while(1)
  {
    k_sleep(K_SECONDS(1));
    test_data.wheel_revolution++;
    test_data.last_event_time += 1024;
    bt_gatt_notify(NULL, &csc_svc.attrs[1], &test_data, sizeof(test_data));
  }
  return 0;
}
