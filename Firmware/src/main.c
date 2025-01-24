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

/* Global variables section */
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
typedef struct __attribute__((packed))
{
  uint8_t flags;
  uint32_t wheel_revolution;
  uint16_t last_event_time;
} csc_measurement_t;
csc_measurement_t cscm_data;

static const struct gpio_dt_spec user_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec user_switch = GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios);

static struct gpio_callback switch_cb_data;

uint64_t curr_time;  // ms
uint64_t prev_time;  // ms
#define DEBOUNCE_TIME_MS 10

uint16_t send_notify_flag;
/* End of global variables section */

/* Bluetooth section */
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
  send_notify_flag = value;
}

static ssize_t read_feature(struct bt_conn* conn,
                            const struct bt_gatt_attr* attr,
                            void* buf,
                            uint16_t len,
                            uint16_t offset)
{
  uint16_t value = 1;  // CSC Feature - Wheel Revolution Data Supported
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &value, sizeof(value));

  return sizeof(value);
}

static ssize_t write_sc(struct bt_conn* conn,
                        const struct bt_gatt_attr* attr,
                        const void* buf,
                        uint16_t len, uint16_t offset, uint8_t flags)
{
  cscm_data.wheel_revolution = *((uint32_t*)buf);
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
/* End of bluetooth section */

/* GPIO section */
static void switch_cb_func(const struct device* port, struct gpio_callback* cb, gpio_port_pins_t pins)
{
  curr_time = k_uptime_get();
  if((curr_time - prev_time) > DEBOUNCE_TIME_MS)
  {
    cscm_data.wheel_revolution++;
    cscm_data.last_event_time += (uint16_t)((curr_time - prev_time) & 0xFFFF);
    prev_time = curr_time;
  }
}
/* End of GPIO section */

int main(void)
{
  LOG_INF("Start of main.");
  int err;

  if(!gpio_is_ready_dt(&user_led))
  {
    return 0;
  }
  if(!gpio_is_ready_dt(&user_switch))
  {
    return 0;
  }

  gpio_pin_configure_dt(&user_led, GPIO_OUTPUT_ACTIVE);
  gpio_pin_configure_dt(&user_switch, GPIO_INPUT);
  gpio_pin_interrupt_configure_dt(&user_switch, GPIO_INT_EDGE_TO_ACTIVE);

  gpio_init_callback(&switch_cb_data, switch_cb_func, BIT(user_switch.pin));
  gpio_add_callback_dt(&user_switch, &switch_cb_data);

  err = bt_enable(NULL);
  if(err)
  {
    printk("Bluetooth init failed (err %d)\n", err);
    return 0;
  }

  bt_ready();

  cscm_data.flags = 1;  // Wheel Revolution Data Present

  while(1)
  {
    if(send_notify_flag)
    {
      bt_gatt_notify(NULL, &csc_svc.attrs[1], &cscm_data, sizeof(cscm_data));
    }
    k_sleep(K_SECONDS(1));
  }
  return 0;
}

/* Test thread for simulating spinning wheel. */
/*
uint16_t millis = 1000;
void sim_tread(void)
{
  while(1)
  {
    switch_cb_func(NULL, NULL, 0);
    k_sleep(K_MSEC(millis));
  }
}

#define SIM_THREAD_STACKSIZE 512
#define SIM_THREAD_PRIORITY 2
K_THREAD_DEFINE(sim_tread_id, SIM_THREAD_STACKSIZE, sim_tread, NULL, NULL, NULL, SIM_THREAD_PRIORITY, 0, 0);
*/