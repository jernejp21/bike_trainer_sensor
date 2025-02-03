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
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>

#if IS_ENABLED(CONFIG_USE_CSC)
#include "csc_service.h"
#else
#include "ftm_service.h"
#endif

/* Global variables section */
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#if IS_ENABLED(CONFIG_DEBUG_INFO)
static const struct gpio_dt_spec user_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#endif
static const struct gpio_dt_spec user_switch = GPIO_DT_SPEC_GET(DT_ALIAS(reedswitch), gpios);

static struct gpio_callback switch_cb_data;

static uint64_t curr_time, prev_time;  // ms

uint16_t send_notify_flag;

void shutdown_cb(struct k_timer* timer);
K_TIMER_DEFINE(shutdown_timer, shutdown_cb, NULL);
/* End of global variables section */

/* Bluetooth section */
static const struct bt_data advertisement[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
#if IS_ENABLED(CONFIG_USE_CSCS)
                  BT_UUID_16_ENCODE(BT_UUID_CSC_VAL)),
#elif IS_ENABLED(CONFIG_USE_FTMS)
                  BT_UUID_16_ENCODE(BT_UUID_FMS_VAL)),
    BT_DATA_BYTES(BT_DATA_SVC_DATA16, BT_UUID_16_ENCODE(BT_UUID_FMS_VAL), 0x01, 0x20, 0x00),  // indoor bike
#elif IS_ENABLED(CONFIG_USE_CPS)
                  BT_UUID_16_ENCODE(BT_UUID_CPS_VAL)),
#endif
};

static const struct bt_data scan_response[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

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
  if((curr_time - prev_time) > CONFIG_SWITCH_DEBOUNCE_TIME_MS)
  {
    k_timer_start(&shutdown_timer, K_SECONDS(CONFIG_SHUTDOWN_TIME_S), K_NO_WAIT);
    uint16_t time = (uint16_t)((curr_time - prev_time) & 0xFFFF);
    update_service_data(time);
#if IS_ENABLED(CONFIG_DEBUG_INFO)
    gpio_pin_toggle_dt(&user_led);
#endif
    prev_time = curr_time;
  }
}
/* End of GPIO section */

void shutdown_cb(struct k_timer* timer)
{
#if IS_ENABLED(CONFIG_DEBUG_INFO)
  gpio_pin_set_dt(&user_led, 0);
#endif
  retained_update();
  gpio_pin_interrupt_configure_dt(&user_switch, GPIO_INT_DISABLE);
  gpio_pin_interrupt_configure_dt(&user_switch, GPIO_INT_LEVEL_LOW);
  sys_poweroff();
  while(1);
}

int main(void)
{
  LOG_INF("Start of main.");
  int err;

#if IS_ENABLED(CONFIG_DEBUG_INFO)
  if(!gpio_is_ready_dt(&user_led))
  {
    return 0;
  }
  gpio_pin_configure_dt(&user_led, GPIO_OUTPUT_ACTIVE);
#endif
  if(!gpio_is_ready_dt(&user_switch))
  {
    return 0;
  }
  gpio_pin_configure_dt(&user_switch, GPIO_INPUT);
  gpio_pin_interrupt_configure_dt(&user_switch, GPIO_INT_EDGE_FALLING);

  gpio_init_callback(&switch_cb_data, switch_cb_func, BIT(user_switch.pin));
  gpio_add_callback_dt(&user_switch, &switch_cb_data);

  retained_validate();

  init_service_data();

  k_timer_start(&shutdown_timer, K_SECONDS(CONFIG_SHUTDOWN_TIME_S), K_NO_WAIT);

  err = bt_enable(NULL);
  if(err)
  {
    printk("Bluetooth init failed (err %d)\n", err);
    return 0;
  }

  bt_ready();

  while(1)
  {
    if(send_notify_flag)
    {
      notify_service();
    }
    k_sleep(K_SECONDS(1));
  }
  return 0;
}

/* Test thread for simulating spinning wheel. */
/*
uint16_t millis = 50;
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