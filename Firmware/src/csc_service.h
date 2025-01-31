/* main.c - Application main entry point */

/*
 * Copyright (c) 2025 Jernej Pangerc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CSC_SERVICE_H
#define CSC_SERVICE_H

#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/uuid.h>

typedef struct __attribute__((packed))
{
  uint8_t flags;
  uint32_t wheel_revolution;
  uint16_t last_event_time;
  /* CRC used to validate the retained data.  This must be
   * stored little-endian, and covers everything up to but not
   * including this field.
   */
  uint32_t crc;
} csc_measurement_t;

extern uint16_t send_notify_flag;

void notify_service(void);
void update_service_data(uint16_t time);
void init_service_data(void);
void retained_validate(void);
void retained_update(void);

#endif // CSC_SERVICE_H