/* cp_service.h - Cycling Power Service module */

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

#define CPF_PEDAL_POWER (1 << 0)
#define CPF_ACC_TORQUE (1 << 1)
#define CPF_WHEEL_REV (1 << 2)
#define CPF_CRANK_REV (1 << 3)
#define CPF_EXTREME_MAGNITUDE (1 << 4)
#define CPF_EXTREME_ANGLE (1 << 5)
#define CPF_DEAD_SPOT_ANGLE (1 << 6)
#define CPF_ACC_ENERGY (1 << 7)
#define CPF_OFFSET_COMPENS_INDX (1 << 8)
#define CPF_OFFSET_COMPENS (1 << 9)
#define CPF_CPM_MASKING (1 << 10)
#define CPF_MULTI_SENSOR (1 << 11)
#define CPF_CRANK_LENGTH (1 << 12)
#define CPF_CHAIN_LENGTH (1 << 13)
#define CPF_CHAIN_WEIGTH (1 << 14)
#define CPF_SPAN_LENGTH (1 << 15)
#define CPF_SENSOR_MEASUREMENT (1 << 16)
#define CPF_INST_DIRECTION (1 << 17)
#define CPF_FACTORY_CALIB_DATE (1 << 18)
#define CPF_ENH_OFFSET (1 << 19)
#define CPF_DIST_SYS_UNSPEC (0 << 20)
#define CPF_DIST_SYS_NOT_FOR_DIST (1 << 20)
#define CPF_DIST_SYS_FOR_DIST (2 << 20)

#define CPM_FLAG_WHEEL_REV (1 << 4)
#define CPM_FLAG_CRANK_REV (1 << 5)

typedef struct __attribute__((packed))
{
  uint16_t flags;
  int16_t inst_power;
  //uint32_t wheel_revolution;
  //uint16_t last_wheel_event_time;
  uint16_t crank_revolution;
  uint16_t last_crank_event_time;
  /* CRC used to validate the retained data.  This must be
   * stored little-endian, and covers everything up to but not
   * including this field.
   */
  uint32_t crc;
} cp_measurement_t;

extern uint16_t send_notify_flag;

void notify_service(void);
void update_service_data(uint16_t time);
void init_service_data(void);
void retained_validate(void);
void retained_update(void);

#endif // CSC_SERVICE_H