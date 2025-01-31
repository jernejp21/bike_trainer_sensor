/* main.c - Application main entry point */

/*
 * Copyright (c) 2025 Jernej Pangerc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FTM_SERVICE_H
#define FTM_SERVICE_H

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/types.h>

#define FMF_AVG_SPEED (1 << 0) /* Average Speed Supported */
#define FMF_CADENCE (1 << 1) /* Cadence Supported */
#define FMF_TOTAL_DIST (1 << 2) /* Total Distance Supported */
#define FMF_INCLINATION (1 << 3) /* Inclination Supported */
#define FMF_ELEVATION_GAIN (1 << 4) /* Elevation Gain Supported */
#define FMF_PACE (1 << 5) /* Pace Supported */
#define FMF_STEP_COUNT (1 << 6) /* Step Count Supported */
#define FMF_RESISTANCE_LEVEL (1 << 7) /* Resistance Level Supported */
#define FMF_STRIDE_COUNT (1 << 8) /* Stride Count Supported */
#define FMF_EXPENDED_ENERGY (1 << 9) /* Expended Energy Supported */
#define FMF_HEAR_RATE_MEASURMENT (1 << 10) /* Heart Rate Measurement Supported */
#define FMF_METABOLIC_EQV (1 << 11) /* Metabolic Equivalent Supported */
#define FMF_ELAPSED_TIME (1 << 12) /* Elapsed Time Supported */
#define FMF_REMAINING_TIME (1 << 13) /* Remaining Time Supported */
#define FMF_POWER_MEASURMENT (1 << 14) /* Power Measurement Supported */
#define FMF_FOCE_ON_BELT_POWER_OUT (1 << 15) /* Force on Belt and Power Output Supported */
#define FMF_USER_DATA_RET (1 << 16) /* User Data Retention Supported */

#define TSF_SPEED_TARGET (1 << 0) /* Speed Target Setting Supported */
#define TSF_INCLINATION_TARGET (1 << 1) /* Inclination Target Setting Supported */
#define TSF_RESISTANCE_TARGET (1 << 2) /* Resistance Target Setting Supported */
#define TSF_POWER_TARGET (1 << 3) /* Power Target Setting Supported */
#define TSF_HEART_RATE_TARGET (1 << 4) /* Heart Rate Target Setting Supported */
#define TSF_TARGET_EXPENDED_ENERGY (1 << 5) /* Targeted Expended Energy Configuration Supported */
#define TSF_TARGET_STEP_NUMBER (1 << 6) /* Targeted Step Number Configuration Supported */
#define TSF_STRIDE_STEP_NUMBER (1 << 7) /* Targeted Stride Number Configuration Supported */
#define TSF_TARGET_DISTANCE (1 << 8) /* Targeted Distance Configuration Supported */
#define TSF_TARGET_TRAIN_TIME (1 << 9) /* Targeted Training Time Configuration Supported */
#define TSF_TARGET_TIME_ZONES2 (1 << 10) /* Targeted Time in Two Heart Rate Zones Configuration Supported */
#define TSF_TARGET_TIME_ZONES3 (1 << 11) /* Targeted Time in Three Heart Rate Zones Configuration Supported */
#define TSF_TARGET_TIME_ZONES5 (1 << 12) /* Targeted Time in Five Heart Rate Zones Configuration Supported */
#define TSF_INDOOR_BIKE_SIM_PARAM (1 << 13) /* Indoor Bike Simulation Parameters Supported */
#define TSF_WHEEL_CIRCUMFERENCE (1 << 14) /* Wheel Circumference Configuration Supported */
#define TSF_SPINDOWN_CTRL (1 << 15) /* Spin Down Control Supported */
#define TSF_TARGET_CADENCE (1 << 16) /* Targeted Cadence Configuration Supported */

#define IBD_FLAG_INST_SPEED_PRESENT (0 << 0) /* Instantaneous Speed field present */
#define IBD_FLAG_INST_SPEED_NOT_PRESENT (1 << 0) /* Instantaneous Speed field not present */
#define IBD_FLAG_AVG_SPEED (1 << 1) /* Average Speed present */
#define IBD_FLAG_INST_CADENCE (1 << 2) /* Instantaneous Cadence present */
#define IBD_FLAG_AVG_CADENCE (1 << 3) /* Average Cadence present */
#define IBD_FLAG_TOTAL_DIST (1 << 4) /* Total Distance Present */
#define IBD_FLAG_RESISTANCE_LVL (1 << 5) /* Resistance Level Present */
#define IBD_FLAG_INST_POWER (1 << 6) /* Instantaneous Power Present */
#define IBD_FLAG_AVG_POWER (1 << 7) /* Average Power Present */
#define IBD_FLAG_EXPENDED_ENERGY (1 << 8) /* Expended Energy Present */
#define IBD_FLAG_HEART_RATE (1 << 9) /* Heart Rate Present */
#define IBD_FLAG_METABOLIC_EQV (1 << 10) /* Metabolic Equivalent Present */
#define IBD_FLAG_ELAPSED_TIME (1 << 11) /* Elapsed Time Present */
#define IBD_FLAG_REMAINING_TIME (1 << 12) /* Remaining Time Present */


typedef struct __attribute__((packed))
{
  uint16_t flags;
  uint16_t inst_speed; /* km/h, resolution 0.01 km/h */
  uint16_t avg_speed; /* km/h, resolution 0.01 km/h */
  //uint16_t inst_cadence; /* 1/min, resolution 0.5 min^-1*/
  //uint16_t avg_cadence; /* 1/min, resolution 0.5 min^-1*/
  uint8_t total_dist[3]; /* m, resolution 1 m */
  //uint8_t resistance_lvl; /* no unit, resolution 0.1 */
  //int16_t inst_power; /* W, resolution 1 W */
  //int16_t avg_power; /* W, resolution 1 W */
  //uint16_t total_energy; /* cal, resolution 1 cal */
  //uint16_t energy_per_hour; /* cal, resolution 1 cal */
  //uint8_t energy_per_minute; /* cal, resolution 1 cal */
  //uint8_t heart_rate; /* BPM, resolution 1 BMP */
  //uint8_t metabolic_eqv;
  uint16_t elapsed_time; /* s, resolution 1 s */
  //uint16_t remaining_time; /* s, resolution 1 s */
  /* CRC used to validate the retained data.  This must be
   * stored little-endian, and covers everything up to but not
   * including this field.
   */
  uint32_t crc;
} indoor_bike_data_t;

extern uint16_t send_notify_flag;

void notify_service(void);
void update_service_data(uint16_t time);
void init_service_data(void);
void retained_validate(void);
void retained_update(void);

#endif  // CSC_SERVICE_H