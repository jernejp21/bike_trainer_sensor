/* kinetic_road_machine.h - Power-speed ralation. */

/*
 * Copyright (c) 2025 Jernej Pangerc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef KINETIC_ROAD_MACHINE_H
#define KINETIC_ROAD_MACHINE_H

#include <zephyr/types.h>

/* Power at every 10 km/h from 0 to 60 km/h. */
/* Example: {0, 10, 50, 100, 200, 500, 1000} */
/* 0 W at 0 km/h, 10 W at 10 km/h, 50 W at 20 km/h, 100 W at 30 km/h  etc. */
uint16_t power_curve[] = {0, 50, 100, 200, 400, 657, 1100};

#endif  // KINETIC_ROAD_MACHINE_H