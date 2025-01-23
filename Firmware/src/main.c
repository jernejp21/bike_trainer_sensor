/* main.c - Application main entry point */

/*
 * Copyright (c) 2025 Jernej Pangerc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

int main(void)
{
  LOG_INF("Start of main.");

  while(1)
  {
    k_sleep(K_SECONDS(1));
  }
  return 0;
}
