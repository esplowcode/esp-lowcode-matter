/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

typedef struct {
    uint8_t sda_pin;
    uint8_t scl_pin;
    uint8_t address;
    uint16_t freq_khz;
} temperature_sensor_sht30_config_t;

int temperature_sensor_sht30_init(void* i2c_dev, temperature_sensor_sht30_config_t *config);
int temperature_sensor_sht30_get_celsius(void* i2c_dev, float *temperature);

 #ifdef __cplusplus
 }
 #endif
