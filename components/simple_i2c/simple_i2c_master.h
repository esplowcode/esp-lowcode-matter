/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <soc/i2c_struct.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "hal/gpio_types.h"

/**
 * @brief Read from I2C device
 *
 * @param dev               I2C device
 * @param device_addr       I2C device address (7-bit)
 * @param data_rd           Buffer to hold data to be read
 * @param size              Size of data to be read in bytes
 * @param ticks_to_wait     Operation timeout in CPU cycles. Set to -1 to wait forever.
 *
 * @return esp_err_t    ESP_OK when successful
 *
 * @note the SIMPLE I2C does not support 10-bit I2C device addresses.
 */
esp_err_t simple_i2c_master_read_from_device(i2c_dev_t *dev, uint16_t device_addr,
                                              uint8_t *data_rd, size_t size,
                                              int32_t ticks_to_wait);

/**
 * @brief Write to I2C device
 *
 * @param dev               I2C device
 * @param device_addr       I2C device address (7-bit)
 * @param data_wr           Buffer which holds the data to be written
 * @param size              Size of data to be written in bytes
 * @param ticks_to_wait     Operation timeout in CPU cycles. Set to -1 to wait forever.
 *
 * @return esp_err_t    ESP_OK when successful
 *
 * @note the SIMPLE I2C does not support 10-bit I2C device addresses.
 */
esp_err_t simple_i2c_master_write_to_device(i2c_dev_t *dev, uint16_t device_addr,
                                             const uint8_t *data_wr, size_t size,
                                             int32_t ticks_to_wait);

/**
 * @brief Write to and then read from an I2C device in a single transaction
 *
 * @param dev               I2C device
 * @param device_addr       I2C device address (7-bit)
 * @param data_wr           Buffer which holds the data to be written
 * @param write_size        Size of data to be written in bytes
 * @param data_rd           Buffer to hold data to be read
 * @param read_size         Size of data to be read in bytes
 * @param ticks_to_wait     Operation timeout in CPU cycles. Set to -1 to wait forever.
 *
 * @return esp_err_t    ESP_OK when successful
 *
 * @note the SIMPLE I2C does not support 10-bit I2C device addresses.
 */
esp_err_t simple_i2c_master_write_read_device(i2c_dev_t *dev, uint16_t device_addr,
                                               const uint8_t *data_wr, size_t write_size,
                                               uint8_t *data_rd, size_t read_size,
                                               int32_t ticks_to_wait);

/**
 * @brief Init i2c master
 * 
 * @param i2c_port          I2C Port
 * @param scl_io            I2C SCL GPIO Pin
 * @param sda_io            I2C SDA GPIO Pin
 * 
 * @return i2c_dev_t        I2C DEV Handle. NULL if I2C port not supported.alignas\
 * 
 * @note only I2C_NUM_0 is supported
 */
i2c_dev_t *i2c_master_init(int i2c_port, gpio_num_t scl_io, gpio_num_t sda_io);

#ifdef __cplusplus
}
#endif
