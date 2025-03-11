// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>

#include <esp_log.h>
#include <hal/i2c_types.h>

#include "app_priv.h"

#include <low_code.h>
#include <system.h>
#include <button_driver.h>
#include <simple_i2c_master.h>
#include <temperature_sensor_sht30.h>

#define BUTTON_GPIO_NUM (gpio_num_t)9

#define I2C_SCL_IO (gpio_num_t)1
#define I2C_SDA_IO (gpio_num_t)2

static const char *TAG = "app_driver";

static i2c_dev_t *dev;

static void app_driver_trigger_factory_reset_button_callback(void *arg, void *data)
{
    /* Update by sending event */
    low_code_event_t event = {
        .event_type = LOW_CODE_EVENT_FACTORY_RESET
    };

    low_code_event_to_system(&event);
    printf("%s: Factory reset triggered\n", TAG);
}

static void app_driver_temperature_report(float temp)
{
    int16_t temperature = temp*100;
    /* Update the feature */
    low_code_feature_data_t update_data = {
        .details = {
            .endpoint_id = 1,
            .low_level = {
                .matter = {
                    .cluster_id = 0x0402,
                    .attribute_id = 0x0000,
                },
            }
        },
        .value = {
            .type = LOW_CODE_VALUE_TYPE_INTEGER,
            .value_len = sizeof(int16_t),
            .value = (uint8_t*)&temperature,
        },
    };

    low_code_feature_update_to_system(&update_data);
}

int app_driver_read_temperature()
{
    float temperature = 0.0;
    temperature_sensor_sht30_get_celsius(dev, &temperature);
    system_delay_ms(100);
    app_driver_temperature_report(temperature);
    printf("Temperature: %f\n", temperature);
    return 0;
}

int app_driver_init()
{
    printf("%s: Initializing driver\n", TAG);

    /* Initialize button */
    button_config_t btn_cfg = {
        .gpio_num = BUTTON_GPIO_NUM,
        .pullup_en = 1,
        .active_level = 0,
    };
    button_handle_t btn_handle = button_driver_create(&btn_cfg);
    if (!btn_handle) {
        printf("Failed to create the button");
        return -1;
    }

    /* Register callback to factory reset the device on button long press */
    button_driver_register_cb(btn_handle, BUTTON_LONG_PRESS_UP, app_driver_trigger_factory_reset_button_callback, NULL);

    /* Initialize I2C */
    dev = i2c_master_init(I2C_NUM_0, I2C_SCL_IO, I2C_SDA_IO);

    /* Initialize sensor */
    temperature_sensor_sht30_config_t config = {
        .sda_pin = I2C_SCL_IO,
        .scl_pin = I2C_SDA_IO,
        .address = 0x44
    };
    // temperature_sensor_sht30_init(dev, &config);
    return 0;
}

int app_driver_feature_update()
{
    printf("%s: Feature update\n", TAG);

    /* Nothing to do here. The device reports the feature data by itself. */
    return 0;
}

int app_driver_event_handler(low_code_event_t *event)
{
    printf("%s: Received event: %d\n", TAG, event->event_type);

    switch (event->event_type) {
        case LOW_CODE_EVENT_SETUP_MODE_START:
            printf("%s: Setup mode started\n", TAG);
            break;
        case LOW_CODE_EVENT_SETUP_MODE_END:
            printf("%s: Setup mode ended\n", TAG);
            break;
        case LOW_CODE_EVENT_SETUP_DEVICE_CONNECTED:
            printf("%s: Device connected during setup\n", TAG);
            break;
        case LOW_CODE_EVENT_SETUP_STARTED:
            printf("%s: Setup process started\n", TAG);
            break;
        case LOW_CODE_EVENT_SETUP_SUCCESSFUL:
            printf("%s: Setup process successful\n", TAG);
            break;
        case LOW_CODE_EVENT_SETUP_FAILED:
            printf("%s: Setup process failed\n", TAG);
            break;
        case LOW_CODE_EVENT_NETWORK_CONNECTED:
            printf("%s: Network connected\n", TAG);
            break;
        case LOW_CODE_EVENT_NETWORK_DISCONNECTED:
            printf("%s: Network disconnected\n", TAG);
            break;
        case LOW_CODE_EVENT_OTA_STARTED:
            printf("%s: OTA update started\n", TAG);
            break;
        case LOW_CODE_EVENT_OTA_STOPPED:
            printf("%s: OTA update stopped\n", TAG);
            break;
        case LOW_CODE_EVENT_READY:
            printf("%s: Device is ready\n", TAG);
            break;
        case LOW_CODE_EVENT_IDENTIFICATION_START:
            printf("%s: Identification started\n", TAG);
            break;
        case LOW_CODE_EVENT_IDENTIFICATION_STOP:
            printf("%s: Identification stopped\n", TAG);
            break;
        case LOW_CODE_EVENT_TEST_MODE_LOW_CODE:
            printf("%s: Low code test mode is triggered for subtype: %d\n", TAG, (int)*((int*)(event->event_data)));
            break;
        case LOW_CODE_EVENT_TEST_MODE_COMMON:
            printf("%s: common test mode triggered\n", TAG);
            break;
        case LOW_CODE_EVENT_TEST_MODE_BLE:
            printf("%s: ble test mode triggered\n", TAG);
            break;
        case LOW_CODE_EVENT_TEST_MODE_SNIFFER:
            printf("%s: sniffer test mode triggered\n", TAG);
            break;
        default:
            printf("%s: Unhandled event type: %d\n", TAG, event->event_type);
            break;
    }

    return 0;
}
