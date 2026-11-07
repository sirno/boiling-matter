/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

#include <esp_err.h>
#include <esp_matter.h>

// Called from the esp_timer task. `value` is NAN when the sensor could not be read.
// Temperature callbacks receive °C, battery callbacks receive the (smoothed) battery voltage in mV.
using sensor_cb_t = void (*)(uint16_t endpoint_id, float value, void *user_data);

typedef struct {
    sensor_cb_t cb;
    uint16_t endpoint_id;
} sensor_config_endpoint_t;

typedef struct {
    uint32_t interval_ms;         // temperature sampling period
    uint32_t battery_interval_ms; // battery sampling period (sampled on the first cycle, then at this period)
    sensor_config_endpoint_t temperature;
    sensor_config_endpoint_t batvoltage;
} sensor_config_t;

// Resolves the battery ADC channel and calibration. On failure the battery is never sampled.
esp_err_t init_voltage_adc();
// Finds the DS18B20 and applies the configured resolution. ESP_ERR_NOT_FOUND is not fatal:
// the bus is rescanned on every sampling cycle until a sensor shows up.
esp_err_t init_ds18x20_devices();
// Starts periodic sampling. `config` must outlive the driver.
esp_err_t init_sensor_readouts(sensor_config_t *config);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include "esp_openthread_types.h"
#endif

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG()                                           \
    {                                                                                   \
        .radio_mode = RADIO_MODE_NATIVE,                                                \
    }

#define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG()                                            \
    {                                                                                   \
        .host_connection_mode = HOST_CONNECTION_MODE_NONE,                              \
    }

#define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG()                                            \
    {                                                                                   \
        .storage_partition_name = "nvs", .netif_queue_size = 10, .task_queue_size = 10, \
    }
#endif
