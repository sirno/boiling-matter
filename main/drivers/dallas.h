// This file implements the DS18B20 sensor driver.
// This is implemented keeping the Matter requirements in mind.
#pragma once

#include <esp_err.h>
#include <esp_timer.h>
#include <ds18x20.h>

using dallas_sensor_cb_t = void (*)(uint16_t endpoint_id, float temp, void *user_data);

typedef struct {
    // This callback function will be called periodically to report the temperature.
    dallas_sensor_cb_t cb = NULL;
    // endpoint_id associated with temperature sensor
    uint16_t endpoint_id;
    // user data
    void *user_data = NULL;
    // polling interval in milliseconds, defaults to 5000 ms
    uint32_t interval_ms = 5000;
} dallas_sensor_config_t;

/**
 * @brief Initialize sensor driver. This function should be called only once
 *
 * @param config sensor configurations. This should last for the lifetime of the driver
 *               as driver layer do not make a copy of this object.
 *
 * @return esp_err_t - ESP_OK on success,
 *                     ESP_ERR_INVALID_ARG if config is NULL
 *                     ESP_ERR_INVALID_STATE if driver is already initialized
 *                     appropriate error code otherwise
 */
esp_err_t dallas_sensor_init(dallas_sensor_config_t *config);
