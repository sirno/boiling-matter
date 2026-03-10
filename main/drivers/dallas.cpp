/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>

#include <drivers/dallas.h>

#define ONEWIRE_BUS_GPIO (static_cast<gpio_num_t>(CONFIG_ONEWIRE_DATA_PIN))
#define ONEWIRE_MAX_DS18B20 1

static const char *TAG = "ds18b20";

// TODO: support multiple endpoints for multiple DS18B20 devices. For now, we
// just support one endpoint and report all DS18B20 readings to this endpoint.
typedef struct {
  size_t n_sensors;
  dallas_sensor_config_t *config;
  onewire_addr_t sensor_addresses[ONEWIRE_MAX_DS18B20];
  esp_timer_handle_t timer;
  bool is_initialized;
} dallas_sensor_ctx_t;

static dallas_sensor_ctx_t s_ctx;

// Scan the OneWire bus for DS18B20 devices and create device handles
static void init_ds18b20_devices() {
  ESP_ERROR_CHECK(ds18x20_scan_devices(ONEWIRE_BUS_GPIO, s_ctx.sensor_addresses, ONEWIRE_MAX_DS18B20, &s_ctx.n_sensors));
}

static void timer_cb_internal(void *arg) {
  auto *ctx = (dallas_sensor_ctx_t *)arg;
  if (!(ctx && ctx->config && ctx->is_initialized)) {
    return;
  }

  float temperatures[ONEWIRE_MAX_DS18B20];

  ESP_ERROR_CHECK(ds18x20_measure_and_read_multi(ONEWIRE_BUS_GPIO, ctx->sensor_addresses, ctx->n_sensors, temperatures));

  for (int i = 0; i < ctx->n_sensors; i++) {
    ESP_LOGI(TAG, "temperature read from DS18B20[%d]: %.2fC", i, temperatures[i]);

    ctx->config->cb(ctx->config->endpoint_id, temperatures[i],
                    ctx->config->user_data);
  }
}

esp_err_t dallas_sensor_init(dallas_sensor_config_t *config) {
  /* Validate Arguments */
  if (config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (s_ctx.is_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Setup OneWire bus and scan for DS18B20 devices */
  init_ds18b20_devices();

  // error if no DS18B20 device found, as this driver is only for DS18B20 sensor
  if (s_ctx.n_sensors == 0) {
    ESP_LOGW(TAG, "No DS18B20 device found on the bus");
    return ESP_ERR_NOT_FOUND;
  }

  s_ctx.config = config;

  /* Setup timer for periodic temperature measurement */
  esp_timer_create_args_t args = {
      .callback = timer_cb_internal,
      .arg = &s_ctx,
  };

  esp_err_t err = esp_timer_create(&args, &s_ctx.timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_timer_create failed, err:%d", err);
    return err;
  }

  err = esp_timer_start_periodic(s_ctx.timer, config->interval_ms * 1000);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_timer_start_periodic failed: %d", err);
    return err;
  }

  s_ctx.is_initialized = true;

  return ESP_OK;
}
