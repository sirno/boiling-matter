
#include "esp_err.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

#include <ds18x20.h>
#include <app_priv.h>

static const char *TAG = "app_driver";

#define ONEWIRE_BUS_GPIO (static_cast<gpio_num_t>(CONFIG_ONEWIRE_DATA_PIN))
#define ONEWIRE_MAX_DS18B20 1

#define VOLTAGE_READ_PIN (static_cast<adc_channel_t>(CONFIG_VOLTAGE_READ_PIN))
#define VOLTAGE_SMOOTHING_WINDOW_SIZE 10

#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC1_CHAN0 ADC_CHANNEL_0

#define ADC_NUM_SAMPLES 5
#define ADC_EMA_ALPHA 0.2f

static size_t n_sensors;
static onewire_addr_t sensor_addrs[ONEWIRE_MAX_DS18B20];

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t voltage_handle;
static esp_timer_handle_t timer;

#ifdef CONFIG_USE_WINDOW_FOR_VOLTAGE_SMOOTHING
static float voltage_window[VOLTAGE_SMOOTHING_WINDOW_SIZE] = {0};
static int voltage_window_index = 0;


static float smooth_voltage(float voltage) {
  voltage_window[voltage_window_index] = voltage;
  voltage_window_index = (voltage_window_index + 1) % VOLTAGE_SMOOTHING_WINDOW_SIZE;

  int count = 0;
  float sum = 0;
  for (int i = 0; i < VOLTAGE_SMOOTHING_WINDOW_SIZE; i++) {
    if (voltage_window[i] > 0) {
      sum += voltage_window[i];
      count++;
    }
  }

  if (count == 0) {
    return voltage;
  }

  return sum / count;
}
#endif

#ifdef CONFIG_USE_EMA_FOR_VOLTAGE_SMOOTHING
static float ema_voltage = 0;

static float smooth_voltage(float voltage) {
  if (ema_voltage <= 0) {
    ema_voltage = voltage;
    return voltage;
  }

  ema_voltage = ADC_EMA_ALPHA * voltage + (1 - ADC_EMA_ALPHA) * ema_voltage;
  return ema_voltage;
}
#endif

#ifndef CONFIG_USE_WINDOW_FOR_VOLTAGE_SMOOTHING
#ifndef CONFIG_USE_EMA_FOR_VOLTAGE_SMOOTHING
static float smooth_voltage(float voltage) {
  return voltage;
}
#endif
#endif

static esp_err_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return ret;
}

esp_err_t init_voltage_adc() {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };

    esp_err_t err = adc_oneshot_new_unit(&init_config1, &adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed, err:%d", err);
        return err;
    }

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(adc_handle, ADC1_CHAN0, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed, err:%d", err);
        return err;
    };

    err = adc_calibration_init(ADC_UNIT_1, ADC1_CHAN0, ADC_ATTEN, &voltage_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_calibration_init failed, err:%d", err);
        return err;
    }

    return ESP_OK;
};

esp_err_t init_ds18x20_devices() {
  esp_err_t err = ds18x20_scan_devices(ONEWIRE_BUS_GPIO, sensor_addrs, ONEWIRE_MAX_DS18B20, &n_sensors);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "ds18x20_scan_devices failed, err:%d", err);
  }
  return err;
}

static void timer_cb_internal(void *arg) {
  auto *ctx = (sensor_config_t *)arg;
  if (!(ctx && ctx->batvoltage.cb && ctx->temperature.cb)) {
    ESP_LOGE(TAG, "invalid timer callback context");
    return;
  }

  float temperatures[ONEWIRE_MAX_DS18B20];

  ESP_ERROR_CHECK(ds18x20_measure_and_read_multi(ONEWIRE_BUS_GPIO, sensor_addrs, n_sensors, temperatures));

  for (int i = 0; i < n_sensors; i++) {
    ctx->temperature.cb(ctx->temperature.endpoint_id, temperatures[i], NULL);
  }

  int cummulative_voltage = 0;

  for (int i = 0; i < ADC_NUM_SAMPLES; i++) {
    int adc_raw;
    int voltage;

    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC1_CHAN0, &adc_raw));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(voltage_handle, adc_raw, &voltage));

    cummulative_voltage += voltage;
  };

  int voltage = cummulative_voltage / ADC_NUM_SAMPLES;
  ctx->batvoltage.cb(ctx->batvoltage.endpoint_id, smooth_voltage(2.0 * voltage), NULL);
}

esp_err_t init_sensor_readouts(sensor_config_t *config) {
  esp_timer_create_args_t args = {
      .callback = timer_cb_internal,
      .arg = config,
  };

  esp_err_t err = esp_timer_create(&args, &timer);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_timer_create failed, err:%d", err);
    return err;
  }

  err = esp_timer_start_periodic(timer, config->interval_ms * 1000);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_timer_start_periodic failed: %d", err);
    return err;
  }

  return ESP_OK;
};
