/*
   Sensor driver: DS18B20 temperature over 1-Wire and battery voltage over the ADC.

   Everything runs from esp_timer callbacks and nothing blocks for the DS18B20 conversion:
   a periodic timer starts the conversion and a one-shot timer reads the result once it is
   ready. In between, the CPU can light-sleep and other esp_timer users (e.g. NimBLE during
   commissioning) keep running.
*/

#include <inttypes.h>
#include <math.h>

#include <driver/gpio.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_check.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#if CONFIG_PM_ENABLE
#include <esp_pm.h>
#endif

#include <app_priv.h>
#include <ds18x20.h>

static const char *TAG = "app_driver";

static sensor_config_t *s_config;
static esp_timer_handle_t s_sample_timer;
static esp_timer_handle_t s_read_timer;

/* ------------------------------------------------------------------------------------------ */
/* DS18B20                                                                                    */
/* ------------------------------------------------------------------------------------------ */

#define ONEWIRE_BUS_GPIO (static_cast<gpio_num_t>(CONFIG_ONEWIRE_DATA_PIN))

// The bus is scanned for up to this many devices; the first DS18B20 found is used.
#define ONEWIRE_MAX_DEVICES 4
// After this many consecutive failed readings, MeasuredValue is reported as null and the bus
// is rescanned on every following cycle.
#define TEMP_MAX_CONSECUTIVE_FAILURES 3
// Temperature register value after power-on, before a conversion has completed (85.0 °C).
#define DS18B20_POWER_ON_RAW 0x0550
// Added to the datasheet's maximum conversion time.
#define DS18B20_CONVERSION_MARGIN_MS 10

static_assert(CONFIG_DS18B20_RESOLUTION >= 9 && CONFIG_DS18B20_RESOLUTION <= 12,
              "DS18B20 resolution must be 9 to 12 bits");

// Scratchpad byte offsets (DS18B20 datasheet, memory map).
enum : uint8_t { SP_TEMP_LSB = 0, SP_TEMP_MSB = 1, SP_TH = 2, SP_TL = 3, SP_CONFIG = 4 };

// Configuration register layout: 0 R1 R0 1 1 1 1 1, where R1:R0 = resolution - 9.
constexpr uint8_t ds18b20_config_for(int bits) { return static_cast<uint8_t>(((bits - 9) << 5) | 0x1F); }
constexpr bool ds18b20_config_valid(uint8_t config) { return (config & 0x9F) == 0x1F; }
constexpr int ds18b20_bits_from_config(uint8_t config) { return 9 + ((config >> 5) & 0x03); }
// Maximum conversion time is 93.75 ms at 9 bits and doubles per bit, up to 750 ms at 12 bits.
constexpr uint32_t ds18b20_conversion_ms(int bits)
{
    return (750u >> (12 - bits)) + 1 + DS18B20_CONVERSION_MARGIN_MS;
}

static onewire_addr_t s_sensor_addr;    // DS18X20_ANY (skip ROM) when it is the only device on the bus
static bool s_sensor_present;
static int s_resolution_bits = 12;      // resolution the sensor actually accepted
static uint8_t s_expected_config;       // configuration register read back after setup
static int s_consecutive_failures;
static float s_last_valid_temp = NAN;
static bool s_power_on_value_pending;   // an unconfirmed 85.0 °C reading was just discarded

#if CONFIG_PM_ENABLE
// 1-Wire is bit-banged with busy-wait delays. Keep the CPU clock fixed while the bus is in use so
// a DFS frequency switch cannot stretch or shrink the slot timing.
static esp_pm_lock_handle_t s_bus_pm_lock;
#endif

class BusLock {
public:
    BusLock()
    {
#if CONFIG_PM_ENABLE
        if (s_bus_pm_lock) {
            esp_pm_lock_acquire(s_bus_pm_lock);
        }
#endif
    }
    ~BusLock()
    {
#if CONFIG_PM_ENABLE
        if (s_bus_pm_lock) {
            esp_pm_lock_release(s_bus_pm_lock);
        }
#endif
    }
    BusLock(const BusLock &) = delete;
    BusLock &operator=(const BusLock &) = delete;
};

// Applies CONFIG_DS18B20_RESOLUTION and records what the sensor reports back. Sensors that ignore
// the setting keep working, using the conversion time of the resolution they report.
static void configure_resolution()
{
    BusLock lock;
    uint8_t sp[8];

    s_resolution_bits = 12;
    s_expected_config = 0;
    if (ds18x20_read_scratchpad(ONEWIRE_BUS_GPIO, s_sensor_addr, sp) != ESP_OK) {
        ESP_LOGW(TAG, "could not read DS18B20 configuration, assuming 12-bit");
        return;
    }

    const uint8_t wanted = ds18b20_config_for(CONFIG_DS18B20_RESOLUTION);
    if (sp[SP_CONFIG] != wanted) {
        // Keep the alarm registers and only change the resolution. This is not copied to the
        // sensor's EEPROM; if the sensor loses power, read_temperature() notices and reapplies it.
        uint8_t regs[3] = {sp[SP_TH], sp[SP_TL], wanted};
        if (ds18x20_write_scratchpad(ONEWIRE_BUS_GPIO, s_sensor_addr, regs) != ESP_OK ||
            ds18x20_read_scratchpad(ONEWIRE_BUS_GPIO, s_sensor_addr, sp) != ESP_OK) {
            ESP_LOGW(TAG, "could not set DS18B20 resolution, assuming 12-bit");
            return;
        }
    }

    s_expected_config = sp[SP_CONFIG];
    if (ds18b20_config_valid(sp[SP_CONFIG])) {
        s_resolution_bits = ds18b20_bits_from_config(sp[SP_CONFIG]);
    }
    if (s_resolution_bits != CONFIG_DS18B20_RESOLUTION) {
        ESP_LOGW(TAG, "DS18B20 runs at %d-bit resolution (configured %d)", s_resolution_bits,
                 CONFIG_DS18B20_RESOLUTION);
    }
}

static bool find_sensor()
{
    onewire_addr_t addrs[ONEWIRE_MAX_DEVICES];
    size_t found = 0;
    esp_err_t err;
    {
        BusLock lock;
        err = ds18x20_scan_devices(ONEWIRE_BUS_GPIO, addrs, ONEWIRE_MAX_DEVICES, &found);
    }
    if (err != ESP_OK) {
        return false;
    }

    // `found` counts every device on the bus, including those that did not fit in `addrs`.
    const size_t stored = found < ONEWIRE_MAX_DEVICES ? found : ONEWIRE_MAX_DEVICES;
    for (size_t i = 0; i < stored; i++) {
        if (static_cast<uint8_t>(addrs[i]) != DS18X20_FAMILY_DS18B20) {
            continue;
        }
        // Skip ROM saves 64 bit slots per transaction but only works with a single device.
        s_sensor_addr = (found == 1) ? DS18X20_ANY : addrs[i];
        s_sensor_present = true;
        configure_resolution();
        ESP_LOGI(TAG, "using DS18B20 %08" PRIx32 "%08" PRIx32 " (%u device(s) on bus), %d-bit",
                 static_cast<uint32_t>(addrs[i] >> 32), static_cast<uint32_t>(addrs[i]),
                 static_cast<unsigned>(found), s_resolution_bits);
        return true;
    }

    if (found > 0) {
        ESP_LOGW(TAG, "%u 1-Wire device(s) on bus, but no DS18B20", static_cast<unsigned>(found));
    }
    return false;
}

// Reads the result of the last conversion into *out_c (°C). Returns an error when the reading
// has to be discarded.
static esp_err_t read_temperature(float *out_c)
{
    uint8_t sp[8];
    esp_err_t err;
    {
        BusLock lock;
        // The scratchpad keeps the result, so a corrupted read (CRC error) can simply be repeated.
        err = ds18x20_read_scratchpad(ONEWIRE_BUS_GPIO, s_sensor_addr, sp);
        if (err != ESP_OK) {
            err = ds18x20_read_scratchpad(ONEWIRE_BUS_GPIO, s_sensor_addr, sp);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DS18B20 read failed: %s", esp_err_to_name(err));
        return err;
    }

    if (sp[SP_CONFIG] != s_expected_config) {
        // The sensor lost its configuration (e.g. a supply glitch reset it to the EEPROM default),
        // so the conversion we waited for may not have finished. Reapply and drop this sample.
        ESP_LOGW(TAG, "DS18B20 configuration changed (0x%02x), reconfiguring", sp[SP_CONFIG]);
        configure_resolution();
        return ESP_ERR_INVALID_STATE;
    }

    int16_t raw = static_cast<int16_t>((sp[SP_TEMP_MSB] << 8) | sp[SP_TEMP_LSB]);
    // Bits below the configured resolution are undefined.
    raw = static_cast<int16_t>(raw & ~((1 << (12 - s_resolution_bits)) - 1));
    const float temp = raw / 16.0f;

    if (temp < -55.0f || temp > 125.0f) {
        ESP_LOGW(TAG, "DS18B20 reading out of range: %.2f °C", temp);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // 85.0 °C is also the power-on value. Accept it when it continues from nearby readings or when
    // the next conversion confirms it (a probe really sitting at 85 °C).
    if (raw == DS18B20_POWER_ON_RAW) {
        const bool near_last = !isnan(s_last_valid_temp) && fabsf(s_last_valid_temp - 85.0f) <= 5.0f;
        if (!near_last && !s_power_on_value_pending) {
            ESP_LOGW(TAG, "discarding unconfirmed 85.00 °C (DS18B20 power-on value)");
            s_power_on_value_pending = true;
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    s_power_on_value_pending = false;

    s_last_valid_temp = temp;
    *out_c = temp;
    return ESP_OK;
}

static void temperature_failed()
{
    if (s_consecutive_failures >= TEMP_MAX_CONSECUTIVE_FAILURES) {
        return; // already reported as unknown
    }
    if (++s_consecutive_failures == TEMP_MAX_CONSECUTIVE_FAILURES) {
        ESP_LOGE(TAG, "DS18B20 unreadable %d times in a row, reporting temperature as unknown",
                 TEMP_MAX_CONSECUTIVE_FAILURES);
        s_sensor_present = false; // rescan from the next cycle on
        s_config->temperature.cb(s_config->temperature.endpoint_id, NAN, NULL);
    }
}

static void start_temperature_conversion()
{
    if (esp_timer_is_active(s_read_timer)) {
        return; // previous conversion still pending
    }
    if (!s_sensor_present && !find_sensor()) {
        temperature_failed();
        return;
    }

    esp_err_t err;
    {
        BusLock lock;
        err = ds18x20_measure(ONEWIRE_BUS_GPIO, s_sensor_addr, false);
        if (err != ESP_OK) {
            err = ds18x20_measure(ONEWIRE_BUS_GPIO, s_sensor_addr, false);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "could not start DS18B20 conversion: %s", esp_err_to_name(err));
        temperature_failed();
        return;
    }

    err = esp_timer_start_once(s_read_timer, ds18b20_conversion_ms(s_resolution_bits) * 1000ULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_start_once failed: %s", esp_err_to_name(err));
        temperature_failed();
    }
}

static void read_timer_cb(void *arg)
{
    float temp;
    if (read_temperature(&temp) != ESP_OK) {
        temperature_failed();
        return;
    }
    s_consecutive_failures = 0;
    ESP_LOGD(TAG, "DS18B20: %.2f °C", temp);
    s_config->temperature.cb(s_config->temperature.endpoint_id, temp, NULL);
}

esp_err_t init_ds18x20_devices()
{
#if CONFIG_PM_ENABLE
    if (!s_bus_pm_lock && esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "onewire", &s_bus_pm_lock) != ESP_OK) {
        ESP_LOGW(TAG, "could not create 1-Wire PM lock; bus timing may suffer under DFS");
    }
#endif
    return find_sensor() ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/* ------------------------------------------------------------------------------------------ */
/* Battery voltage                                                                            */
/* ------------------------------------------------------------------------------------------ */

#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_NUM_SAMPLES 16
#define ADC_EMA_ALPHA 0.2f
#define VOLTAGE_SMOOTHING_WINDOW_SIZE 10
// The battery is measured through a 1:1 resistor divider.
#define BATTERY_DIVIDER_RATIO 2.0f

static adc_unit_t s_adc_unit;
static adc_channel_t s_adc_channel;
static adc_cali_handle_t s_adc_cali;
static bool s_adc_ready;
static uint32_t s_battery_every_n_cycles = 1;
static uint32_t s_cycles_until_battery; // 0: sample on the next cycle

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
    ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(CONFIG_VOLTAGE_READ_PIN, &s_adc_unit, &s_adc_channel), TAG,
                        "GPIO%d is not an ADC pin", CONFIG_VOLTAGE_READ_PIN);
    ESP_RETURN_ON_ERROR(adc_calibration_init(s_adc_unit, s_adc_channel, ADC_ATTEN, &s_adc_cali), TAG,
                        "ADC calibration unavailable");
    s_adc_ready = true;
    return ESP_OK;
}

// The ADC unit is created for each measurement and deleted right after. While a oneshot unit
// exists, ESP-IDF keeps the TOP and MODEM power domains on during light sleep (see
// adc_oneshot_new_unit() with CONFIG_PM_POWER_DOWN_PERIPHERAL_IN_LIGHT_SLEEP), which raises the
// sleep current for as long as the unit is open.
static esp_err_t read_battery_mv(float *out_mv)
{
    adc_oneshot_unit_handle_t adc = NULL;
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = s_adc_unit,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_config, &adc), TAG, "adc_oneshot_new_unit failed");

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t err = adc_oneshot_config_channel(adc, s_adc_channel, &channel_config);

    int sum_mv = 0;
    for (int i = 0; i < ADC_NUM_SAMPLES && err == ESP_OK; i++) {
        int raw = 0;
        int mv = 0;
        err = adc_oneshot_read(adc, s_adc_channel, &raw);
        if (err == ESP_OK) {
            err = adc_cali_raw_to_voltage(s_adc_cali, raw, &mv);
        }
        sum_mv += mv;
    }

    adc_oneshot_del_unit(adc);
    ESP_RETURN_ON_ERROR(err, TAG, "battery ADC read failed");

    *out_mv = BATTERY_DIVIDER_RATIO * static_cast<float>(sum_mv) / ADC_NUM_SAMPLES;
    return ESP_OK;
}

static void maybe_sample_battery()
{
    if (!s_adc_ready) {
        return;
    }
    if (s_cycles_until_battery > 0) {
        s_cycles_until_battery--;
        return;
    }

    float mv;
    if (read_battery_mv(&mv) != ESP_OK) {
        return; // retry on the next cycle
    }
    s_cycles_until_battery = s_battery_every_n_cycles - 1;
    s_config->batvoltage.cb(s_config->batvoltage.endpoint_id, smooth_voltage(mv), NULL);
}

/* ------------------------------------------------------------------------------------------ */
/* Scheduling                                                                                 */
/* ------------------------------------------------------------------------------------------ */

static void sample_timer_cb(void *arg)
{
    maybe_sample_battery();
    start_temperature_conversion();
}

esp_err_t init_sensor_readouts(sensor_config_t *config)
{
    ESP_RETURN_ON_FALSE(config && config->temperature.cb && config->batvoltage.cb && config->interval_ms > 0,
                        ESP_ERR_INVALID_ARG, TAG, "invalid sensor config");
    ESP_RETURN_ON_FALSE(!s_sample_timer, ESP_ERR_INVALID_STATE, TAG, "sensor readouts already started");

    s_config = config;
    // Battery sampling piggybacks on the temperature cycle: every n-th cycle, rounded.
    const uint32_t n = (config->battery_interval_ms + config->interval_ms / 2) / config->interval_ms;
    s_battery_every_n_cycles = n > 0 ? n : 1;

    const esp_timer_create_args_t read_args = {
        .callback = read_timer_cb,
        .name = "ds18b20_read",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&read_args, &s_read_timer), TAG, "esp_timer_create failed");

    const esp_timer_create_args_t sample_args = {
        .callback = sample_timer_cb,
        .name = "sensor_sample",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&sample_args, &s_sample_timer), TAG, "esp_timer_create failed");

    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_sample_timer, static_cast<uint64_t>(config->interval_ms) * 1000),
                        TAG, "esp_timer_start_periodic failed");
    return ESP_OK;
}
