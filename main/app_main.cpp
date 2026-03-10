/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#if CONFIG_PM_ENABLE
#include <esp_pm.h>
#endif

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <common_macros.h>
#include <log_heap_numbers.h>

#include <app_priv.h>
#include <app_reset.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

static const char *TAG = "app_main";
uint16_t light_endpoint_id = 0;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

constexpr auto k_timeout_seconds = 300;

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
extern const uint8_t cd_start[] asm("_binary_certification_declaration_der_start");
extern const uint8_t cd_end[] asm("_binary_certification_declaration_der_end");

const chip::ByteSpan cdSpan(cd_start, static_cast<size_t>(cd_end - cd_start));
#endif // CONFIG_ENABLE_SET_CERT_DECLARATION_API

#if CONFIG_ENABLE_ENCRYPTED_OTA
extern const char decryption_key_start[] asm("_binary_esp_image_encryption_key_pem_start");
extern const char decryption_key_end[] asm("_binary_esp_image_encryption_key_pem_end");

static const char *s_decryption_key = decryption_key_start;
static const uint16_t s_decryption_key_len = decryption_key_end - decryption_key_start;
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

// Application cluster specification, 7.18.2.11. Temperature
// represents a temperature on the Celsius scale with a resolution of 0.01°C.
// temp = (temperature in °C) x 100
static void temp_sensor_notification(uint16_t endpoint_id, float temp, void *user_data) {
  // schedule the attribute update so that we can report it from matter thread
  chip::DeviceLayer::SystemLayer().ScheduleLambda([endpoint_id, temp]() {
    attribute_t *attribute = attribute::get(endpoint_id, TemperatureMeasurement::Id,
                                            TemperatureMeasurement::Attributes::MeasuredValue::Id);

    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    attribute::get_val(attribute, &val);
    val.val.u16 = static_cast<uint16_t>(temp * 100);

    attribute::update(endpoint_id, TemperatureMeasurement::Id,
                      TemperatureMeasurement::Attributes::MeasuredValue::Id, &val);
  });
}

const static float voltage_levels[] = {
    4.15,
    4.05,
    4.01,
    3.99,
    3.92,
    3.86,
    3.80,
    3.20
};

const static uint8_t percentage_levels[] = {
    100,
    90,
    80,
    60,
    50,
    40,
    20,
    0
};

constexpr size_t num_levels = sizeof(voltage_levels) / sizeof(voltage_levels[0]);

// Ensure that voltage_levels and percentage_levels have the same number of elements
static_assert(num_levels == sizeof(percentage_levels) / sizeof(percentage_levels[0]), "voltage_levels and percentage_levels should have the same number of elements");

static float voltage_to_percentage(float voltage) {
  int i = 0;
  for (; i < num_levels; i++) {
    if (voltage >= voltage_levels[i]) {
      break;
    }
  }

  if (i == 0) {
    return percentage_levels[0];
  }

  if (i == num_levels) {
    return percentage_levels[num_levels - 1];
  }

  float percentage = percentage_levels[i] + (percentage_levels[i + 1] - percentage_levels[i]) * (voltage - voltage_levels[i]) / (voltage_levels[i + 1] - voltage_levels[i]);

  return percentage;
}

// Application cluster specification, 7.11.7. Battery Voltage
// represents a voltage with a resolution of 0.01V.
static void battery_voltage_notification(uint16_t endpoint_id, float voltage, void *user_data) {
  float charge_percentage = voltage_to_percentage(voltage / 1000.0);
  float charge_level = charge_percentage * 2.0;

  if (charge_level < 0) {
    charge_level = 0.0;
  } else if (charge_level > 200) {
    charge_level = 200.0;
  }

  ESP_LOGI(TAG, "Battery voltage: %.2f V, charge level: %.2f%%", voltage / 1000.0, charge_percentage * 100);

  // schedule the attribute update so that we can report it from matter thread
  chip::DeviceLayer::SystemLayer().ScheduleLambda([endpoint_id, voltage, charge_level]() {
    // battery voltage
    attribute_t *attribute_bat_voltage = attribute::get(endpoint_id, PowerSource::Id,
                                            PowerSource::Attributes::BatVoltage::Id);
    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    attribute::get_val(attribute_bat_voltage, &val);
    val.val.u32 = static_cast<uint32_t>(voltage);

    attribute::update(endpoint_id, PowerSource::Id, PowerSource::Attributes::BatVoltage::Id, &val);

    // battery charge percentage
    attribute_t *attribute_charge_percentage = attribute::get(endpoint_id, PowerSource::Id,
                                            PowerSource::Attributes::BatPercentRemaining::Id);

    esp_matter_attr_val_t charge_percentage_val = esp_matter_invalid(NULL);
    attribute::get_val(attribute_charge_percentage, &charge_percentage_val);
    charge_percentage_val.val.u8 = static_cast<uint8_t>(charge_level);

    attribute::update(endpoint_id, PowerSource::Id, PowerSource::Attributes::BatPercentRemaining::Id, &charge_percentage_val);
  });
}

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved: {
        ESP_LOGI(TAG, "Fabric removed successfully");
        if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
            chip::CommissioningWindowManager  &commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
            constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
            if (!commissionMgr.IsCommissioningWindowOpen()) {
                /* After removing last fabric, this example does not remove the Wi-Fi credentials
                 * and still has IP connectivity so, only advertising on DNS-SD.
                 */
                CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                                                                            chip::CommissioningWindowAdvertisement::kDnssdOnly);
                if (err != CHIP_NO_ERROR) {
                    ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
                }
            }
        }
        break;
    }

    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(TAG, "Fabric is committed");
        break;

    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized and memory reclaimed");
        break;

    default:
        break;
    }
}

// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    switch (type) {
      case PRE_UPDATE:
          ESP_LOGI(TAG, "Attribute pre_update callback: endpoint: %u, cluster: %u, attribute: %u", endpoint_id, cluster_id, attribute_id);
          break;
      case POST_UPDATE:
          ESP_LOGI(TAG, "Attribute post_update callback: endpoint: %u, cluster: %u, attribute: %u", endpoint_id, cluster_id, attribute_id);
          break;
      case READ:
          ESP_LOGI(TAG, "Attribute read callback: endpoint: %u, cluster: %u, attribute: %u", endpoint_id, cluster_id, attribute_id);
          break;
      case WRITE:
          ESP_LOGI(TAG, "Attribute write callback: endpoint: %u, cluster: %u, attribute: %u", endpoint_id, cluster_id, attribute_id);
          break;
    }

    return ESP_OK;
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;

    // node handle can be used to add/modify other endpoints.
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    // add temperature sensor device endpoint
    temperature_sensor::config_t temp_sensor_config;
    endpoint_t *temp_sensor_ep =
        temperature_sensor::create(node, &temp_sensor_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(temp_sensor_ep != nullptr,
                         ESP_LOGE(TAG, "Failed to create temperature_sensor endpoint"));

    // add power source cluster
    esp_matter::cluster::power_source::config_t power_source_config;
    power_source_config.feature_flags = esp_matter::cluster::power_source::feature::battery::get_id();
    endpoint_t *power_source_cluster = esp_matter::cluster::power_source::create(temp_sensor_ep, &power_source_config, CLUSTER_FLAG_SERVER);
    ABORT_APP_ON_FAILURE(power_source_cluster != nullptr,
                         ESP_LOGE(TAG, "Failed to create power_source endpoint"));

    // add battery voltage attribute to cluster
    esp_matter::cluster::power_source::attribute::create_bat_voltage
      (power_source_cluster, nullable<uint32_t>(0), nullable<uint32_t>(0), nullable<uint32_t>(4200));
    // esp_matter::cluster::power_source::attribute::create_bat_charge_level(power_source_cluster, 0);
    esp_matter::cluster::power_source::attribute::create_bat_percent_remaining(power_source_cluster, nullable<uint8_t>(0), nullable<uint8_t>(0), nullable<uint8_t>(255));

    // initialize temperature sensor driver (ds18b20)
    err = init_ds18x20_devices();
    err = init_voltage_adc();

    static sensor_config_t sensor_config = {
        .interval_ms = CONFIG_SENSOR_READ_INTERVAL,
        .temperature = {
            .cb = temp_sensor_notification,
            .endpoint_id = endpoint::get_id(temp_sensor_ep),
        },
        .batvoltage = {
            .cb = battery_voltage_notification,
            .endpoint_id = endpoint::get_id(temp_sensor_ep),
        },
    };
    ESP_LOGI(TAG, "Registering readouts for temperature endpoint %u", sensor_config.temperature.endpoint_id);
    ESP_LOGI(TAG, "Registering readouts for voltage endpoint %u", sensor_config.batvoltage.endpoint_id);
    err = init_sensor_readouts(&sensor_config);
    ABORT_APP_ON_FAILURE(err == ESP_OK,
                         ESP_LOGE(TAG, "Failed to initialize temperature sensor driver"));

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    err = esp_pm_configure(&pm_config);
#endif

}
