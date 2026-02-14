#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include "MatterSoilSensor.h"
#if CONFIG_ENABLE_MATTER_OVER_THREAD
#include "esp_openthread_types.h"
#include "platform/ESP32/OpenthreadLauncher.h"
#endif

using namespace esp_matter;
using namespace esp_matter::cluster;

static esp_matter::node_t *s_matter_node = nullptr;

MatterSoilSensor::MatterSoilSensor() {}

MatterSoilSensor::~MatterSoilSensor() {
  end();
}

bool MatterSoilSensor::begin(uint16_t _rawMoisture) {
  if (endpointId != 0) {
    log_e("Matter Soil Sensor with Endpoint Id %d has already been created.", endpointId);
    return false;
  }

  if (_rawMoisture > 10000) {
    log_e("Soil Sensor Percentage value out of range [0..100].");
    return false;
  }

  // Create the Matter node if it doesn't exist yet
  if (s_matter_node == nullptr) {
    node::config_t node_config;
    s_matter_node = node::create(&node_config, NULL, NULL);
    if (s_matter_node == nullptr) {
      log_e("Failed to create Matter node");
      return false;
    }
  }

  // Create a generic endpoint
  endpoint_t *ep = endpoint::create(s_matter_node, ENDPOINT_FLAG_NONE, NULL);
  if (ep == nullptr) {
    log_e("Failed to create Soil Sensor endpoint");
    return false;
  }

  // Add descriptor cluster
  descriptor::config_t descriptor_config;
  descriptor::create(ep, &descriptor_config, CLUSTER_FLAG_SERVER);

  // Register device type
  esp_err_t err = endpoint::add_device_type(ep, SOIL_SENSOR_DEVICE_TYPE_ID, SOIL_SENSOR_DEVICE_TYPE_VERSION);
  if (err != ESP_OK) {
    log_e("Failed to add Soil Sensor device type: %d", err);
    return false;
  }

  // Add identify cluster
  identify::config_t identify_config;
  identify_config.identify_type = 2;  // kVisibleIndicator
  identify::create(ep, &identify_config, CLUSTER_FLAG_SERVER);

  // Create soil measurement cluster (0x040B)
  cluster_t *soil_cluster = cluster::create(ep, SOIL_MEASUREMENT_CLUSTER_ID, CLUSTER_FLAG_SERVER);
  if (soil_cluster == nullptr) {
    log_e("Failed to create Soil Measurement cluster");
    return false;
  }

  // Add mandatory global attributes
  global::attribute::create_cluster_revision(soil_cluster, 3);
  global::attribute::create_feature_map(soil_cluster, 0);

  // Add MeasuredValue attribute (0x0000) - nullable uint16
  nullable<uint16_t> measured_value(_rawMoisture);
  attribute::create(soil_cluster, SOIL_MEASURED_VALUE_ATTR_ID, ATTRIBUTE_FLAG_NULLABLE, esp_matter_nullable_uint16(measured_value));

  // Add MinMeasuredValue attribute (0x0001) - nullable uint16, set to null
  nullable<uint16_t> min_value;
  attribute::create(soil_cluster, SOIL_MIN_MEASURED_VALUE_ATTR_ID, ATTRIBUTE_FLAG_NULLABLE, esp_matter_nullable_uint16(min_value));

  // Add MaxMeasuredValue attribute (0x0002) - nullable uint16, set to null
  nullable<uint16_t> max_value;
  attribute::create(soil_cluster, SOIL_MAX_MEASURED_VALUE_ATTR_ID, ATTRIBUTE_FLAG_NULLABLE, esp_matter_nullable_uint16(max_value));

  rawMoisture = _rawMoisture;
  endpointId = endpoint::get_id(ep);
  log_i("Soil Sensor created with endpoint_id %d", endpointId);

  started = true;
  return true;
}

void MatterSoilSensor::end() {
  started = false;
}

bool MatterSoilSensor::setRawMoisture(uint16_t _rawMoisture) {
  if (!started) {
    log_e("Matter Soil Sensor device has not begun.");
    return false;
  }

  if (_rawMoisture > 10000) {
    log_e("Soil Sensor Percentage value out of range [0..100].");
    return false;
  }

  // avoid processing if there was no change
  if (rawMoisture == _rawMoisture) {
    return true;
  }

  esp_matter_attr_val_t moistureVal = esp_matter_nullable_uint16(_rawMoisture);
  esp_err_t err = attribute::update(endpointId, SOIL_MEASUREMENT_CLUSTER_ID, SOIL_MEASURED_VALUE_ATTR_ID, &moistureVal);
  if (err != ESP_OK) {
    log_e("Failed to update Soil Sensor Attribute: %d", err);
    return false;
  }
  rawMoisture = _rawMoisture;
  log_v("Soil Sensor set to %.02f Percent", (float)_rawMoisture / 100.00);

  return true;
}

bool MatterSoilSensor::startMatter() {
#if CONFIG_ENABLE_MATTER_OVER_THREAD
  // Set OpenThread platform config (same as Matter.begin())
  esp_openthread_platform_config_t ot_config;
  memset(&ot_config, 0, sizeof(esp_openthread_platform_config_t));
  ot_config.radio_config.radio_mode = RADIO_MODE_NATIVE;
  ot_config.host_config.host_connection_mode = HOST_CONNECTION_MODE_NONE;
  ot_config.port_config.storage_partition_name = "nvs";
  ot_config.port_config.netif_queue_size = 10;
  ot_config.port_config.task_queue_size = 10;
  set_openthread_platform_config(&ot_config);
#endif

  esp_err_t err = esp_matter::start(NULL);
  if (err != ESP_OK) {
    log_e("Failed to start Matter: %d", err);
    return false;
  }
  return true;
}

#endif /* CONFIG_ESP_MATTER_ENABLE_DATA_MODEL */
