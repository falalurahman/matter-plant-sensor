#pragma once
#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include <Arduino.h>
#include <esp_matter.h>

// Matter 1.5 Soil Moisture Measurement cluster
static constexpr uint32_t SOIL_MEASUREMENT_CLUSTER_ID = 0x040B;
static constexpr uint32_t SOIL_MEASURED_VALUE_ATTR_ID = 0x0000;
static constexpr uint32_t SOIL_MIN_MEASURED_VALUE_ATTR_ID = 0x0001;
static constexpr uint32_t SOIL_MAX_MEASURED_VALUE_ATTR_ID = 0x0002;
static constexpr uint32_t SOIL_SENSOR_DEVICE_TYPE_ID = 0x0045;
static constexpr uint8_t SOIL_SENSOR_DEVICE_TYPE_VERSION = 1;

class MatterSoilSensor {
public:
  MatterSoilSensor();
  ~MatterSoilSensor();

  // begin Matter Soil Sensor endpoint with initial moisture percent
  bool begin(double moisturePercent = 0.00) {
    if (moisturePercent < 0.0 || moisturePercent > 100.0) {
      log_e("Soil Sensor Percentage value out of range [0..100].");
      return false;
    }
    return begin(static_cast<uint16_t>(moisturePercent * 100.0f));
  }

  // stop processing Matter events
  void end();

  // set the moisture percent with 1/100th of a percent precision
  bool setMoisture(double moisturePercent) {
    if (moisturePercent < 0.0 || moisturePercent > 100.0) {
      log_e("Soil Sensor Percentage value out of range [0..100].");
      return false;
    }
    return setRawMoisture(static_cast<uint16_t>(moisturePercent * 100.0f));
  }

  // returns the reported moisture percent with 1/100th of precision
  double getMoisture() {
    return (double)rawMoisture / 100.0;
  }

  void operator=(double moisturePercent) {
    setMoisture(moisturePercent);
  }

  operator double() {
    return (double)getMoisture();
  }

  // Start the Matter stack (replaces Matter.begin() for custom endpoints)
  static bool startMatter();

private:
  bool started = false;
  uint16_t rawMoisture = 0;
  uint16_t endpointId = 0;
  bool begin(uint16_t _rawMoisture);
  bool setRawMoisture(uint16_t _rawMoisture);
};
#endif /* CONFIG_ESP_MATTER_ENABLE_DATA_MODEL */
