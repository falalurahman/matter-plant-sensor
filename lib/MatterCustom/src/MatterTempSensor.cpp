// MatterTempSensor.cpp — Temperature Measurement endpoint (EP1)

#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include "MatterTempSensor.h"
#include "MatterCustom.h"

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

bool MatterTempSensor::attributeChangeCB(uint16_t endpoint_id, uint32_t cluster_id,
                                          uint32_t attribute_id, esp_matter_attr_val_t *val) {
    if (!_started) return false;
    log_d("TempSensor attr update: ep=%u cluster=0x%lX attr=0x%lX",
          endpoint_id, cluster_id, attribute_id);
    return true;
}

bool MatterTempSensor::begin(int16_t raw) {
    if (getEndPointId() != 0) {
        log_e("TempSensor: endpoint already created (id=%d)", getEndPointId());
        return false;
    }

    temperature_sensor::config_t cfg;
    cfg.temperature_measurement.measured_value     = raw;
    cfg.temperature_measurement.min_measured_value = nullptr;
    cfg.temperature_measurement.max_measured_value = nullptr;

    endpoint_t *ep = temperature_sensor::create(node::get(), &cfg, ENDPOINT_FLAG_NONE, (void *)this);
    if (ep == nullptr) {
        log_e("TempSensor: failed to create endpoint");
        return false;
    }
    _rawTemp = raw;
    setEndPointId(endpoint::get_id(ep));
    log_i("TempSensor: endpoint_id=%d", getEndPointId());
    _started = true;
    return true;
}

bool MatterTempSensor::begin(double tempC) {
    // Clamp to Matter range: −100 °C … +327.67 °C (int16 × 100)
    double clamped = constrain(tempC, -100.0, 327.67);
    return begin(static_cast<int16_t>(clamped * 100.0));
}

void MatterTempSensor::end() { _started = false; }

bool MatterTempSensor::setRaw(int16_t raw) {
    if (!_started) { log_e("TempSensor: not started"); return false; }
    if (_rawTemp == raw) return true;

    esp_matter_attr_val_t v = esp_matter_invalid(NULL);
    if (!getAttributeVal(TemperatureMeasurement::Id,
                         TemperatureMeasurement::Attributes::MeasuredValue::Id, &v)) {
        log_e("TempSensor: failed to read attribute");
        return false;
    }
    v.val.i16 = raw;
    if (!updateAttributeVal(TemperatureMeasurement::Id,
                            TemperatureMeasurement::Attributes::MeasuredValue::Id, &v)) {
        log_e("TempSensor: failed to update attribute");
        return false;
    }
    _rawTemp = raw;
    log_v("TempSensor: %.2f °C", raw / 100.0f);
    return true;
}

bool MatterTempSensor::setTemperature(double tempC) {
    double clamped = constrain(tempC, -100.0, 327.67);
    return setRaw(static_cast<int16_t>(clamped * 100.0));
}

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
