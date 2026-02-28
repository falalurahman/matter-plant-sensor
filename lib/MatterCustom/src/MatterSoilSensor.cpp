// MatterSoilSensor.cpp — Soil Moisture endpoint (EP4)
// Uses humidity_sensor device type / RelativeHumidityMeasurement cluster.

#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include "MatterSoilSensor.h"
#include "MatterCustom.h"

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

bool MatterSoilSensor::attributeChangeCB(uint16_t endpoint_id, uint32_t cluster_id,
                                          uint32_t attribute_id, esp_matter_attr_val_t *val) {
    if (!_started) return false;
    log_d("SoilSensor attr update: ep=%u cluster=0x%lX attr=0x%lX",
          endpoint_id, cluster_id, attribute_id);
    return true;
}

bool MatterSoilSensor::begin(uint16_t raw) {
    if (getEndPointId() != 0) {
        log_e("SoilSensor: endpoint already created (id=%d)", getEndPointId());
        return false;
    }
    if (raw > 10000) {
        log_e("SoilSensor: raw value out of range (max 10000)");
        return false;
    }

    humidity_sensor::config_t cfg;
    cfg.relative_humidity_measurement.measured_value     = raw;
    cfg.relative_humidity_measurement.min_measured_value = nullptr;
    cfg.relative_humidity_measurement.max_measured_value = nullptr;

    endpoint_t *ep = humidity_sensor::create(node::get(), &cfg, ENDPOINT_FLAG_NONE, (void *)this);
    if (ep == nullptr) {
        log_e("SoilSensor: failed to create endpoint");
        return false;
    }
    _rawMoisture = raw;
    setEndPointId(endpoint::get_id(ep));
    log_i("SoilSensor: endpoint_id=%d", getEndPointId());
    _started = true;
    return true;
}

bool MatterSoilSensor::begin(double moisturePct) {
    double clamped = constrain(moisturePct, 0.0, 100.0);
    return begin(static_cast<uint16_t>(clamped * 100.0));
}

void MatterSoilSensor::end() { _started = false; }

bool MatterSoilSensor::setRaw(uint16_t raw) {
    if (!_started) { log_e("SoilSensor: not started"); return false; }
    if (raw > 10000) { log_e("SoilSensor: value out of range"); return false; }
    if (_rawMoisture == raw) return true;

    esp_matter_attr_val_t v = esp_matter_invalid(NULL);
    if (!getAttributeVal(RelativeHumidityMeasurement::Id,
                         RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &v)) {
        log_e("SoilSensor: failed to read attribute");
        return false;
    }
    v.val.u16 = raw;
    if (!updateAttributeVal(RelativeHumidityMeasurement::Id,
                            RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &v)) {
        log_e("SoilSensor: failed to update attribute");
        return false;
    }
    _rawMoisture = raw;
    log_v("SoilSensor: %.2f %%", raw / 100.0f);
    return true;
}

bool MatterSoilSensor::setMoisture(double moisturePct) {
    double clamped = constrain(moisturePct, 0.0, 100.0);
    return setRaw(static_cast<uint16_t>(clamped * 100.0));
}

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
