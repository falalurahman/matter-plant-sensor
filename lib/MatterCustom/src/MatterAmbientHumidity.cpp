// MatterAmbientHumidity.cpp — Relative Humidity Measurement endpoint (EP2)

#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include "MatterAmbientHumidity.h"
#include "MatterCustom.h"

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

bool MatterAmbientHumidity::attributeChangeCB(uint16_t endpoint_id, uint32_t cluster_id,
                                               uint32_t attribute_id, esp_matter_attr_val_t *val) {
    if (!_started) return false;
    log_d("AmbientHumidity attr update: ep=%u cluster=0x%lX attr=0x%lX",
          endpoint_id, cluster_id, attribute_id);
    return true;
}

bool MatterAmbientHumidity::begin(uint16_t raw) {
    if (getEndPointId() != 0) {
        log_e("AmbientHumidity: endpoint already created (id=%d)", getEndPointId());
        return false;
    }
    if (raw > 10000) {
        log_e("AmbientHumidity: raw value out of range (max 10000)");
        return false;
    }

    humidity_sensor::config_t cfg;
    cfg.relative_humidity_measurement.measured_value     = raw;
    cfg.relative_humidity_measurement.min_measured_value = nullptr;
    cfg.relative_humidity_measurement.max_measured_value = nullptr;

    endpoint_t *ep = humidity_sensor::create(node::get(), &cfg, ENDPOINT_FLAG_NONE, (void *)this);
    if (ep == nullptr) {
        log_e("AmbientHumidity: failed to create endpoint");
        return false;
    }
    _rawHumidity = raw;
    setEndPointId(endpoint::get_id(ep));
    log_i("AmbientHumidity: endpoint_id=%d", getEndPointId());
    _started = true;
    return true;
}

bool MatterAmbientHumidity::begin(double humidityPct) {
    double clamped = constrain(humidityPct, 0.0, 100.0);
    return begin(static_cast<uint16_t>(clamped * 100.0));
}

void MatterAmbientHumidity::end() { _started = false; }

bool MatterAmbientHumidity::setRaw(uint16_t raw) {
    if (!_started) { log_e("AmbientHumidity: not started"); return false; }
    if (raw > 10000) { log_e("AmbientHumidity: value out of range"); return false; }
    if (_rawHumidity == raw) return true;

    esp_matter_attr_val_t v = esp_matter_invalid(NULL);
    if (!getAttributeVal(RelativeHumidityMeasurement::Id,
                         RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &v)) {
        log_e("AmbientHumidity: failed to read attribute");
        return false;
    }
    v.val.u16 = raw;
    if (!updateAttributeVal(RelativeHumidityMeasurement::Id,
                            RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &v)) {
        log_e("AmbientHumidity: failed to update attribute");
        return false;
    }
    _rawHumidity = raw;
    log_v("AmbientHumidity: %.2f %%", raw / 100.0f);
    return true;
}

bool MatterAmbientHumidity::setHumidity(double humidityPct) {
    double clamped = constrain(humidityPct, 0.0, 100.0);
    return setRaw(static_cast<uint16_t>(clamped * 100.0));
}

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
