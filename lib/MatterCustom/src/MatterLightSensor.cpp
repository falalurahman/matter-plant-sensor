// MatterLightSensor.cpp — Illuminance Measurement endpoint (EP3)

#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include "MatterLightSensor.h"
#include "MatterCustom.h"
#include <math.h>

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

// ── Encoding helper ──────────────────────────────────────────────────────────
// Matter spec §2.2.5.1: MeasuredValue = 10000 × log₁₀(Lux) + 1, clamped to uint16.
uint16_t MatterLightSensor::luxToMatter(float lux) {
    if (lux <= 0.0f) return 0;
    float encoded = 10000.0f * log10f(lux) + 1.0f;
    if (encoded < 0.0f)       return 0;
    if (encoded > 0xFFFEu)    return 0xFFFE;
    return static_cast<uint16_t>(encoded);
}

// ── Attribute change callback ────────────────────────────────────────────────
bool MatterLightSensor::attributeChangeCB(uint16_t endpoint_id, uint32_t cluster_id,
                                           uint32_t attribute_id, esp_matter_attr_val_t *val) {
    if (!_started) return false;
    log_d("LightSensor attr update: ep=%u cluster=0x%lX attr=0x%lX",
          endpoint_id, cluster_id, attribute_id);
    return true;
}

// ── begin(raw) ───────────────────────────────────────────────────────────────
bool MatterLightSensor::begin(uint16_t raw) {
    if (getEndPointId() != 0) {
        log_e("LightSensor: endpoint already created (id=%d)", getEndPointId());
        return false;
    }

    light_sensor::config_t cfg;
    // min/max are already null-initialised by the config_t default constructor.
    cfg.illuminance_measurement.illuminance_measured_value = raw;

    endpoint_t *ep = light_sensor::create(node::get(), &cfg, ENDPOINT_FLAG_NONE, (void *)this);
    if (ep == nullptr) {
        log_e("LightSensor: failed to create endpoint");
        return false;
    }
    _rawLux = raw;
    setEndPointId(endpoint::get_id(ep));
    log_i("LightSensor: endpoint_id=%d", getEndPointId());
    _started = true;
    return true;
}

bool MatterLightSensor::begin(float lux) {
    _lux = (lux < 0.0f) ? 0.0f : lux;
    return begin(luxToMatter(_lux));
}

void MatterLightSensor::end() { _started = false; }

// ── setRaw() ─────────────────────────────────────────────────────────────────
bool MatterLightSensor::setRaw(uint16_t raw) {
    if (!_started) { log_e("LightSensor: not started"); return false; }
    if (_rawLux == raw) return true;

    esp_matter_attr_val_t v = esp_matter_invalid(NULL);
    if (!getAttributeVal(IlluminanceMeasurement::Id,
                         IlluminanceMeasurement::Attributes::MeasuredValue::Id, &v)) {
        log_e("LightSensor: failed to read attribute");
        return false;
    }
    v.val.u16 = raw;
    if (!updateAttributeVal(IlluminanceMeasurement::Id,
                            IlluminanceMeasurement::Attributes::MeasuredValue::Id, &v)) {
        log_e("LightSensor: failed to update attribute");
        return false;
    }
    _rawLux = raw;
    log_v("LightSensor: %.1f lux (raw=%u)", _lux, raw);
    return true;
}

bool MatterLightSensor::setLux(float lux) {
    _lux = (lux < 0.0f) ? 0.0f : lux;
    return setRaw(luxToMatter(_lux));
}

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
