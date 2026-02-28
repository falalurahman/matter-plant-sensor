// MatterLightSensor.h — EP3: Illuminance Measurement (cluster 0x0400)
// Inherits MatterEndPoint; does NOT include Matter.h.
//
// Matter spec encoding:
//   MeasuredValue = 10000 × log10(lux) + 1   (for lux > 0)
//   MeasuredValue = 0                          (for lux == 0)

#pragma once
#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include <MatterEndPoint.h>

class MatterLightSensor : public MatterEndPoint {
public:
    MatterLightSensor() {}
    ~MatterLightSensor() { end(); }

    // Initialize and register the endpoint. lux is the initial reading (≥ 0).
    bool begin(float lux = 0.0f);
    void end();

    // Update the measured illuminance in lux.
    bool setLux(float lux);
    float getLux() const { return _lux; }

    void operator=(float lux) { setLux(lux); }
    operator float()          { return getLux(); }

    bool attributeChangeCB(uint16_t endpoint_id, uint32_t cluster_id,
                           uint32_t attribute_id, esp_matter_attr_val_t *val) override;

    // Convert lux → Matter MeasuredValue (uint16).
    static uint16_t luxToMatter(float lux);

private:
    bool     _started = false;
    uint16_t _rawLux  = 0;   // encoded Matter value
    float    _lux     = 0.0f;

    bool begin(uint16_t raw);
    bool setRaw(uint16_t raw);
};

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
