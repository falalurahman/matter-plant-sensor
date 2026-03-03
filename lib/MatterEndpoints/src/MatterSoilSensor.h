// MatterSoilSensor.h — EP4: Soil Moisture (mapped to humidity_sensor device type,
//                           RelativeHumidityMeasurement cluster 0x0405)
//
// The Matter 1.3 spec has a draft "Soil Moisture Measurement" cluster (0x040C),
// but the current esp-matter SDK does not expose a dedicated device type for it.
// We reuse the humidity_sensor device type (same data format: 0–10000 = 0–100 %).
// Inherits MatterEndPoint; does NOT include Matter.h.

#pragma once
#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include <MatterEndPoint.h>

class MatterSoilSensor : public MatterEndPoint {
public:
    MatterSoilSensor() {}
    ~MatterSoilSensor() { end(); }

    // Initialize and register the endpoint. moisturePct is the initial reading (0–100).
    bool begin(double moisturePct = 0.0);
    void end();

    // Update the measured soil moisture (%, precision to 0.01 %).
    bool setMoisture(double moisturePct);
    double getMoisture() const { return _rawMoisture / 100.0; }

    void operator=(double moisturePct) { setMoisture(moisturePct); }
    operator double()                  { return getMoisture(); }

    bool attributeChangeCB(uint16_t endpoint_id, uint32_t cluster_id,
                           uint32_t attribute_id, esp_matter_attr_val_t *val) override;

private:
    bool     _started      = false;
    uint16_t _rawMoisture  = 0;   // value × 100  (0–10000 → 0–100 %)

    bool begin(uint16_t raw);
    bool setRaw(uint16_t raw);
};

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
