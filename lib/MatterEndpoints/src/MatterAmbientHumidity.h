// MatterAmbientHumidity.h — EP2: Relative Humidity Measurement (cluster 0x0405)
// Inherits MatterEndPoint; does NOT include Matter.h.

#pragma once
#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include <MatterEndPoint.h>

class MatterAmbientHumidity : public MatterEndPoint {
public:
    MatterAmbientHumidity() {}
    ~MatterAmbientHumidity() { end(); }

    // Initialize and register the endpoint. humidityPct is the initial reading (0–100).
    bool begin(double humidityPct = 0.0);
    void end();

    // Update the measured humidity (%, precision to 0.01 %).
    bool setHumidity(double humidityPct);
    double getHumidity() const { return _rawHumidity / 100.0; }

    void operator=(double humidityPct) { setHumidity(humidityPct); }
    operator double()                  { return getHumidity(); }

    bool attributeChangeCB(uint16_t endpoint_id, uint32_t cluster_id,
                           uint32_t attribute_id, esp_matter_attr_val_t *val) override;

private:
    bool     _started     = false;
    uint16_t _rawHumidity = 0;   // value × 100  (0–10000 → 0–100 %)

    bool begin(uint16_t raw);
    bool setRaw(uint16_t raw);
};

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
