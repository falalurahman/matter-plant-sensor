// MatterTempSensor.h — EP1: Temperature Measurement (cluster 0x0402)
// Inherits MatterEndPoint; does NOT include Matter.h.

#pragma once
#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include <MatterEndPoint.h>

class MatterTempSensor : public MatterEndPoint {
public:
    MatterTempSensor() {}
    ~MatterTempSensor() { end(); }

    // Initialize and register the endpoint. tempC is the initial reading.
    bool begin(double tempC = 0.0);
    void end();

    // Update the measured temperature (°C, precision to 0.01 °C).
    bool setTemperature(double tempC);
    double getTemperature() const { return _rawTemp / 100.0; }

    void operator=(double tempC)  { setTemperature(tempC); }
    operator double()             { return getTemperature(); }

    bool attributeChangeCB(uint16_t endpoint_id, uint32_t cluster_id,
                           uint32_t attribute_id, esp_matter_attr_val_t *val) override;

private:
    bool    _started  = false;
    int16_t _rawTemp  = 0;   // value × 100  (signed, supports negative temps)

    bool begin(int16_t raw);
    bool setRaw(int16_t raw);
};

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
