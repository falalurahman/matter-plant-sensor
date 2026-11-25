# 🌿 ESP32-H2 Matter Plant Sensor: Technical Specification

## 📋 Project Overview
A lightweight, high-efficiency Matter-over-Thread plant monitoring system. This project bypasses the heavy `Matter.h` abstraction in favor of a custom, memory-optimized implementation utilizing only `MatterEndpoint.h` from the `arduino-esp32` core.



## 🏗️ Hardware Architecture: ESP32-H2 SuperMini
The ESP32-H2 is selected for its native **IEEE 802.15.4 (Thread)** support and low power consumption.

| Component | Interface | Pin | Purpose |
| :--- | :--- | :--- | :--- |
| **Soil Moisture** | ADC1_CH0 | GPIO 1 | Capacitive moisture sensing |
| **SHT4x / BME280**| I2C (0x44) | SDA:4, SCL:5 | Ambient Temp & Humidity |
| **BH1750** | I2C (0x23) | SDA:4, SCL:5 | Light intensity (Lux) |
| **Battery VCC** | ADC1_CH1 | GPIO 2 | Voltage monitoring (via 1:1 Divider) |

---

## 🏛️ Matter Endpoint Mapping
To ensure granular control in smart home ecosystems (HomeKit, Google Home, Alexa), each sensor is assigned a dedicated Endpoint ID.

| Endpoint | Name | Cluster | Attribute |
| :--- | :--- | :--- | :--- |
| **EP 0** | Root Node | `PowerSource` | BatteryPercentageRemaining (0-200) |
| **EP 1** | Temp Sensor | `TemperatureMeasurement` | MeasuredValue ($100 \times ^\circ C$) |
| **EP 2** | Humidity | `RelativeHumidityMeasurement` | MeasuredValue ($100 \times \%$) |
| **EP 3** | Light Sensor | `IlluminanceMeasurement` | MeasuredValue ($10^4 \times \log_{10}(Lux) + 1$) |
| **EP 4** | Soil Sensor | `RelativeMoistureMeasurement` | MeasuredValue ($100 \times \%$) |

---

## 🛠️ Custom Library: `lib/MatterCustom`
To minimize binary size, we strip the standard Arduino Matter library and recreate only the necessary endpoint implementation classes.

### Library Strategy
* **Minimalist Headers:** Only include `MatterEndpoint.h`.
* **Static Allocation:** Avoid dynamic heap allocation for Matter attributes where possible.
* **Direct SDK Access:** Use `esp_matter::node::create()` to initialize the node without the overhead of the full Arduino wrapper.

---

## 🚦 Application Logic: Non-Blocking State Machine
The `main.cpp` logic is driven by a finite state machine (FSM) to ensure the Thread stack remains responsive and to allow for low-power transitions.



### States:
1. **`SYSTEM_BOOT`**: Initialize NVS, Thread stack, I2C bus, and ADC.
2. **`MATTER_READY`**: Confirm commissioning and network join status.
3. **`IDLE_WAIT`**: Non-blocking wait using `millis()`.
4. **`SENSOR_READ`**: 
   * Trigger non-blocking I2C conversions.
   * Sample ADC for Soil Moisture and Battery Voltage.
5. **`DATA_PUSH`**: Compare new values against thresholds; update Matter attributes if changed.
6. **`POWER_SAVE`**: Enter Light Sleep while maintaining the Thread Child-Parent link.

---

## 📈 Optimization Targets
* **Binary Size:** Target < 1.8MB to support OTA on the ESP32-H2's internal flash.
* **Math Accuracy:** Battery percentage is derived from ADC raw values using:
  $$Percentage = \frac{V_{now} - V_{min}}{V_{max} - V_{min}} \times 100$$
* **Power:** Maintain a "Sleepy End Device" (SED) profile to maximize battery life.

---

## 🚀 Development Roadmap
- [ ] **Phase 1:** Setup `platformio.ini` with ESP-IDF + Arduino as a component.
- [ ] **Phase 2:** Implement `MatterCustom` library with 4 distinct sensor endpoints.
- [ ] **Phase 3:** Write the non-blocking I2C/ADC polling state machine.
- [ ] **Phase 4:** Optimize Thread polling intervals and test against a Matter Controller.
