#include <Arduino.h>
#include <Matter.h>
#include <esp_sleep.h>
#include <esp_matter_icd_configuration.h>

// Build-flag defaults (override via -D in platformio.ini)
#ifndef SOIL_MOISTURE_PIN
#define SOIL_MOISTURE_PIN 1
#endif

#ifndef ICD_WAKE_INTERVAL_S
#define ICD_WAKE_INTERVAL_S 3600  // 1 hour
#endif

// Matter Humidity Sensor endpoint (used for soil moisture)
MatterHumiditySensor soilMoisture;

// Boot button for decommissioning
const uint8_t buttonPin = BOOT_PIN;
uint32_t button_time_stamp = 0;
bool button_state = false;
const uint32_t decommissioningTimeout = 5000;  // 5 seconds

// Track boot count in RTC memory (survives deep sleep)
RTC_DATA_ATTR uint32_t bootCount = 0;

double readSoilMoisture() {
    // Read raw ADC value (12-bit: 0-4095)
    uint16_t raw = analogRead(SOIL_MOISTURE_PIN);
    // Map to 0-100% (invert: dry = high ADC, wet = low ADC for capacitive sensors)
    double moisture = map(raw, 4095, 0, 0, 10000) / 100.0;
    // Clamp to valid range
    if (moisture < 0.0) moisture = 0.0;
    if (moisture > 100.0) moisture = 100.0;
    return moisture;
}

void setup() {
    Serial.begin(115200);
    bootCount++;

    Serial.printf("Boot count: %u\n", bootCount);
    Serial.printf("Wake interval: %d seconds\n", ICD_WAKE_INTERVAL_S);

    // Print wakeup reason
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER:
            Serial.println("Wakeup: timer");
            break;
        case ESP_SLEEP_WAKEUP_EXT0:
            Serial.println("Wakeup: external signal (ext0)");
            break;
        default:
            Serial.printf("Wakeup: other (%d) - likely power-on/reset\n", wakeup_reason);
            break;
    }

    // Initialize decommission button
    pinMode(buttonPin, INPUT_PULLUP);

    // Configure ADC
    analogSetAttenuation(ADC_11db);

    // Configure ICD parameters before Matter.begin()
    esp_matter::icd::config_t icd_config = {};
    icd_config.idle_mode_duration_s = ICD_WAKE_INTERVAL_S;
    icd_config.active_mode_duration_ms = 10000;   // 10s active after wakeup
    icd_config.active_threshold_ms = 5000;         // 5s activity threshold
    icd_config.slow_interval_ms = 30000;           // 30s slow poll interval
    icd_config.fast_interval_ms = 200;             // 200ms fast poll interval
    esp_matter::icd::set_configuration_data(&icd_config);

    // Initialize soil moisture sensor endpoint with initial reading
    double initialMoisture = readSoilMoisture();
    soilMoisture.begin(initialMoisture);
    Serial.printf("Initial soil moisture: %.2f%%\n", initialMoisture);

    // Start Matter (must be called after all endpoints are initialized)
    Matter.begin();

    // Handle commissioning on first boot
    if (!Matter.isDeviceCommissioned()) {
        Serial.println("Device not commissioned. Waiting for commissioning...");
        Serial.printf("Manual pairing code: %s\n", Matter.getManualPairingCode().c_str());
        Serial.printf("QR code URL: %s\n", Matter.getOnboardingQRCodeUrl().c_str());

        // Wait for commissioning (check button for decommission during wait)
        while (!Matter.isDeviceCommissioned()) {
            delay(100);

            // Check decommission button during commissioning wait
            if (digitalRead(buttonPin) == LOW && !button_state) {
                button_time_stamp = millis();
                button_state = true;
            }
            if (digitalRead(buttonPin) == HIGH && button_state) {
                button_state = false;
            }
        }
        Serial.println("Device commissioned successfully!");
    }

    // Wait for network connection
    if (!Matter.isDeviceConnected()) {
        Serial.println("Waiting for Thread network connection...");
        while (!Matter.isDeviceConnected()) {
            delay(100);
        }
        Serial.println("Connected to Thread network.");
    }
}

void loop() {
    // Read and report soil moisture
    double moisture = readSoilMoisture();
    soilMoisture.setHumidity(moisture);
    Serial.printf("Soil moisture: %.2f%%\n", moisture);

    // Check decommission button
    if (digitalRead(buttonPin) == LOW && !button_state) {
        button_time_stamp = millis();
        button_state = true;
    }
    if (digitalRead(buttonPin) == HIGH && button_state) {
        button_state = false;
    }
    if (button_state && (millis() - button_time_stamp) > decommissioningTimeout) {
        Serial.println("Decommissioning device...");
        Matter.decommission();
        Serial.println("Decommissioned. Restarting...");
        delay(500);
        esp_restart();
    }

    // Allow Matter stack time to process and send the update
    delay(5000);

    // Configure timer wakeup and enter deep sleep
    Serial.printf("Entering deep sleep for %d seconds...\n", ICD_WAKE_INTERVAL_S);
    Serial.flush();
    esp_sleep_enable_timer_wakeup((uint64_t)ICD_WAKE_INTERVAL_S * 1000000ULL);
    esp_deep_sleep_start();
}
