// main.cpp — ESP32-H2 Matter Plant Sensor
//
// Architecture: non-blocking FSM, Matter-over-Thread, ICD (Sleepy End Device).
// No dependency on the high-level Matter.h wrapper; uses MatterCustom + direct
// esp_matter SDK calls throughout.
//
// Hardware (ESP32-H2 SuperMini) — pins configurable via platformio.ini build flags:
//   SOIL_MOISTURE_PIN (ADC1_CH0) — capacitive soil-moisture sensor
//   BATTERY_ADC_PIN   (ADC1_CH1) — battery voltage via 1:1 resistor divider
//   SENSOR_PWR_PIN               — NPN transistor base (HIGH = sensors powered on)
//   I2C_SDA_PIN / I2C_SCL_PIN    — I2C bus
//     I2C_ADDR_SHT4X — SHT4x  (ambient temperature + humidity)
//     I2C_ADDR_BH1750 — BH1750 (ambient light, lux)

#include <Arduino.h>
#include <Wire.h>
#include <esp_sleep.h>
#include <math.h>
#if CONFIG_ENABLE_MATTER_OVER_THREAD
#include <esp_openthread.h>
#include <openthread/thread.h>
#endif

#include "MatterInit.h"
#include "MatterTempSensor.h"
#include "MatterAmbientHumidity.h"
#include "MatterLightSensor.h"
#include "MatterSoilSensor.h"

// ─── Build-flag defaults ────────────────────────────────────────────────────
#ifndef SOIL_MOISTURE_PIN
#define SOIL_MOISTURE_PIN 1
#endif
#ifndef BATTERY_ADC_PIN
#define BATTERY_ADC_PIN 2
#endif
#ifndef SENSOR_PWR_PIN
#define SENSOR_PWR_PIN 11
#endif
#ifndef CHARGE_DETECT_PIN
#define CHARGE_DETECT_PIN 3
#endif
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 4
#endif
#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 5
#endif
#ifndef I2C_ADDR_SHT4X
#define I2C_ADDR_SHT4X 0x44
#endif
#ifndef I2C_ADDR_BH1750
#define I2C_ADDR_BH1750 0x23
#endif
#ifndef ACTION_BUTTON_PIN
#define ACTION_BUTTON_PIN 10
#endif
#ifndef ICD_WAKE_INTERVAL_S
#define ICD_WAKE_INTERVAL_S 3600  // 1 hour; mirrors CONFIG_ICD_IDLE_MODE_INTERVAL_SEC
#endif

// Action button confirmation window (ms).
static constexpr uint32_t ACTION_CONFIRM_WINDOW_MS = 5000;

// Battery voltage limits (mV) for a single-cell LiPo, 1:1 divider on ADC.
static constexpr float BATTERY_MAX_MILLIVOLTS = 4200.0f;
static constexpr float BATTERY_MIN_MILLIVOLTS = 3000.0f;

// Minimum change required before pushing a Matter attribute update.
static constexpr double  TEMPERATURE_THRESHOLD_CELSIUS = 0.1;   // °C
static constexpr double  HUMIDITY_THRESHOLD            = 0.5;   // %
static constexpr float   LUX_THRESHOLD_PERCENT         = 5.0f;  // % relative change
static constexpr double  SOIL_THRESHOLD                = 0.5;   // %
static constexpr uint8_t BATTERY_THRESHOLD             = 1;     // %

// BH1750 measurement time for One-Time High-Resolution Mode 2 (0.5 lx).
static constexpr uint32_t LIGHT_SENSOR_MEASUREMENT_MILLIS = 180;

// ─── Matter endpoints ────────────────────────────────────────────────────────
static MatterTempSensor      gTempSensor;
static MatterAmbientHumidity gAmbHumidity;
static MatterLightSensor     gLightSensor;
static MatterSoilSensor      gSoilSensor;

// ─── FSM ─────────────────────────────────────────────────────────────────────
enum class State : uint8_t {
    SYSTEM_BOOT,
    MATTER_READY,
    MATTER_DECOMMISSIONED,   // wait for BLE commission + Thread join
    COMMISSIONING_GRACE,     // idle hold so controller can complete subscriptions
    ACTION_BUTTON_PRESSED,   // button held: short press → force read, long press → decommission
    IDLE_WAIT,
    SENSOR_READ,
    DATA_PUSH,
    POWER_SAVE,
};
static State gState = State::SYSTEM_BOOT;

// ─── Sensor data (last read) ─────────────────────────────────────────────────
static double   gTemperatureCelsius = 0.0;
static double   gHumidityPercent    = 0.0;
static float    gLux                = 0.0f;
static double   gSoilPercent        = 0.0;
static uint8_t  gBatteryPercent     = 0;
static uint32_t gBatteryMillivolts  = 0;   // actual Vbat in mV (ADC × 2, 1:1 divider)
static bool     gIsCharging         = false;

// ─── Timing ──────────────────────────────────────────────────────────────────
static uint32_t gWakeStartMillis             = 0;  // millis() at start of setup() for cycle time logging
static uint32_t gSensorPowerOnMillis         = 0;  // millis() when sensor power GPIO went HIGH
static uint32_t gIdleStart                   = 0;
static uint32_t gActionPressedAt             = 0;  // millis() of first button press in session
static uint32_t gActionDecommStart           = 0;  // millis() of current continuous hold; 0 = not held
static uint32_t gLightSensorMeasurementStart = 0;
static bool     gLightSensorMeasurementArmed = false;

// ─── Sensor sampling (3× median) ─────────────────────────────────────────────
static uint8_t  gSampleCount                 = 0;
static double   gTemperatureReadings[3]      = {};
static double   gHumidityReadings[3]         = {};
static float    gLuxReadings[3]              = {};
static double   gSoilMoistureReadings[3]     = {};
static uint8_t  gBatteryReadings[3]          = {};
static uint32_t gBatteryMillivoltReadings[3] = {};

// ─── RTC state (survives deep sleep) ─────────────────────────────────────────
RTC_DATA_ATTR static uint32_t               gBootCount  = 0;
RTC_DATA_ATTR static esp_sleep_wakeup_cause_t gWakeReason = ESP_SLEEP_WAKEUP_UNDEFINED;

// ════════════════════════════════════════════════════════════════════════════
// ── I2C drivers (minimal, no external library) ───────────────────────────────
// ════════════════════════════════════════════════════════════════════════════

// ── SHT4x ────────────────────────────────────────────────────────────────────
// CRC-8 check for SHT4x data (polynomial 0x31, initial value 0xFF).
static uint8_t sht4xCRC(uint8_t msb, uint8_t lsb) {
    uint8_t crc = 0xFF;
    for (uint8_t b : {msb, lsb}) {
        crc ^= b;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
    return crc;
}

// Trigger a high-precision measurement (0xFD) and synchronously read the result.
// Returns false if the sensor is absent or CRC fails.
static bool sht4xRead(double &temperatureCelsius, double &humidityPercent) {
    Wire.beginTransmission(I2C_ADDR_SHT4X);
    Wire.write(0xFD);  // Measure T+RH, high precision
    if (Wire.endTransmission() != 0) return false;

    delay(15);  // SHT4x high-precision: 8.3 ms typical; 15 ms gives margin

    if (Wire.requestFrom(static_cast<uint8_t>(I2C_ADDR_SHT4X), static_cast<uint8_t>(6)) != 6)
        return false;

    uint8_t buf[6];
    for (auto &b : buf) b = Wire.read();

    if (sht4xCRC(buf[0], buf[1]) != buf[2]) return false;  // temperature CRC
    if (sht4xCRC(buf[3], buf[4]) != buf[5]) return false;  // humidity CRC

    uint16_t rawT  = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    uint16_t rawRH = (static_cast<uint16_t>(buf[3]) << 8) | buf[4];

    temperatureCelsius = -45.0 + 175.0 * rawT  / 65535.0;
    humidityPercent    = constrain(-6.0 + 125.0 * rawRH / 65535.0, 0.0, 100.0);
    return true;
}

// ── BH1750 ───────────────────────────────────────────────────────────────────
// Send the "Power On" command.  Measurements are triggered on-demand via
// bh1750Trigger() (One-Time High-Resolution Mode 2, 0x21).
static void bh1750Init() {
    Wire.beginTransmission(I2C_ADDR_BH1750);
    Wire.write(0x01);  // Power On
    Wire.endTransmission();
}

// Trigger a one-time high-resolution mode 2 measurement (non-blocking start).
static void bh1750Trigger() {
    Wire.beginTransmission(I2C_ADDR_BH1750);
    Wire.write(0x21);  // One-Time High-Resolution Mode 2
    Wire.endTransmission();
}

// Read the most recent BH1750 result.  Call after LIGHT_SENSOR_MEASUREMENT_MILLIS has elapsed.
static bool bh1750Read(float &lux) {
    if (Wire.requestFrom(static_cast<uint8_t>(I2C_ADDR_BH1750), static_cast<uint8_t>(2)) != 2)
        return false;
    uint16_t raw = (static_cast<uint16_t>(Wire.read()) << 8) | Wire.read();
    lux = raw / 1.2f;  // BH1750 data sheet conversion factor
    return true;
}

// ── ADC helpers ───────────────────────────────────────────────────────────────
static double readSoilMoisture() {
    uint16_t raw = analogRead(SOIL_MOISTURE_PIN);
    // Capacitive sensor: dry = high ADC, wet = low ADC.  Calibrated via build flags.
    double pct = (double)(SOIL_MOISTURE_ADC_DRY - raw) /
                 (double)(SOIL_MOISTURE_ADC_DRY - SOIL_MOISTURE_ADC_WET) * 100.0;
    return constrain(pct, 0.0, 100.0);
}

static uint8_t readBatteryPercent() {
    uint32_t adcMillivolts  = analogReadMilliVolts(BATTERY_ADC_PIN);  // eFuse-calibrated; no manual scaling needed
    float batteryMillivolts = adcMillivolts * 2.0f;  // 1:1 divider → actual Vbat = 2× ADC pin voltage
    gBatteryMillivolts = static_cast<uint32_t>(batteryMillivolts);
    float batteryPercent = (batteryMillivolts - BATTERY_MIN_MILLIVOLTS) /
                           (BATTERY_MAX_MILLIVOLTS - BATTERY_MIN_MILLIVOLTS) * 100.0f;
    Serial.printf("BAT_PERCENT: (ADC pin: %lu mV, battery: %.1f mV)\n", adcMillivolts, batteryMillivolts);
    return static_cast<uint8_t>(constrain(batteryPercent, 0.0f, 100.0f));
}

static bool readChargeDetect() {
    uint32_t mv = analogReadMilliVolts(CHARGE_DETECT_PIN);
    Serial.printf("CHARGE_DETECT: %lu mV\n", mv);
    return mv > 2000;
}

// ════════════════════════════════════════════════════════════════════════════
// ── Utility ──────────────────────────────────────────────────────────────────
// ════════════════════════════════════════════════════════════════════════════

// Return the median of three values (sort-based, no heap allocation).
template <typename T>
static T median3(T a, T b, T c) {
    if (a > b) { T t = a; a = b; b = t; }
    if (b > c) { T t = b; b = c; c = t; }
    if (a > b) { T t = a; a = b; b = t; }
    return b;
}

// ════════════════════════════════════════════════════════════════════════════
// ── Arduino entry points ─────────────────────────────────────────────────────
// ════════════════════════════════════════════════════════════════════════════

void setup() {
    gWakeStartMillis = millis();  // Record wake time for end-of-cycle log

    // Power on sensors immediately via NPN transistor on SENSOR_PWR_PIN.
    // Sensors stabilize while the Matter stack initializes below (~3–10 s).
    pinMode(static_cast<gpio_num_t>(SENSOR_PWR_PIN), OUTPUT);
    digitalWrite(static_cast<gpio_num_t>(SENSOR_PWR_PIN), HIGH);
    gSensorPowerOnMillis = millis();

    Serial.begin(115200);
    gBootCount++;
    Serial.printf("\n=== Plant Sensor boot #%lu ===\n", gBootCount);

    gWakeReason = esp_sleep_get_wakeup_cause();
    Serial.printf("Wakeup reason: %d\n", (int)gWakeReason);

    // Configure the action button before the first loop() iteration so that
    // the top-of-loop digitalRead() check never sees a floating pin.
    pinMode(ACTION_BUTTON_PIN, INPUT_PULLUP);

    gState = State::SYSTEM_BOOT;
}

void loop() {
    // Detect fresh button press from any state — transition to ACTION_BUTTON_PRESSED.
    if (gState != State::ACTION_BUTTON_PRESSED &&
        digitalRead(ACTION_BUTTON_PIN) == LOW) {
        if(MatterInit::isCommissioned()) {
            gState = State::ACTION_BUTTON_PRESSED;
        }
    }
    switch (gState) {

    // ── SYSTEM_BOOT ──────────────────────────────────────────────────────────
    // NOTE: Deep sleep causes a full chip reset on wakeup — setup() re-runs and
    // the Matter + OpenThread stacks are fully re-initialized from NVS every cycle.
    // The "Failed to remove advertised services: 3" (ESP_ERR_NOT_FOUND) errors
    // logged during Matter startup are benign: mDNS cleanup runs before any
    // services have been registered in the new boot cycle.  NVS preserves fabrics,
    // the Thread dataset, CASE session resumption IDs, and subscription state.
    case State::SYSTEM_BOOT: {
        // ADC
        analogSetAttenuation(ADC_11db);

        // I2C
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
        Wire.setClock(100000);  // 100 kHz standard mode — explicit, deterministic for both sensors
        bh1750Init();
        delay(200);  // BH1750 first measurement settling

        // Matter node
        if (!MatterInit::init()) {
            Serial.println("ERROR: Matter node init failed — halting");
            while (true) delay(1000);
        }

        // Register endpoints (order determines endpoint IDs: EP1, EP2, EP3, EP4).
        gTempSensor.begin(0.0);
        gAmbHumidity.begin(0.0);
        gLightSensor.begin(0.0f);
        gSoilSensor.begin(0.0);

        // Start Matter / OpenThread stack.
        if (!MatterInit::start()) {
            Serial.println("ERROR: Matter stack start failed — halting");
            while (true) delay(1000);
        }

        // Set Thread child timeout to 4× wake interval so the parent router
        // keeps the child entry alive across the full deep-sleep window.
        // Default OpenThread timeout is 240 s; at 120 s sleep this is borderline.
#if CONFIG_ENABLE_MATTER_OVER_THREAD
        {
            otInstance *ot = esp_openthread_get_instance();
            if (ot) {
                otThreadSetChildTimeout(ot, ICD_WAKE_INTERVAL_S * 4);
                Serial.printf("OpenThread child timeout set to %d s\n",
                              ICD_WAKE_INTERVAL_S * 4);
            }
        }
#endif

        Serial.printf("Wake interval: %d s\n", ICD_WAKE_INTERVAL_S);
        gState = State::MATTER_READY;
        break;
    }

    // ── MATTER_READY ─────────────────────────────────────────────────────────
    // Visited once on fresh boot only (light-sleep wakeup goes IDLE_WAIT directly).
    // If already commissioned (power-cycle of provisioned device) → skip grace.
    case State::MATTER_READY: {
        if (MatterInit::isCommissioned()) {
            if (gWakeReason == ESP_SLEEP_WAKEUP_EXT1) {
                // Button woke us from deep sleep — handle as short/long press.
                // Set gActionPressedAt so the 5-second window starts immediately,
                // even if the button is released before loop() first runs the case.
                Serial.println("Woke from button press — entering ACTION_BUTTON_PRESSED.");
                gActionPressedAt = millis();
                gState = State::ACTION_BUTTON_PRESSED;
            } else {
                Serial.println("Already commissioned — going directly to sensor cycle.");
                gIdleStart = millis();
                gState = State::IDLE_WAIT;
            }
        } else {
            gState = State::MATTER_DECOMMISSIONED;
        }
        break;
    }

    // ── MATTER_DECOMMISSIONED ────────────────────────────────────────────────
    // Wait for BLE commissioning then Thread network join.
    // Once both are satisfied, enter the post-commissioning grace period.
    case State::MATTER_DECOMMISSIONED: {
        if (!MatterInit::isCommissioned()) {
            static uint32_t lastPrint = 0;
            if (millis() - lastPrint >= 10000) {
                lastPrint = millis();
                Serial.println("Waiting for commissioning via BLE…");
                Serial.printf("  Setup discriminator : %d\n", MATTER_DEVICE_SETUP_DISCRIMINATOR);
                Serial.printf("  Setup passcode      : %lu\n", (unsigned long)MATTER_DEVICE_SETUP_PASSCODE);
            }
            break;
        }

        if (!MatterInit::isConnected()) {
            static uint32_t lastPrint = 0;
            if (millis() - lastPrint >= 5000) {
                lastPrint = millis();
                Serial.println("Waiting for Thread network connection…");
            }
            break;
        }

        gState = State::COMMISSIONING_GRACE;
        break;
    }

    // ── COMMISSIONING_GRACE ──────────────────────────────────────────────────
    // Wait for the Matter controller to establish its first subscription before
    // starting the sensor cycle.  This ensures the controller has had time to
    // complete attribute reads and ICD registration after commissioning.
    // Falls back to starting the cycle after 5 minutes if no subscription arrives.
    case State::COMMISSIONING_GRACE: {
        static uint32_t graceEnteredAt = 0;
        if (graceEnteredAt == 0) {
            graceEnteredAt = millis();
            Serial.println("Commissioning done — waiting for controller subscription...");
        }

        if (MatterInit::hasActiveSubscription()) {
            Serial.printf("Controller subscribed (%.1fs) — starting sensor cycle.\n",
                          (millis() - graceEnteredAt) / 1000.0f);
            graceEnteredAt = 0;
            gIdleStart = millis();
            gState = State::IDLE_WAIT;
            break;
        }

        // 5-minute fallback in case the controller never subscribes.
        if (millis() - graceEnteredAt >= 300000UL) {
            Serial.println("Grace timeout — starting sensor cycle anyway.");
            graceEnteredAt = 0;
            gIdleStart = millis();
            gState = State::IDLE_WAIT;
        }
        break;
    }

    // ── ACTION_BUTTON_PRESSED ────────────────────────────────────────────────
    // gActionPressedAt:   timestamp of the very first press (session start).
    // gActionDecommStart: timestamp of the current continuous hold; 0 = not held.
    //
    // Decommission if any continuous hold reaches 5 s (first press or re-press).
    // Sensor read if button is not held and 5 s have elapsed since the first press.
    case State::ACTION_BUTTON_PRESSED: {
        bool held = (digitalRead(ACTION_BUTTON_PIN) == LOW);

        if (held) {
            if (gActionPressedAt == 0)    gActionPressedAt   = millis();
            if (gActionDecommStart == 0)  gActionDecommStart = millis();
        } else {
            gActionDecommStart = 0;   // reset hold timer on release
        }

        if (gActionDecommStart != 0 && millis() - gActionDecommStart >= ACTION_CONFIRM_WINDOW_MS) {
            gActionPressedAt = 0;
            gActionDecommStart = 0;
            Serial.println("Action button: held 5 s — decommissioning…");
            MatterInit::decommission();
            delay(500);
            gState = State::MATTER_DECOMMISSIONED;
            break;
        }

        if (!held && gActionPressedAt != 0 && millis() - gActionPressedAt >= ACTION_CONFIRM_WINDOW_MS) {
            gActionPressedAt = 0;
            Serial.println("Action button: 5 s window expired — forcing sensor read.");
            gIdleStart = millis();
            gState     = State::IDLE_WAIT;
        }
        break;
    }

    // ── IDLE_WAIT ────────────────────────────────────────────────────────────
    // Ensures at least 1 s of sensor power-on time has elapsed before reading.
    // In practice Matter init + Thread rejoin takes >3 s so this is a safety
    // floor rather than a real delay.  Uses millis() — non-blocking.
    case State::IDLE_WAIT: {
        if (millis() - gSensorPowerOnMillis >= 1000) {
            gSampleCount = 0;
            // Arm BH1750 first one-shot measurement
            bh1750Trigger();
            gLightSensorMeasurementStart = millis();
            gLightSensorMeasurementArmed = true;
            gState = State::SENSOR_READ;
        }
        break;
    }

    // ── SENSOR_READ ──────────────────────────────────────────────────────────
    // Collects 3 readings from every sensor; reports the median of each.
    // BH1750 needs 180 ms per one-shot — total extra latency ≈ 360 ms.
    case State::SENSOR_READ: {
        // Wait for current BH1750 conversion to finish (~180 ms).
        if (gLightSensorMeasurementArmed && (millis() - gLightSensorMeasurementStart < LIGHT_SENSOR_MEASUREMENT_MILLIS)) break;
        gLightSensorMeasurementArmed = false;

        // SHT4x read first: its WRITE command is first on the bus each sub-cycle (clean bus).
        // BH1750 READ follows SHT4x READ — READ→READ transition is always clean.
        // (Reversed order fixes READ→WRITE back-to-back stall on ESP32-H2 Wire.)
        if (!sht4xRead(gTemperatureReadings[gSampleCount], gHumidityReadings[gSampleCount])) {
            Serial.printf("SHT4x read failed at sample %u — using previous values\n", gSampleCount);
            gTemperatureReadings[gSampleCount] = gTemperatureCelsius;
            gHumidityReadings[gSampleCount]    = gHumidityPercent;
        }

        if (!bh1750Read(gLuxReadings[gSampleCount])) {
            Serial.printf("BH1750 read failed at sample %u — using previous value\n", gSampleCount);
            gLuxReadings[gSampleCount] = gLux;
        }

        gSoilMoistureReadings[gSampleCount]     = readSoilMoisture();
        gBatteryReadings[gSampleCount]           = readBatteryPercent();  // also sets gBatteryMillivolts
        gBatteryMillivoltReadings[gSampleCount]  = gBatteryMillivolts;
        gIsCharging = readChargeDetect();

        gSampleCount++;

        if (gSampleCount < 3) {
            // Trigger next BH1750 one-shot and stay in SENSOR_READ.
            bh1750Trigger();
            gLightSensorMeasurementStart = millis();
            gLightSensorMeasurementArmed = true;
        } else {
            // All 3 samples collected — compute medians.
            gTemperatureCelsius = median3(gTemperatureReadings[0],      gTemperatureReadings[1],      gTemperatureReadings[2]);
            gHumidityPercent    = median3(gHumidityReadings[0],         gHumidityReadings[1],         gHumidityReadings[2]);
            gLux                = median3(gLuxReadings[0],              gLuxReadings[1],              gLuxReadings[2]);
            gSoilPercent        = median3(gSoilMoistureReadings[0],     gSoilMoistureReadings[1],     gSoilMoistureReadings[2]);
            gBatteryPercent     = median3(gBatteryReadings[0],          gBatteryReadings[1],          gBatteryReadings[2]);
            gBatteryMillivolts  = median3(gBatteryMillivoltReadings[0], gBatteryMillivoltReadings[1], gBatteryMillivoltReadings[2]);
            Serial.printf("Sensors (median/3) → T=%.2f°C  RH=%.1f%%  Lux=%.1f  Soil=%.1f%%  Bat=%u%%\n",
                          gTemperatureCelsius, gHumidityPercent, gLux, gSoilPercent, gBatteryPercent);
            gState = State::DATA_PUSH;
        }
        break;
    }

    // ── DATA_PUSH ────────────────────────────────────────────────────────────
    case State::DATA_PUSH: {
        // Only push if the change exceeds the threshold (saves radio traffic).
        static double  lastTemperature  = -999.0;
        static double  lastHumidity     = -1.0;
        static float   lastLux          = -1.0f;
        static double  lastSoilMoisture = -1.0;
        static uint8_t lastBattery      = 255;
        static bool    lastCharging     = false;

        // Step 1: Wait for Thread + subscription BEFORE touching attributes.
        // Updating attributes before Thread connects triggers immediate failed CASE
        // session attempts, corrupting retry state before the wait begins.
        // 90 s covers: CIP Check-In round-trip + CASE establishment + MRP retransmits.
        if (!MatterInit::waitForSubscription(90000)) {
            Serial.println("DATA_PUSH: no subscription — sleeping anyway");
            gState = State::POWER_SAVE;
            break;
        }

        // Step 2: Subscription is active — update attributes now.
        if (fabs(gTemperatureCelsius - lastTemperature) >= TEMPERATURE_THRESHOLD_CELSIUS) {
            gTempSensor.setTemperature(gTemperatureCelsius);
            lastTemperature = gTemperatureCelsius;
        }
        if (fabs(gHumidityPercent - lastHumidity) >= HUMIDITY_THRESHOLD) {
            gAmbHumidity.setHumidity(gHumidityPercent);
            lastHumidity = gHumidityPercent;
        }
        float luxChange = (lastLux > 0.0f) ? fabsf(gLux - lastLux) / lastLux * 100.0f : 100.0f;
        if (luxChange >= LUX_THRESHOLD_PERCENT) {
            gLightSensor.setLux(gLux);
            lastLux = gLux;
        }
        if (fabs(gSoilPercent - lastSoilMoisture) >= SOIL_THRESHOLD) {
            gSoilSensor.setMoisture(gSoilPercent);
            lastSoilMoisture = gSoilPercent;
        }
        if (abs((int)gBatteryPercent - (int)lastBattery) >= BATTERY_THRESHOLD) {
            MatterInit::setBatteryPercent(gBatteryPercent);
            MatterInit::setBatteryVoltage(gBatteryMillivolts);
            lastBattery = gBatteryPercent;
        }
        if (gIsCharging != lastCharging) {
            MatterInit::setBatteryChargeState(gIsCharging);
            lastCharging = gIsCharging;
        }

        // Step 3: Drain dirty reports (subscription already active — should be fast).
        if (!MatterInit::waitForDirtyDrain(5000)) {
            Serial.println("DATA_PUSH: drain timeout — sleeping anyway");
        }

        gState = State::POWER_SAVE;
        break;
    }

    // ── POWER_SAVE ───────────────────────────────────────────────────────────
    // Enter deep sleep.  Thread link is dropped; on wakeup setup() re-initialises
    // the full Matter + OpenThread stack from NVS (~3–10 s Thread rejoin time).
    // Timer wakeup fires after ICD_WAKE_INTERVAL_S for the next sensor reading.
    // EXT1 wakeup fires on ACTION_BUTTON_PIN LOW (button press).
    case State::POWER_SAVE: {
        // If the commissioning window is open (e.g. kFabricRemoved fired while we
        // were mid-cycle and eventCB re-opened it), do NOT sleep — a sleep here
        // would lock out commissioning for up to ICD_WAKE_INTERVAL_S.
        if (MatterInit::isCommissioningWindowOpen()) {
            Serial.println("Commissioning window open — staying awake for recommissioning");
            gState = State::MATTER_DECOMMISSIONED;
            break;
        }

        // Wait for the ICD to transition from ActiveMode to IdleMode before sleeping.
        // After DATA_PUSH drains all attribute reports, the ICD active-mode timer
        // counts down and fires OnEnterIdleMode() — typically within a few seconds.
        // Cap at 30 s to avoid hanging if the callback is never received.
        static uint32_t icdWaitStart = 0;
        if (icdWaitStart == 0) icdWaitStart = millis();

        if (!MatterInit::isIcdIdle()) {
            static uint32_t lastIcdLog = 0;
            if (millis() - lastIcdLog >= 5000) {
                lastIcdLog = millis();
                Serial.printf("POWER_SAVE: waiting for ICD idle... (%lu s)\n",
                              (millis() - icdWaitStart) / 1000);
            }
            if (millis() - icdWaitStart < 30000) break;  // yield — check next loop
            Serial.println("POWER_SAVE: ICD idle timeout — sleeping anyway");
        } else {
            Serial.println("ICD idle — entering deep sleep.");
        }
        icdWaitStart = 0;  // reset so next wakeup cycle starts fresh

        // Power off sensors before entering deep sleep.
        digitalWrite(static_cast<gpio_num_t>(SENSOR_PWR_PIN), LOW);

        // Release I2C bus and float SDA/SCL to prevent leakage current during sleep.
        Wire.end();
        pinMode(I2C_SDA_PIN, INPUT);
        pinMode(I2C_SCL_PIN, INPUT);

        Serial.printf("Cycle complete — active for %lu ms. Entering deep sleep for %d s…\n",
              millis() - gWakeStartMillis, ICD_WAKE_INTERVAL_S);
        Serial.flush();
        delay(100);  // drain UART TX buffer before power-down

        esp_sleep_enable_timer_wakeup(
            static_cast<uint64_t>(ICD_WAKE_INTERVAL_S) * 1000000ULL);
        // gpio_wakeup_enable is light-sleep only; use EXT1 for deep sleep.
        esp_sleep_enable_ext1_wakeup(1ULL << ACTION_BUTTON_PIN,
                                     ESP_EXT1_WAKEUP_ANY_LOW);
        esp_deep_sleep_start();
        // Does not return — next execution begins in setup() on wakeup.
        break;
    }

    }  // end switch
}
