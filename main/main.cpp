// main.cpp — ESP32-H2 Matter Plant Sensor
//
// Architecture: non-blocking FSM, Matter-over-Thread, ICD (Sleepy End Device).
// No dependency on the high-level Matter.h wrapper; uses MatterCustom + direct
// esp_matter SDK calls throughout.
//
// Hardware (ESP32-H2 SuperMini):
//   GPIO 1 (ADC1_CH0) — capacitive soil-moisture sensor
//   GPIO 2 (ADC1_CH1) — battery voltage via 1:1 resistor divider
//   GPIO 3            — NPN transistor base (HIGH = sensors powered on)
//   GPIO 4 (SDA) / GPIO 5 (SCL) — I2C bus
//     0x44 — SHT4x  (ambient temperature + humidity)
//     0x23 — BH1750 (ambient light, lux)

#include <Arduino.h>
#include <Wire.h>
#include <esp_sleep.h>
#include <math.h>
#if CONFIG_ENABLE_MATTER_OVER_THREAD
#include <esp_openthread.h>
#include <openthread/thread.h>
#endif

#include "MatterCustom.h"
#include "MatterTempSensor.h"
#include "MatterAmbientHumidity.h"
#include "MatterLightSensor.h"
#include "MatterSoilSensor.h"

// ─── Build-flag defaults ────────────────────────────────────────────────────
#ifndef SOIL_MOISTURE_PIN
#define SOIL_MOISTURE_PIN 1
#endif
#ifndef ICD_WAKE_INTERVAL_S
#define ICD_WAKE_INTERVAL_S 3600  // 1 hour; mirrors CONFIG_ICD_IDLE_MODE_INTERVAL_SEC
#endif
#ifndef COMMISSIONING_GRACE_MS
#define COMMISSIONING_GRACE_MS 30000  // 10s stay-awake after first commissioning
#endif

// ─── Hardware constants ──────────────────────────────────────────────────────
static constexpr uint8_t     PIN_BATTERY_ADC  = 2;
static constexpr gpio_num_t  PIN_SENSOR_PWR   = GPIO_NUM_3;  // NPN base — HIGH=sensors on
static constexpr uint8_t     PIN_I2C_SDA      = 4;
static constexpr uint8_t     PIN_I2C_SCL      = 5;
static constexpr uint8_t  I2C_ADDR_SHT4X   = 0x44;
static constexpr uint8_t  I2C_ADDR_BH1750  = 0x23;

// Action button (BOOT pin) — short press: force sensor read; long press: decommission
static constexpr uint8_t  ACTION_BUTTON_PIN    = BOOT_PIN;
static constexpr uint32_t ACTION_LONG_PRESS_MS = 5000;

// Battery voltage limits (mV) for a single-cell LiPo, 1:1 divider on ADC.
// ADC attenuation 11 dB → full-scale ~3100 mV.  Divider halves Vbat.
static constexpr float VBAT_MAX_MV = 4200.0f;
static constexpr float VBAT_MIN_MV = 3000.0f;
static constexpr float ADC_FULL_MV = 3100.0f;

// Minimum change required before pushing a Matter attribute update.
static constexpr double TEMP_THRESHOLD_C    = 0.1;   // °C
static constexpr double HUMIDITY_THRESHOLD  = 0.5;   // %
static constexpr float  LUX_THRESHOLD_PCT   = 5.0f;  // % relative change
static constexpr double SOIL_THRESHOLD      = 0.5;   // %
static constexpr uint8_t BATTERY_THRESHOLD  = 1;     // %

// BH1750 measurement time for Continuous High-Resolution Mode 2 (0.5 lx).
static constexpr uint32_t BH1750_MEAS_MS = 180;

// ─── Matter endpoints ────────────────────────────────────────────────────────
static MatterTempSensor     gTempSensor;
static MatterAmbientHumidity gAmbHumidity;
static MatterLightSensor    gLightSensor;
static MatterSoilSensor     gSoilSensor;

// ─── FSM ─────────────────────────────────────────────────────────────────────
enum class State : uint8_t {
    SYSTEM_BOOT,
    MATTER_READY,
    MATTER_DECOMMISSIONED,   // wait for BLE commission + Thread join
    COMMISSIONING_GRACE,     // idle hold so controller can finish setup
    ACTION_BUTTON_PRESSED,   // button held: short press → force read, long press → decommission
    IDLE_WAIT,
    SENSOR_READ,
    DATA_PUSH,
    POWER_SAVE,
};
static State gState = State::SYSTEM_BOOT;

// ─── Sensor data (last read) ─────────────────────────────────────────────────
static double  gTempC        = 0.0;
static double  gHumidityPct  = 0.0;
static float   gLux          = 0.0f;
static double  gSoilPct      = 0.0;
static uint8_t  gBatteryPct   = 0;
static uint32_t gBatteryMv    = 0;   // actual Vbat in mV (ADC × 2, 1:1 divider)

// ─── Timing ──────────────────────────────────────────────────────────────────
static uint32_t gWakeStartMs     = 0;  // millis() at start of setup() for cycle time logging
static uint32_t gSensorPowerOnMs = 0;  // millis() when sensor power GPIO went HIGH
static uint32_t gIdleStart       = 0;
static uint32_t gBH1750Start     = 0;
static bool     gBH1750Armed     = false;

// ─── Sensor sampling (3× median) ─────────────────────────────────────────────
static uint8_t gReadCount          = 0;
static double  gTempReadings[3]    = {};
static double  gHumReadings[3]     = {};
static float   gLuxReadings[3]     = {};
static double  gSoilReadings[3]    = {};
static uint8_t  gBatReadings[3]    = {};
static uint32_t gBatMvReadings[3]  = {};

// ─── RTC state (survives deep sleep) ─────────────────────────────────────────
RTC_DATA_ATTR static uint32_t               gBootCount  = 0;
RTC_DATA_ATTR static esp_sleep_wakeup_cause_t gWakeReason = ESP_SLEEP_WAKEUP_UNDEFINED;

// ════════════════════════════════════════════════════════════════════════════
// ── I2C drivers (minimal, no external library) ───────────────────────────────
// ════════════════════════════════════════════════════════════════════════════

// ── SHT4x ────────────────────────────────────────────────────────────────────
// Trigger a high-precision measurement (0xFD) and synchronously read the result.
// Returns false if the sensor is absent or CRC fails.
static bool sht4xRead(double &tempC, double &humPct) {
    Wire.beginTransmission(I2C_ADDR_SHT4X);
    Wire.write(0xFD);  // Measure T+RH, high precision
    if (Wire.endTransmission() != 0) return false;

    delay(15);  // SHT4x high-precision: 8.3 ms typical; 15 ms gives margin

    if (Wire.requestFrom(static_cast<uint8_t>(I2C_ADDR_SHT4X), static_cast<uint8_t>(6)) != 6)
        return false;

    uint8_t buf[6];
    for (auto &b : buf) b = Wire.read();

    uint16_t rawT  = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    uint16_t rawRH = (static_cast<uint16_t>(buf[3]) << 8) | buf[4];

    tempC  = -45.0 + 175.0 * rawT  / 65535.0;
    humPct = constrain(-6.0 + 125.0 * rawRH / 65535.0, 0.0, 100.0);
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

// Read the most recent BH1750 result.  Call after BH1750_MEAS_MS has elapsed.
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
    // Capacitive sensor: dry = high ADC (4095), wet = low ADC (0).  Invert.
    double pct = map(raw, 4095, 0, 0, 10000) / 100.0;
    return constrain(pct, 0.0, 100.0);
}

static uint8_t readBatteryPercent() {
    uint16_t raw = analogRead(PIN_BATTERY_ADC);
    float adc_mv  = raw * ADC_FULL_MV / 4095.0f;
    float vbat_mv = adc_mv * 2.0f;  // 1:1 divider → actual Vbat = 2× ADC voltage
    gBatteryMv = static_cast<uint32_t>(vbat_mv);  // store for BatVoltage attribute
    float pct = (vbat_mv - VBAT_MIN_MV) / (VBAT_MAX_MV - VBAT_MIN_MV) * 100.0f;
    return static_cast<uint8_t>(constrain(pct, 0.0f, 100.0f));
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
    gWakeStartMs = millis();  // Record wake time for end-of-cycle log

    // Power on sensors immediately via NPN transistor on GPIO 3.
    // Sensors stabilize while the Matter stack initializes below (~3–10 s).
    pinMode(PIN_SENSOR_PWR, OUTPUT);
    digitalWrite(PIN_SENSOR_PWR, HIGH);
    gSensorPowerOnMs = millis();

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
        gState = State::ACTION_BUTTON_PRESSED;
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
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
        Wire.setClock(100000);  // 100 kHz standard mode — explicit, deterministic for both sensors
        bh1750Init();
        delay(200);  // BH1750 first measurement settling

        // Matter node
        if (!MatterCustomNode::init()) {
            Serial.println("ERROR: Matter node init failed — halting");
            while (true) delay(1000);
        }

        // Register endpoints (order determines endpoint IDs: EP1, EP2, EP3, EP4).
        gTempSensor.begin(0.0);
        gAmbHumidity.begin(0.0);
        gLightSensor.begin(0.0f);
        gSoilSensor.begin(0.0);

        // Start Matter / OpenThread stack.
        if (!MatterCustomNode::start()) {
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
        if (MatterCustomNode::isCommissioned()) {
            if (gWakeReason == ESP_SLEEP_WAKEUP_EXT1) {
                // Button woke us from deep sleep — handle as short/long press.
                Serial.println("Woke from button press — entering ACTION_BUTTON_PRESSED.");
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
        if (!MatterCustomNode::isCommissioned()) {
            static uint32_t lastPrint = 0;
            if (millis() - lastPrint >= 10000) {
                lastPrint = millis();
                Serial.println("Waiting for commissioning via BLE…");
                Serial.printf("  Setup discriminator : %d\n", MATTER_DEVICE_SETUP_DISCRIMINATOR);
                Serial.printf("  Setup passcode      : %lu\n", (unsigned long)MATTER_DEVICE_SETUP_PASSCODE);
            }
            break;
        }

        if (!MatterCustomNode::isConnected()) {
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
    // Sit idle so the Matter controller (e.g. Apple Home) can complete attribute
    // reads, subscriptions, and ICD registration before we start the sensor cycle.
    case State::COMMISSIONING_GRACE: {
        static uint32_t graceEnteredAt = 0;
        static uint32_t stayAwakeUntil = 0;

        if (graceEnteredAt == 0) graceEnteredAt = millis();

        // Arm the 5-minute timer once kCommissioningComplete fires.
        if (stayAwakeUntil == 0 && MatterCustomNode::getAndClearJustCommissioned()) {
            stayAwakeUntil = millis() + COMMISSIONING_GRACE_MS;
            Serial.printf("Commissioning done — holding awake for %u s\n",
                          COMMISSIONING_GRACE_MS / 1000);
        }

        // Stay idle while the grace timer is running.
        if (stayAwakeUntil > 0 && millis() < stayAwakeUntil) {
            static uint32_t lastGraceLog = 0;
            if (millis() - lastGraceLog >= 15000) {
                lastGraceLog = millis();
                Serial.printf("  Grace: %lu s remaining...\n",
                              (stayAwakeUntil - millis()) / 1000);
            }
            break;
        }

        // Grace expired (or skipped) — start sensor cycle.
        if (stayAwakeUntil > 0) {
            graceEnteredAt = 0;
            stayAwakeUntil = 0;
            Serial.println("Grace complete — starting sensor cycle.");
            gIdleStart = millis();
            gState = State::IDLE_WAIT;
        }
        break;
    }

    // ── ACTION_BUTTON_PRESSED ────────────────────────────────────────────────
    // Tracks how long the action button is held.
    // Short press (<5 s): force an immediate sensor read cycle.
    // Long press (≥5 s): factory-reset Matter credentials → restart.
    case State::ACTION_BUTTON_PRESSED: {
        static uint32_t pressedAt = 0;
        if (pressedAt == 0) pressedAt = millis();

        bool held = (digitalRead(ACTION_BUTTON_PIN) == LOW);

        if (!held) {
            // Released before timeout — force sensor read.
            pressedAt = 0;
            Serial.println("Action button: short press — forcing sensor read.");
            gIdleStart = millis();
            gState = State::IDLE_WAIT;
            break;
        }

        if (millis() - pressedAt >= ACTION_LONG_PRESS_MS) {
            // Held long enough — decommission.
            pressedAt = 0;
            Serial.println("Action button: long press — decommissioning…");
            MatterCustomNode::decommission();
            delay(500);
            gState = State::MATTER_DECOMMISSIONED;
        }
        break;
    }

    // ── IDLE_WAIT ────────────────────────────────────────────────────────────
    // Ensures at least 1 s of sensor power-on time has elapsed before reading.
    // In practice Matter init + Thread rejoin takes >3 s so this is a safety
    // floor rather than a real delay.  Uses millis() — non-blocking.
    case State::IDLE_WAIT: {
        if (millis() - gSensorPowerOnMs >= 1000) {
            gReadCount = 0;
            // Arm BH1750 first one-shot measurement
            bh1750Trigger();
            gBH1750Start = millis();
            gBH1750Armed = true;
            gState = State::SENSOR_READ;
        }
        break;
    }

    // ── SENSOR_READ ──────────────────────────────────────────────────────────
    // Collects 3 readings from every sensor; reports the median of each.
    // BH1750 needs 180 ms per one-shot — total extra latency ≈ 360 ms.
    case State::SENSOR_READ: {
        // Wait for current BH1750 conversion to finish (~180 ms).
        if (gBH1750Armed && (millis() - gBH1750Start < BH1750_MEAS_MS)) break;
        gBH1750Armed = false;

        // SHT4x read first: its WRITE command is first on the bus each sub-cycle (clean bus).
        // BH1750 READ follows SHT4x READ — READ→READ transition is always clean.
        // (Reversed order fixes READ→WRITE back-to-back stall on ESP32-H2 Wire.)
        if (!sht4xRead(gTempReadings[gReadCount], gHumReadings[gReadCount])) {
            log_w("SHT4x read failed at sample %u — using previous values", gReadCount);
            gTempReadings[gReadCount] = gTempC;
            gHumReadings[gReadCount]  = gHumidityPct;
        }

        if (!bh1750Read(gLuxReadings[gReadCount])) {
            log_w("BH1750 read failed at sample %u — using previous value", gReadCount);
            gLuxReadings[gReadCount] = gLux;
        }
        
        gSoilReadings[gReadCount]  = readSoilMoisture();
        gBatReadings[gReadCount]   = readBatteryPercent();  // also sets gBatteryMv
        gBatMvReadings[gReadCount] = gBatteryMv;

        gReadCount++;

        if (gReadCount < 3) {
            // Trigger next BH1750 one-shot and stay in SENSOR_READ.
            bh1750Trigger();
            gBH1750Start = millis();
            gBH1750Armed = true;
        } else {
            // All 3 samples collected — compute medians.
            gTempC       = median3(gTempReadings[0], gTempReadings[1], gTempReadings[2]);
            gHumidityPct = median3(gHumReadings[0],  gHumReadings[1],  gHumReadings[2]);
            gLux         = median3(gLuxReadings[0],  gLuxReadings[1],  gLuxReadings[2]);
            gSoilPct     = median3(gSoilReadings[0], gSoilReadings[1], gSoilReadings[2]);
            gBatteryPct  = median3(gBatReadings[0],   gBatReadings[1],   gBatReadings[2]);
            gBatteryMv   = median3(gBatMvReadings[0], gBatMvReadings[1], gBatMvReadings[2]);
            log_i("Sensors (median/3) → T=%.2f°C  RH=%.1f%%  Lux=%.1f  Soil=%.1f%%  Bat=%u%%",
                  gTempC, gHumidityPct, gLux, gSoilPct, gBatteryPct);
            gState = State::DATA_PUSH;
        }
        break;
    }

    // ── DATA_PUSH ────────────────────────────────────────────────────────────
    case State::DATA_PUSH: {
        // Only push if the change exceeds the threshold (saves radio traffic).
        static double  lastTemp    = -999.0;
        static double  lastHum     = -1.0;
        static float   lastLux     = -1.0f;
        static double  lastSoil    = -1.0;
        static uint8_t lastBat     = 255;

        // Step 1: Wait for Thread + subscription BEFORE touching attributes.
        // Updating attributes before Thread connects triggers immediate failed CASE
        // session attempts, corrupting retry state before the wait begins.
        // 90 s covers: CIP Check-In round-trip + CASE establishment + MRP retransmits.
        if (!MatterCustomNode::waitForSubscription(90000)) {
            log_w("DATA_PUSH: no subscription — sleeping anyway");
            gState = State::POWER_SAVE;
            break;
        }

        // Step 2: Subscription is active — update attributes now.
        if (fabs(gTempC - lastTemp) >= TEMP_THRESHOLD_C) {
            gTempSensor.setTemperature(gTempC);
            lastTemp = gTempC;
        }
        if (fabs(gHumidityPct - lastHum) >= HUMIDITY_THRESHOLD) {
            gAmbHumidity.setHumidity(gHumidityPct);
            lastHum = gHumidityPct;
        }
        float luxChange = (lastLux > 0.0f) ? fabsf(gLux - lastLux) / lastLux * 100.0f : 100.0f;
        if (luxChange >= LUX_THRESHOLD_PCT) {
            gLightSensor.setLux(gLux);
            lastLux = gLux;
        }
        if (fabs(gSoilPct - lastSoil) >= SOIL_THRESHOLD) {
            gSoilSensor.setMoisture(gSoilPct);
            lastSoil = gSoilPct;
        }
        if (abs((int)gBatteryPct - (int)lastBat) >= BATTERY_THRESHOLD) {
            MatterCustomNode::setBatteryPercent(gBatteryPct);
            MatterCustomNode::setBatteryVoltage(gBatteryMv);
            lastBat = gBatteryPct;
        }

        // Step 3: Drain dirty reports (subscription already active — should be fast).
        if (!MatterCustomNode::waitForDirtyDrain(5000)) {
            log_w("DATA_PUSH: drain timeout — sleeping anyway");
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
        if (MatterCustomNode::isCommissioningWindowOpen()) {
            log_i("Commissioning window open — staying awake for recommissioning");
            gState = State::MATTER_DECOMMISSIONED;
            break;
        }

        // Wait for the ICD to transition from ActiveMode to IdleMode before sleeping.
        // After DATA_PUSH drains all attribute reports, the ICD active-mode timer
        // counts down and fires OnEnterIdleMode() — typically within a few seconds.
        // Cap at 30 s to avoid hanging if the callback is never received.
        static uint32_t icdWaitStart = 0;
        if (icdWaitStart == 0) icdWaitStart = millis();

        if (!MatterCustomNode::isIcdIdle()) {
            static uint32_t lastIcdLog = 0;
            if (millis() - lastIcdLog >= 5000) {
                lastIcdLog = millis();
                log_i("POWER_SAVE: waiting for ICD idle... (%lu s)",
                      (millis() - icdWaitStart) / 1000);
            }
            if (millis() - icdWaitStart < 30000) break;  // yield — check next loop
            log_w("POWER_SAVE: ICD idle timeout — sleeping anyway");
        } else {
            Serial.println("ICD idle — entering deep sleep.");
        }
        icdWaitStart = 0;  // reset so next wakeup cycle starts fresh

        // Power off sensors before entering deep sleep.
        digitalWrite(PIN_SENSOR_PWR, LOW);

        Serial.printf("Cycle complete — active for %lu ms. Entering deep sleep for %d s…\n",
              millis() - gWakeStartMs, ICD_WAKE_INTERVAL_S);
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
