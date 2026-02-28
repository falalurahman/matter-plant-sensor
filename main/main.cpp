// main.cpp — ESP32-H2 Matter Plant Sensor
//
// Architecture: non-blocking FSM, Matter-over-Thread, ICD (Sleepy End Device).
// No dependency on the high-level Matter.h wrapper; uses MatterCustom + direct
// esp_matter SDK calls throughout.
//
// Hardware (ESP32-H2 SuperMini):
//   GPIO 1 (ADC1_CH0) — capacitive soil-moisture sensor
//   GPIO 2 (ADC1_CH1) — battery voltage via 1:1 resistor divider
//   GPIO 4 (SDA) / GPIO 5 (SCL) — I2C bus
//     0x44 — SHT4x  (ambient temperature + humidity)
//     0x23 — BH1750 (ambient light, lux)

#include <Arduino.h>
#include <Wire.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <math.h>

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
static constexpr uint8_t  PIN_BATTERY_ADC  = 2;
static constexpr uint8_t  PIN_I2C_SDA      = 4;
static constexpr uint8_t  PIN_I2C_SCL      = 5;
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
static uint8_t gBatteryPct   = 0;

// ─── Timing ──────────────────────────────────────────────────────────────────
static uint32_t gIdleStart       = 0;
static uint32_t gBH1750Start     = 0;
static bool     gBH1750Armed     = false;

// ─── RTC state (survives light sleep) ────────────────────────────────────────
RTC_DATA_ATTR static uint32_t gBootCount = 0;

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

    delay(10);  // SHT4x high-precision measurement time ≤ 8.3 ms

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
// Send the "Power On" + "Continuous High-Resolution Mode 2" commands.
// Must be called once during init; measurements start automatically.
static void bh1750Init() {
    Wire.beginTransmission(I2C_ADDR_BH1750);
    Wire.write(0x01);  // Power On
    Wire.endTransmission();
    delay(1);
    Wire.beginTransmission(I2C_ADDR_BH1750);
    Wire.write(0x11);  // Continuous High-Resolution Mode 2 (0.5 lx, 120 ms typ)
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
    float pct = (vbat_mv - VBAT_MIN_MV) / (VBAT_MAX_MV - VBAT_MIN_MV) * 100.0f;
    return static_cast<uint8_t>(constrain(pct, 0.0f, 100.0f));
}

// ════════════════════════════════════════════════════════════════════════════
// ── Arduino entry points ─────────────────────────────────────────────────────
// ════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    gBootCount++;
    Serial.printf("\n=== Plant Sensor boot #%lu ===\n", gBootCount);

    esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();
    Serial.printf("Wakeup reason: %d\n", wakeReason);

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
    case State::SYSTEM_BOOT: {
        // Action button
        pinMode(ACTION_BUTTON_PIN, INPUT_PULLUP);

        // ADC
        analogSetAttenuation(ADC_11db);

        // I2C
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
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

        Serial.printf("Wake interval: %d s\n", ICD_WAKE_INTERVAL_S);
        gState = State::MATTER_READY;
        break;
    }

    // ── MATTER_READY ─────────────────────────────────────────────────────────
    // Visited once on fresh boot only (light-sleep wakeup goes IDLE_WAIT directly).
    // If already commissioned (power-cycle of provisioned device) → skip grace.
    case State::MATTER_READY: {
        if (MatterCustomNode::isCommissioned()) {
            Serial.println("Already commissioned — going directly to sensor cycle.");
            gIdleStart = millis();
            gState = State::IDLE_WAIT;
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
    // Short active-wait so the Matter stack can process any queued messages
    // before we enter sensor reading.  Uses millis() — non-blocking.
    case State::IDLE_WAIT: {

        // On first boot give the stack 2 s to settle after commissioning/join.
        if (millis() - gIdleStart >= 2000) {
            gState = State::SENSOR_READ;
            // Arm BH1750 one-shot measurement
            bh1750Trigger();
            gBH1750Start = millis();
            gBH1750Armed = true;
        }
        break;
    }

    // ── SENSOR_READ ──────────────────────────────────────────────────────────
    case State::SENSOR_READ: {
        // Wait for BH1750 conversion to finish (~180 ms).
        if (gBH1750Armed && (millis() - gBH1750Start < BH1750_MEAS_MS)) break;
        gBH1750Armed = false;

        // Read SHT4x (blocking, ~10 ms).
        if (!sht4xRead(gTempC, gHumidityPct)) {
            log_w("SHT4x read failed — using previous values");
        }

        // Read BH1750.
        if (!bh1750Read(gLux)) {
            log_w("BH1750 read failed — using previous value");
        }

        // Read ADC sensors.
        gSoilPct    = readSoilMoisture();
        gBatteryPct = readBatteryPercent();

        Serial.printf("Sensors → T=%.2f°C  RH=%.1f%%  Lux=%.1f  Soil=%.1f%%  Bat=%u%%\n",
                      gTempC, gHumidityPct, gLux, gSoilPct, gBatteryPct);

        gState = State::DATA_PUSH;
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
            lastBat = gBatteryPct;
        }

        // Allow the Matter stack a moment to dispatch the updates.
        delay(1000);

        gState = State::POWER_SAVE;
        break;
    }

    // ── POWER_SAVE ───────────────────────────────────────────────────────────
    // Enter light sleep.  The OpenThread SED stack wakes automatically every
    // CONFIG_ICD_SLOW_POLL_INTERVAL_MS to maintain the Thread parent link.
    // Our timer wakeup fires after ICD_WAKE_INTERVAL_S for the next reading.
    case State::POWER_SAVE: {
        Serial.printf("Entering light sleep for %d s…\n", ICD_WAKE_INTERVAL_S);
        Serial.flush();

        esp_sleep_enable_timer_wakeup(
            static_cast<uint64_t>(ICD_WAKE_INTERVAL_S) * 1000000ULL);
        gpio_wakeup_enable((gpio_num_t)ACTION_BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
        esp_sleep_enable_gpio_wakeup();
        esp_light_sleep_start();

        // Execution resumes here after wakeup (light sleep preserves stack/state).
        auto wakeReason = esp_sleep_get_wakeup_cause();
        if (wakeReason == ESP_SLEEP_WAKEUP_GPIO) {
            Serial.println("Woke from button press.");
            gState = State::ACTION_BUTTON_PRESSED;
        } else {
            Serial.println("Woke from timer.");
            gIdleStart = millis();
            gState = State::IDLE_WAIT;
        }
        break;
    }

    }  // end switch
}
