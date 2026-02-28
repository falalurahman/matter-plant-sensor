// MatterCustom.h
// Minimalist Matter node initializer — bypasses Matter.h and ArduinoMatter.
// Includes only MatterEndPoint.h from the arduino-esp32 core.
// Direct SDK access via esp_matter::node::create().

#pragma once
#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include <Arduino.h>
#include <esp_matter.h>
#include <MatterEndPoint.h>

#if CONFIG_ENABLE_MATTER_OVER_THREAD
#include "esp_openthread_types.h"
#include "platform/ESP32/OpenthreadLauncher.h"
#endif

// ─── MatterCustomNode ────────────────────────────────────────────────────────
// Singleton wrapper for esp_matter node lifecycle.
// Call init() before creating any endpoints, then begin() after all endpoints
// have been registered to start the Matter/Thread stack.
class MatterCustomNode {
public:
    // Create the Matter root node. Must be called exactly once, before any
    // endpoint's begin() method. Adds the PowerSource (Battery) cluster to EP0.
    static bool init();

    // Start the Matter stack (OpenThread + BLE commissioning).
    // Call after all endpoints have called begin().
    static bool start();

    // ── Status queries (usable after start()) ────────────────────────────────
    static bool isCommissioned();  // fabric table non-empty
    static bool isConnected();     // Thread network attached

    // Factory-reset all Matter credentials and restart commissioning.
    static void decommission();

    // Returns true (and clears the flag) if kCommissioningComplete fired since
    // the last call. Intended to be polled from loop() to detect fresh commissioning
    // without triggering on subsequent wakeups from light sleep.
    static bool getAndClearJustCommissioned();

    // ── Battery reporting ────────────────────────────────────────────────────
    // Update BatteryPercentageRemaining on EP0 (0–100 % → stored as 0–200).
    static bool setBatteryPercent(uint8_t percent);

    // Internal callbacks — do not call directly.
    static esp_err_t attrUpdateCB(
        esp_matter::attribute::callback_type_t type,
        uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id,
        esp_matter_attr_val_t *val, void *priv_data);

    static esp_err_t identifyCB(
        esp_matter::identification::callback_type_t type,
        uint16_t endpoint_id, uint8_t effect_id, uint8_t effect_variant,
        void *priv_data);

private:
    static bool _initialized;
    static bool _started;
    static esp_matter::node_t *_node;
    static volatile bool _justCommissioned;

    static void eventCB(const ChipDeviceEvent *event, intptr_t arg);
};

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
