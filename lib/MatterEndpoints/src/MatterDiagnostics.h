// MatterDiagnostics.h
// Adds three diagnostic clusters to EP0:
//   - General Diagnostics      (0x0033) — reboot count, uptime, network interfaces
//   - Diagnostic Logs          (0x0032) — serves the in-RAM serial log buffer
//   - Thread Network Diagnostics (0x0035) — live Thread radio stats
//
// MatterDiagnostics::log() / logln() are drop-in replacements for Serial.printf()
// / Serial.println() that write to BOTH the serial port AND the Diagnostic Logs
// buffer, so every application log line is retrievable via a Matter controller.

#pragma once
#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include <Arduino.h>
#include <esp_matter.h>
#include <app/clusters/diagnostic-logs-server/DiagnosticLogsProviderDelegate.h>

class MatterDiagnostics
    : public chip::app::Clusters::DiagnosticLogs::DiagnosticLogsProviderDelegate
{
public:
    // ── Cluster registration ────────────────────────────────────────────────
    // Add the three diagnostic clusters to EP0 and register this instance as
    // the Diagnostic Logs provider delegate.
    // Must be called inside MatterInit::init() after the root node is created.
    static bool begin(esp_matter::node_t *node);

    // ── Dual-output log helpers ─────────────────────────────────────────────
    // Write to Serial AND append to the Diagnostic Logs RAM buffer.
    // Replace Serial.printf() / Serial.println() with these in application code.
    static void log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
    static void logln(const char *msg);

    // ── DiagnosticLogsProviderDelegate ──────────────────────────────────────
    // Invoked by the Matter stack when a controller sends RetrieveLogsRequest.
    // Only kEndUserSupport intent is served (the serial log buffer).
    using IntentEnum      = chip::app::Clusters::DiagnosticLogs::IntentEnum;
    using LogSessionHandle = chip::app::Clusters::DiagnosticLogs::LogSessionHandle;

    CHIP_ERROR StartLogCollection(IntentEnum intent,
                                  LogSessionHandle &outHandle,
                                  chip::Optional<uint64_t> &outTimeStamp,
                                  chip::Optional<uint64_t> &outTimeSinceBoot) override;

    CHIP_ERROR EndLogCollection(LogSessionHandle sessionHandle) override;

    CHIP_ERROR CollectLog(LogSessionHandle sessionHandle,
                          chip::MutableByteSpan &outBuffer,
                          bool &outIsEndOfLog) override;

    size_t GetSizeForIntent(IntentEnum intent) override;

    CHIP_ERROR GetLogForIntent(IntentEnum intent,
                               chip::MutableByteSpan &outBuffer,
                               chip::Optional<uint64_t> &outTimeStamp,
                               chip::Optional<uint64_t> &outTimeSinceBoot) override;

private:
    // 4 KB in-RAM log buffer — cleared on each deep-sleep wakeup (full reset).
    static constexpr size_t kLogBufSize = 4096;
    static char   _buf[kLogBufSize];
    static size_t _bufLen;

    // BDX session state (one session at a time)
    static size_t _sessionReadPos;
    static bool   _sessionActive;

    // Singleton delegate instance registered with DiagnosticLogsServer
    static MatterDiagnostics _delegate;
};

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
