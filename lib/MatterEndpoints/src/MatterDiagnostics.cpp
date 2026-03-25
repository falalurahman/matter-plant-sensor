// MatterDiagnostics.cpp

#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include "MatterDiagnostics.h"
#include <stdarg.h>
#include <string.h>
#include <app/clusters/diagnostic-logs-server/diagnostic-logs-server.h>

using namespace chip::app::Clusters::DiagnosticLogs;
using namespace esp_matter;
using namespace esp_matter::cluster;

// ── Static member storage ────────────────────────────────────────────────────
char   MatterDiagnostics::_buf[MatterDiagnostics::kLogBufSize];
size_t MatterDiagnostics::_bufLen         = 0;
size_t MatterDiagnostics::_sessionReadPos = 0;
bool   MatterDiagnostics::_sessionActive  = false;
MatterDiagnostics MatterDiagnostics::_delegate;

// ── begin() ─────────────────────────────────────────────────────────────────
bool MatterDiagnostics::begin(esp_matter::node_t *node)
{
    endpoint_t *root_ep = endpoint::get(node, 0);
    if (!root_ep) {
        log_e("MatterDiagnostics: root endpoint not found");
        return false;
    }

    // General Diagnostics — reboot count, uptime, network interfaces
    {
        general_diagnostics::config_t cfg{};
        cluster_t *c = general_diagnostics::create(root_ep, &cfg, CLUSTER_FLAG_SERVER);
        if (c) {
            log_i("MatterDiagnostics: GeneralDiagnostics cluster added to EP0");
        } else {
            log_w("MatterDiagnostics: failed to add GeneralDiagnostics cluster");
        }
    }

    // Diagnostic Logs — serves this instance as the log provider
    {
        diagnostic_logs::config_t cfg{};
        cluster_t *c = diagnostic_logs::create(root_ep, &cfg, CLUSTER_FLAG_SERVER);
        if (c) {
            DiagnosticLogsServer::Instance()
                .SetDiagnosticLogsProviderDelegate(0, &_delegate);
            log_i("MatterDiagnostics: DiagnosticLogs cluster added to EP0, delegate registered");
        } else {
            log_w("MatterDiagnostics: failed to add DiagnosticLogs cluster");
        }
    }

    // Thread Network Diagnostics — live Thread radio stats (Thread builds only)
#if CONFIG_ENABLE_MATTER_OVER_THREAD
    {
        thread_network_diagnostics::config_t cfg{};
        cluster_t *c = thread_network_diagnostics::create(root_ep, &cfg, CLUSTER_FLAG_SERVER);
        if (c) {
            log_i("MatterDiagnostics: ThreadNetworkDiagnostics cluster added to EP0");
        } else {
            log_w("MatterDiagnostics: failed to add ThreadNetworkDiagnostics cluster");
        }
    }
#endif

    return true;
}

// ── log() ────────────────────────────────────────────────────────────────────
void MatterDiagnostics::log(const char *fmt, ...)
{
    char tmp[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);

    if (len <= 0) return;

    // Write to serial port (same behaviour as Serial.printf)
    Serial.print(tmp);

    // Append to Diagnostic Logs RAM buffer; stop silently when full
    size_t copyLen = (size_t)len < kLogBufSize - _bufLen
                     ? (size_t)len
                     : kLogBufSize - _bufLen;
    if (copyLen > 0) {
        memcpy(_buf + _bufLen, tmp, copyLen);
        _bufLen += copyLen;
    }
}

void MatterDiagnostics::logln(const char *msg)
{
    log("%s\n", msg);
}

// ── DiagnosticLogsProviderDelegate ───────────────────────────────────────────

// GetSizeForIntent — called by the server before starting a session
size_t MatterDiagnostics::GetSizeForIntent(IntentEnum intent)
{
    return (intent == IntentEnum::kEndUserSupport) ? _bufLen : 0;
}

// GetLogForIntent — used by the ResponsePayload transfer protocol
// (fits entire log in a single response TLV field, up to ~256 bytes)
CHIP_ERROR MatterDiagnostics::GetLogForIntent(IntentEnum intent,
                                               chip::MutableByteSpan &outBuffer,
                                               chip::Optional<uint64_t> &,
                                               chip::Optional<uint64_t> &)
{
    if (intent != IntentEnum::kEndUserSupport || _bufLen == 0) {
        outBuffer.reduce_size(0);
        return CHIP_ERROR_NOT_FOUND;
    }
    size_t len = _bufLen < outBuffer.size() ? _bufLen : outBuffer.size();
    memcpy(outBuffer.data(), _buf, len);
    outBuffer.reduce_size(len);
    return CHIP_NO_ERROR;
}

// StartLogCollection — opens a BDX session for chunked log transfer
CHIP_ERROR MatterDiagnostics::StartLogCollection(IntentEnum intent,
                                                  LogSessionHandle &outHandle,
                                                  chip::Optional<uint64_t> &,
                                                  chip::Optional<uint64_t> &)
{
    if (intent != IntentEnum::kEndUserSupport || _bufLen == 0) {
        outHandle = kInvalidLogSessionHandle;
        return CHIP_ERROR_NOT_FOUND;
    }
    _sessionReadPos = 0;
    _sessionActive  = true;
    outHandle       = 0;
    return CHIP_NO_ERROR;
}

// CollectLog — delivers the next chunk over BDX
CHIP_ERROR MatterDiagnostics::CollectLog(LogSessionHandle sessionHandle,
                                          chip::MutableByteSpan &outBuffer,
                                          bool &outIsEndOfLog)
{
    if (!_sessionActive || sessionHandle != 0) {
        return CHIP_ERROR_INCORRECT_STATE;
    }
    size_t remaining = _bufLen - _sessionReadPos;
    size_t chunk     = remaining < outBuffer.size() ? remaining : outBuffer.size();
    memcpy(outBuffer.data(), _buf + _sessionReadPos, chunk);
    outBuffer.reduce_size(chunk);
    _sessionReadPos += chunk;
    outIsEndOfLog    = (_sessionReadPos >= _bufLen);
    return CHIP_NO_ERROR;
}

// EndLogCollection — closes the BDX session
CHIP_ERROR MatterDiagnostics::EndLogCollection(LogSessionHandle)
{
    _sessionActive  = false;
    _sessionReadPos = 0;
    return CHIP_NO_ERROR;
}

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
