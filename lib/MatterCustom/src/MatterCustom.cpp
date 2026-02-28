// MatterCustom.cpp — Matter node lifecycle (no Matter.h dependency)

#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include "MatterCustom.h"
#include <app/server/Server.h>
#include <app/InteractionModelEngine.h>
#include <platform/ConnectivityManager.h>
#include <platform/CHIPDeviceLayer.h>

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace esp_matter::cluster;
using namespace esp_matter::identification;
using namespace chip::app::Clusters;

// ── Static member storage ────────────────────────────────────────────────────
bool MatterCustomNode::_initialized      = false;
bool MatterCustomNode::_started          = false;
esp_matter::node_t *MatterCustomNode::_node = nullptr;
volatile bool MatterCustomNode::_justCommissioned = false;
MatterCustomNode::SubscriptionTracker  MatterCustomNode::_subTracker;
std::atomic<uint32_t> MatterCustomNode::_sDirtyCount{0};

// ── Event callback ───────────────────────────────────────────────────────────
void MatterCustomNode::eventCB(const ChipDeviceEvent *event, intptr_t /*arg*/) {
    switch (event->Type) {
        case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
            log_i("Matter commissioning complete");
            _justCommissioned = true;
            break;
        case chip::DeviceLayer::DeviceEventType::kFabricRemoved: {
            const chip::FabricTable & ft = chip::Server::GetInstance().GetFabricTable();
            uint8_t remaining = ft.FabricCount();
            log_i("Fabric removed — %u fabric(s) remaining:", remaining);
            uint8_t nonApple = 0;
            for (const chip::FabricInfo & fabric : ft) {
                log_i("  idx=%u FabricId=0x%016llx NodeId=0x%016llx VendorId=0x%04x",
                      (unsigned)fabric.GetFabricIndex(),
                      (unsigned long long)fabric.GetFabricId(),
                      (unsigned long long)fabric.GetNodeId(),
                      (unsigned)fabric.GetVendorId());
                if (fabric.GetVendorId() != chip::VendorId(0x1384)) nonApple++;
            }
            if (nonApple == 0) {
                log_i("No non-Apple fabrics left — re-opening commissioning window");
                chip::CommissioningWindowManager &mgr =
                    chip::Server::GetInstance().GetCommissioningWindowManager();
                constexpr auto kTimeout = chip::System::Clock::Seconds16(300);
                if (!mgr.IsCommissioningWindowOpen()) {
                    mgr.OpenBasicCommissioningWindow(
                        kTimeout, chip::CommissioningWindowAdvertisement::kDnssdOnly);
                }
            }
            break;
        }
        case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
            log_d("BLE deinitialized, memory reclaimed");
            break;
        default:
            break;
    }
}

// ── Attribute-update callback ────────────────────────────────────────────────
esp_err_t MatterCustomNode::attrUpdateCB(
    attribute::callback_type_t type,
    uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id,
    esp_matter_attr_val_t *val, void *priv_data)
{
    if (type == attribute::PRE_UPDATE && priv_data != nullptr) {
        MatterEndPoint *ep = static_cast<MatterEndPoint *>(priv_data);
        return ep->attributeChangeCB(endpoint_id, cluster_id, attribute_id, val)
               ? ESP_OK : ESP_FAIL;
    }
    return ESP_OK;
}

// ── Identify callback ────────────────────────────────────────────────────────
esp_err_t MatterCustomNode::identifyCB(
    identification::callback_type_t type,
    uint16_t endpoint_id, uint8_t effect_id, uint8_t effect_variant,
    void *priv_data)
{
    if (priv_data != nullptr) {
        MatterEndPoint *ep = static_cast<MatterEndPoint *>(priv_data);
        bool active = (type == identification::START);
        return ep->endpointIdentifyCB(endpoint_id, active) ? ESP_OK : ESP_FAIL;
    }
    return ESP_OK;
}

// ── getAndClearJustCommissioned() ────────────────────────────────────────────
bool MatterCustomNode::getAndClearJustCommissioned() {
    if (!_justCommissioned) return false;
    _justCommissioned = false;
    return true;
}

// ── init() ───────────────────────────────────────────────────────────────────
bool MatterCustomNode::init() {
    if (_initialized) return true;

    // Create Matter root node (EP0 with descriptor + basic-information clusters).
    static node::config_t node_config;
    _node = node::create(&node_config, attrUpdateCB, identifyCB);
    if (_node == nullptr) {
        log_e("MatterCustom: failed to create Matter node");
        return false;
    }

    // Add PowerSource cluster (battery feature) to root endpoint (EP0).
    endpoint_t *root_ep = endpoint::get(_node, 0);
    if (root_ep) {
        power_source::config_t ps_config;
        ps_config.status = 1;   // PowerSourceStatusEnum::kBattery
        ps_config.order  = 0;
        // Battery feature ID = 0x02 (mandatory attrs: BatChargeLevel, BatReplacementNeeded, BatReplaceability)
        cluster_t *ps_cluster = power_source::create(root_ep, &ps_config, CLUSTER_FLAG_SERVER, 0x02);
        if (ps_cluster == nullptr) {
            log_w("MatterCustom: failed to create PowerSource cluster on EP0");
        } else {
            // BatPercentRemaining is optional and not added by the battery feature automatically.
            // Add it explicitly so we can report remaining capacity (Matter range: 0–200).
            nullable<uint8_t> initPct(200);  // start at 100% (200 = 100% in half-percent units)
            nullable<uint8_t> nullVal;
            esp_matter::cluster::power_source::attribute::create_bat_percent_remaining(ps_cluster, initPct, nullVal, nullVal);
            log_i("MatterCustom: PowerSource cluster + BatPercentRemaining added to EP0");
        }
    }

    // Add ICD Management cluster to EP0 — required for LIT ICD.
    // Exposes IdleModeDuration, ActiveModeDuration, RegisteredClients to controllers.
    // Timing values come from CONFIG_ICD_* sdkconfig entries.
    if (root_ep) {
        icd_management::config_t icd_cfg{};
        cluster_t *icd_cluster = icd_management::create(
            root_ep, &icd_cfg, CLUSTER_FLAG_SERVER, 0);
        if (icd_cluster == nullptr) {
            log_w("MatterCustom: failed to create ICD Management cluster on EP0");
        } else {
            log_i("MatterCustom: ICD Management cluster added to EP0");
        }
    }

    _initialized = true;
    log_i("MatterCustom: node initialized");
    return true;
}

// ── start() ──────────────────────────────────────────────────────────────────
bool MatterCustomNode::start() {
    if (!_initialized) {
        log_e("MatterCustom: call init() before start()");
        return false;
    }
    if (_started) return true;

#if CONFIG_ENABLE_MATTER_OVER_THREAD
    esp_openthread_platform_config_t ot_config;
    memset(&ot_config, 0, sizeof(ot_config));
    ot_config.radio_config.radio_mode          = RADIO_MODE_NATIVE;
    ot_config.host_config.host_connection_mode = HOST_CONNECTION_MODE_NONE;
    ot_config.port_config.storage_partition_name = "nvs";
    ot_config.port_config.netif_queue_size     = 10;
    ot_config.port_config.task_queue_size      = 10;
    set_openthread_platform_config(&ot_config);
#endif

    esp_err_t err = esp_matter::start(eventCB);
    if (err != ESP_OK) {
        log_e("MatterCustom: esp_matter::start() failed: %d", err);
        return false;
    }
    _started = true;
    log_i("MatterCustom: Matter stack started");

    // Register subscription lifecycle callback so we can track when controllers
    // re-establish subscriptions after deep sleep wakeup.
    _subTracker.reset();
    chip::app::InteractionModelEngine::GetInstance()
        ->RegisterReadHandlerAppCallback(&_subTracker);
    log_i("MatterCustom: SubscriptionTracker registered");
    return true;
}

// ── Status queries ───────────────────────────────────────────────────────────
bool MatterCustomNode::isCommissioned() {
    return chip::Server::GetInstance().GetFabricTable().FabricCount() > 0;
}

bool MatterCustomNode::isConnected() {
    return chip::DeviceLayer::ConnectivityMgr().IsThreadAttached();
}

bool MatterCustomNode::isCommissioningWindowOpen() {
    return chip::Server::GetInstance().GetCommissioningWindowManager()
               .IsCommissioningWindowOpen();
}

void MatterCustomNode::decommission() {
    esp_matter::factory_reset();
}

// ── Battery update ───────────────────────────────────────────────────────────
bool MatterCustomNode::setBatteryPercent(uint8_t percent) {
    if (!_started) return false;
    if (percent > 100) percent = 100;

    // BatteryPercentageRemaining attribute is 0–200 (half-percent resolution).
    uint8_t raw = percent * 2;

    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    // PowerSource cluster 0x002F, BatPercentRemaining attribute 0x000E
    constexpr uint32_t kPowerSourceCluster = 0x002F;
    constexpr uint32_t kBatPercentAttr     = 0x000E;

    endpoint_t *root_ep = endpoint::get(_node, 0);
    if (!root_ep) return false;

    cluster_t *ps_cluster = cluster::get(root_ep, kPowerSourceCluster);
    if (!ps_cluster) return false;

    attribute_t *attr = attribute::get(ps_cluster, kBatPercentAttr);
    if (!attr) return false;

    attribute::get_val(attr, &val);
    val.val.u8 = raw;
    return attribute::update(0, kPowerSourceCluster, kBatPercentAttr, &val) == ESP_OK;
}

// ── SubscriptionTracker callbacks ────────────────────────────────────────────
void MatterCustomNode::SubscriptionTracker::OnSubscriptionEstablished(chip::app::ReadHandler &) {
    _count.fetch_add(1);
    log_i("MatterCustom: subscription established (active=%d)", _count.load());
}

void MatterCustomNode::SubscriptionTracker::OnSubscriptionTerminated(chip::app::ReadHandler &) {
    if (_count.load() > 0) _count.fetch_sub(1);
    log_i("MatterCustom: subscription terminated (active=%d)", _count.load());
}

// ── _checkDirtyWork ──────────────────────────────────────────────────────────
// Runs on the CHIP task thread (via PlatformMgr().ScheduleWork) to safely read
// GetNumDirtySubscriptions() without a data race.
void MatterCustomNode::_checkDirtyWork(intptr_t) {
    _sDirtyCount.store(
        (uint32_t)chip::app::InteractionModelEngine::GetInstance()
                      ->GetNumDirtySubscriptions()
    );
}

// ── waitForReportsDelivered() ────────────────────────────────────────────────
bool MatterCustomNode::waitForReportsDelivered(uint32_t timeoutMs) {
    uint32_t start = millis();

    // Phase 1: Wait for Thread connectivity.
    while (!isConnected() && (millis() - start < timeoutMs)) { delay(200); }
    if (!isConnected()) {
        log_w("MatterCustom: Thread not up after %ums — skipping report wait", timeoutMs);
        return false;
    }
    log_i("MatterCustom: Thread connected (%.1fs)", (millis() - start) / 1000.0f);

    // Phase 2: Wait for at least one subscription to be (re-)established.
    // With CONFIG_ENABLE_PERSIST_SUBSCRIPTIONS=y the SDK calls ResumeSubscriptions()
    // automatically, so the callback should fire within a few seconds of Thread up.
    uint32_t remaining = timeoutMs - (millis() - start);
    uint32_t subStart = millis();
    while (!_subTracker.hasActiveSubscription() && (millis() - subStart < remaining)) {
        delay(200);
    }
    if (!_subTracker.hasActiveSubscription()) {
        log_w("MatterCustom: no active subscriptions within timeout — controller not subscribed yet");
        return false;
    }
    log_i("MatterCustom: subscription active (%.1fs)", (millis() - start) / 1000.0f);

    // Phase 3: Poll GetNumDirtySubscriptions() on the CHIP thread until all
    // pending attribute reports have been sent (max 5 s drain window).
    constexpr uint32_t kDrainMs = 5000;
    uint32_t drainStart = millis();
    while (millis() - drainStart < kDrainMs) {
        chip::DeviceLayer::PlatformMgr().ScheduleWork(_checkDirtyWork, 0);
        delay(150);
        if (_sDirtyCount.load() == 0) break;
    }

    bool ok = (_sDirtyCount.load() == 0);
    log_i("MatterCustom: reports %s (%.1fs total)",
          ok ? "delivered" : "drain timeout — sleeping anyway",
          (millis() - start) / 1000.0f);
    return ok;
}

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
