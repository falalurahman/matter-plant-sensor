// MatterInit.cpp — Matter node lifecycle (no Matter.h dependency)

#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include "MatterInit.h"
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
bool MatterInit::_initialized      = false;
bool MatterInit::_started          = false;
esp_matter::node_t *MatterInit::_node = nullptr;
std::atomic<bool> MatterInit::_justCommissioned{false};
MatterInit::ICDObserver          MatterInit::_icdObserver;
std::atomic<bool>                      MatterInit::_icdIdleReady{false};
MatterInit::SubscriptionTracker  MatterInit::_subscriptionTracker;
std::atomic<uint32_t> MatterInit::_pendingDirtyReportCount{0};

// ── ICDObserver callbacks ────────────────────────────────────────────────────
void MatterInit::ICDObserver::OnEnterActiveMode() {
    _icdIdleReady.store(false);
    log_i("MatterCustom: ICD → ActiveMode");
}

void MatterInit::ICDObserver::OnEnterIdleMode() {
    _icdIdleReady.store(true);
    log_i("MatterCustom: ICD → IdleMode — safe to sleep");
}

// ── Event callback ───────────────────────────────────────────────────────────
void MatterInit::eventCB(const ChipDeviceEvent *event, intptr_t /*arg*/) {
    switch (event->Type) {
        case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
            log_i("Matter commissioning complete");
            _justCommissioned.store(true);
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
esp_err_t MatterInit::attrUpdateCB(
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
esp_err_t MatterInit::identifyCB(
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
bool MatterInit::getAndClearJustCommissioned() {
    return _justCommissioned.exchange(false);
}

// ── init() ───────────────────────────────────────────────────────────────────
bool MatterInit::init() {
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
        ps_config.status = static_cast<uint8_t>(PowerSource::PowerSourceStatusEnum::kActive);
        ps_config.order  = 0;
        // kBattery feature (mandatory attrs: BatChargeLevel, BatReplacementNeeded, BatReplaceability)
        cluster_t *ps_cluster = power_source::create(root_ep, &ps_config, CLUSTER_FLAG_SERVER,
                                                     static_cast<uint32_t>(PowerSource::Feature::kBattery));
        if (ps_cluster == nullptr) {
            log_w("MatterCustom: failed to create PowerSource cluster on EP0");
        } else {
            // BatPercentRemaining and BatVoltage are optional — add explicitly.
            esp_matter::cluster::power_source::attribute::create_bat_percent_remaining(ps_cluster, 0, 0, 200);
            esp_matter::cluster::power_source::attribute::create_bat_voltage(ps_cluster, 0, 0, 65535);

            // Rechargeable feature: adds kRechargeable to the feature map and creates
            // BatChargeState + BatFunctionalWhileCharging attributes.
            esp_matter::cluster::power_source::feature::rechargeable::config_t rechargeable_cfg;
            rechargeable_cfg.bat_charge_state = static_cast<uint8_t>(PowerSource::BatChargeStateEnum::kUnknown);
            rechargeable_cfg.bat_functional_while_charging = true;
            esp_matter::cluster::power_source::feature::rechargeable::add(ps_cluster, &rechargeable_cfg);

            log_i("MatterCustom: PowerSource cluster + BatPercentRemaining + BatVoltage + Rechargeable feature added to EP0");
        }
    }

    // Add ICD Management cluster to EP0 — required for LIT ICD.
    // Feature bits must match enabled sdkconfig options; mirrors esp_matter_endpoint.cpp:83-93.
    if (root_ep) {
        icd_management::config_t icd_cfg{};
        // UAT: kActuateSensor (0x10) = pressing a button wakes the device.
        icd_cfg.user_active_mode_trigger.user_active_mode_trigger_hint =
            static_cast<uint32_t>(chip::app::Clusters::IcdManagement::UserActiveModeTriggerBitmap::kActuateSensor);
        strncpy(icd_cfg.user_active_mode_trigger.user_active_mode_trigger_instruction,
                "Press the button to wake the device",
                icd_management::attribute::k_user_active_mode_trigger_instruction_length);

        cluster_t *icd_cluster = icd_management::create(
            root_ep, &icd_cfg, CLUSTER_FLAG_SERVER,
            icd_management::feature::long_idle_time_support::get_id() |
            icd_management::feature::check_in_protocol_support::get_id() |
            icd_management::feature::user_active_mode_trigger::get_id() |
            0);
        if (icd_cluster == nullptr) {
            log_w("MatterCustom: failed to create ICD Management cluster on EP0");
        } else {
            log_i("MatterCustom: ICD Management cluster added to EP0 (LIT+CIP+UAT features)");
        }
    }

    _initialized = true;
    log_i("MatterCustom: node initialized");
    return true;
}

// ── start() ──────────────────────────────────────────────────────────────────
bool MatterInit::start() {
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
    _subscriptionTracker.reset();
    chip::app::InteractionModelEngine::GetInstance()
        ->RegisterReadHandlerAppCallback(&_subscriptionTracker);
    log_i("MatterCustom: SubscriptionTracker registered");

    // Register ICD state observer to detect ActiveMode → IdleMode transitions.
    // OnEnterIdleMode() sets _icdIdleReady = true, gating deep sleep in POWER_SAVE.
    // NOTE: Server.cpp fills both default pool slots (mReportScheduler + DnssdServer)
    // before calling Init(). CHIP_CONFIG_ICD_OBSERVERS_POOL_SIZE must be ≥ 3.
    _icdIdleReady.store(false);
    auto *icdObs = chip::Server::GetInstance().GetICDManager().RegisterObserver(&_icdObserver);
    if (icdObs == nullptr) {
        log_e("MatterCustom: ICDObserver registration FAILED — pool full (increase CHIP_CONFIG_ICD_OBSERVERS_POOL_SIZE)");
    } else {
        log_i("MatterCustom: ICDObserver registered");
    }

    return true;
}

// ── Status queries ───────────────────────────────────────────────────────────
bool MatterInit::isCommissioned() {
    return chip::Server::GetInstance().GetFabricTable().FabricCount() > 0;
}

bool MatterInit::isConnected() {
    return chip::DeviceLayer::ConnectivityMgr().IsThreadAttached();
}

bool MatterInit::isCommissioningWindowOpen() {
    return chip::Server::GetInstance().GetCommissioningWindowManager()
               .IsCommissioningWindowOpen();
}

void MatterInit::decommission() {
    esp_matter::factory_reset();
}

// ── Battery update ───────────────────────────────────────────────────────────
bool MatterInit::setBatteryPercent(uint8_t percent) {
    if (!_started) return false;
    if (percent > 100) percent = 100;

    // BatteryPercentageRemaining attribute is 0–200 (half-percent resolution).
    uint8_t raw = percent * 2;

    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    constexpr uint32_t kPowerSourceCluster = PowerSource::Id;
    constexpr uint32_t kBatPercentAttr     = PowerSource::Attributes::BatPercentRemaining::Id;

    endpoint_t *root_ep = endpoint::get(_node, 0);
    if (!root_ep) return false;

    cluster_t *ps_cluster = cluster::get(root_ep, kPowerSourceCluster);
    if (!ps_cluster) return false;

    attribute_t *attr = attribute::get(ps_cluster, kBatPercentAttr);
    if (!attr) return false;

    attribute::get_val(attr, &val);
    val.val.u8 = raw;
    if (attribute::update(0, kPowerSourceCluster, kBatPercentAttr, &val) != ESP_OK) return false;

    // Also update BatChargeLevel so Apple Home shows the correct battery indicator.
    constexpr uint32_t kBatChargeLevelAttr = PowerSource::Attributes::BatChargeLevel::Id;
    uint8_t chargeLevel = (percent > 33)
        ? static_cast<uint8_t>(PowerSource::BatChargeLevelEnum::kOk)
        : (percent > 10)
            ? static_cast<uint8_t>(PowerSource::BatChargeLevelEnum::kWarning)
            : static_cast<uint8_t>(PowerSource::BatChargeLevelEnum::kCritical);
    attribute_t *lvlAttr = attribute::get(ps_cluster, kBatChargeLevelAttr);
    if (lvlAttr) {
        esp_matter_attr_val_t lvlVal = esp_matter_invalid(NULL);
        attribute::get_val(lvlAttr, &lvlVal);
        lvlVal.val.u8 = chargeLevel;
        attribute::update(0, kPowerSourceCluster, kBatChargeLevelAttr, &lvlVal);
    }
    return true;
}

// ── Battery charge state update ──────────────────────────────────────────────
bool MatterInit::setBatteryChargeState(bool isCharging) {
    if (!_started) return false;
    constexpr uint32_t kPowerSourceCluster = PowerSource::Id;
    constexpr uint32_t kBatChargeStateAttr = PowerSource::Attributes::BatChargeState::Id;
    endpoint_t *root_ep = endpoint::get(_node, 0);
    if (!root_ep) return false;
    cluster_t *ps_cluster = cluster::get(root_ep, kPowerSourceCluster);
    if (!ps_cluster) return false;
    attribute_t *attr = attribute::get(ps_cluster, kBatChargeStateAttr);
    if (!attr) return false;
    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    attribute::get_val(attr, &val);
    val.val.u8 = static_cast<uint8_t>(isCharging
        ? PowerSource::BatChargeStateEnum::kIsCharging
        : PowerSource::BatChargeStateEnum::kIsNotCharging);
    return attribute::update(0, kPowerSourceCluster, kBatChargeStateAttr, &val) == ESP_OK;
}

// ── Battery voltage update ───────────────────────────────────────────────────
bool MatterInit::setBatteryVoltage(uint32_t mv) {
    if (!_started) return false;
    constexpr uint32_t kPowerSourceCluster = PowerSource::Id;
    constexpr uint32_t kBatVoltageAttr     = PowerSource::Attributes::BatVoltage::Id;
    endpoint_t *root_ep = endpoint::get(_node, 0);
    if (!root_ep) return false;
    cluster_t *ps_cluster = cluster::get(root_ep, kPowerSourceCluster);
    if (!ps_cluster) return false;
    attribute_t *attr = attribute::get(ps_cluster, kBatVoltageAttr);
    if (!attr) return false;
    nullable<uint32_t> voltageVal(mv);
    esp_matter_attr_val_t val = esp_matter_nullable_uint32(voltageVal);
    return attribute::update(0, kPowerSourceCluster, kBatVoltageAttr, &val) == ESP_OK;
}

// ── SubscriptionTracker callbacks ────────────────────────────────────────────
void MatterInit::SubscriptionTracker::OnSubscriptionEstablished(chip::app::ReadHandler &) {
    _count.fetch_add(1);
    log_i("MatterCustom: subscription established (active=%d)", _count.load());
}

void MatterInit::SubscriptionTracker::OnSubscriptionTerminated(chip::app::ReadHandler &) {
    if (_count.load() > 0) _count.fetch_sub(1);
    log_i("MatterCustom: subscription terminated (active=%d)", _count.load());
}

// ── _checkDirtyWork ──────────────────────────────────────────────────────────
// Runs on the CHIP task thread (via PlatformMgr().ScheduleWork) to safely read
// GetNumDirtySubscriptions() without a data race.
void MatterInit::_checkDirtyWork(intptr_t) {
    _pendingDirtyReportCount.store(
        (uint32_t)chip::app::InteractionModelEngine::GetInstance()
                      ->GetNumDirtySubscriptions()
    );
}

// ── waitForSubscription() ────────────────────────────────────────────────────
// Phase 1+2: Wait for Thread connectivity and at least one active subscription.
// Must be called BEFORE updating attributes so no failed CASE session attempts
// are triggered while Thread is still joining.
bool MatterInit::waitForSubscription(uint32_t timeoutMs) {
    uint32_t start = millis();

    // Phase 1: Wait for Thread connectivity.
    while (!isConnected() && (millis() - start < timeoutMs)) { delay(200); }
    if (!isConnected()) {
        log_w("MatterCustom: Thread not up after %ums", timeoutMs);
        return false;
    }
    log_i("MatterCustom: Thread connected (%.1fs)", (millis() - start) / 1000.0f);

    // Phase 2: Wait for at least one subscription to be (re-)established.
    // With LIT ICD + CIP the ICD server sends a Check-In after Thread joins;
    // Apple Home responds and re-subscribes — the callback fires within seconds.
    uint32_t remaining = timeoutMs - (millis() - start);
    uint32_t subStart = millis();
    while (!_subscriptionTracker.hasActiveSubscription() && (millis() - subStart < remaining)) {
        delay(200);
    }
    if (!_subscriptionTracker.hasActiveSubscription()) {
        log_w("MatterCustom: no active subscriptions within timeout — controller not subscribed yet");
        return false;
    }
    log_i("MatterCustom: subscription active (%.1fs)", (millis() - start) / 1000.0f);
    return true;
}

// ── waitForDirtyDrain() ──────────────────────────────────────────────────────
// Phase 3: Poll GetNumDirtySubscriptions() on the CHIP thread until all
// pending attribute reports have been sent. Call AFTER attribute setters.
bool MatterInit::waitForDirtyDrain(uint32_t drainMs) {
    uint32_t drainStart = millis();
    while (millis() - drainStart < drainMs) {
        chip::DeviceLayer::PlatformMgr().ScheduleWork(_checkDirtyWork, 0);
        delay(150);
        if (_pendingDirtyReportCount.load() == 0) break;
    }
    bool ok = (_pendingDirtyReportCount.load() == 0);
    log_i("MatterCustom: drain %s", ok ? "complete" : "timeout — sleeping anyway");
    return ok;
}

// ── waitForReportsDelivered() ────────────────────────────────────────────────
// Convenience wrapper: waitForSubscription + waitForDirtyDrain.
// Use the split API (waitForSubscription / waitForDirtyDrain) directly when
// attributes should only be updated after subscription is confirmed.
bool MatterInit::waitForReportsDelivered(uint32_t timeoutMs) {
    if (!waitForSubscription(timeoutMs)) return false;
    return waitForDirtyDrain(5000);
}

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
