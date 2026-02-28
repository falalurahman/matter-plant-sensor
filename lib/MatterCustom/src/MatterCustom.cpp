// MatterCustom.cpp — Matter node lifecycle (no Matter.h dependency)

#include <sdkconfig.h>
#ifdef CONFIG_ESP_MATTER_ENABLE_DATA_MODEL

#include "MatterCustom.h"
#include <app/server/Server.h>
#include <platform/ConnectivityManager.h>

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace esp_matter::cluster;
using namespace esp_matter::identification;
using namespace chip::app::Clusters;

// ── Static member storage ────────────────────────────────────────────────────
bool MatterCustomNode::_initialized = false;
bool MatterCustomNode::_started     = false;
esp_matter::node_t *MatterCustomNode::_node = nullptr;

// ── Event callback ───────────────────────────────────────────────────────────
void MatterCustomNode::eventCB(const ChipDeviceEvent *event, intptr_t /*arg*/) {
    switch (event->Type) {
        case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
            log_i("Matter commissioning complete");
            break;
        case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
            log_i("Fabric removed");
            if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
                log_i("No fabrics left, re-opening commissioning window");
                chip::CommissioningWindowManager &mgr =
                    chip::Server::GetInstance().GetCommissioningWindowManager();
                constexpr auto kTimeout = chip::System::Clock::Seconds16(300);
                if (!mgr.IsCommissioningWindowOpen()) {
                    mgr.OpenBasicCommissioningWindow(
                        kTimeout, chip::CommissioningWindowAdvertisement::kDnssdOnly);
                }
            }
            break;
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
    return true;
}

// ── Status queries ───────────────────────────────────────────────────────────
bool MatterCustomNode::isCommissioned() {
    return chip::Server::GetInstance().GetFabricTable().FabricCount() > 0;
}

bool MatterCustomNode::isConnected() {
    return chip::DeviceLayer::ConnectivityMgr().IsThreadAttached();
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

#endif // CONFIG_ESP_MATTER_ENABLE_DATA_MODEL
