// ===== src/infrastructure/config/ConfigLoader.cpp =====
// Исправлено: убраны backward-compat поля
#include "ConfigLoader.hpp"
#include <fstream>
#include <stdexcept>
#include <utility>
#include <nlohmann/json.hpp>

namespace smartcart::infrastructure::config {
namespace {
using json = nlohmann::json;

template <typename T>
void readOptional(const json& j, const char* key, T& out) {
    if (j.contains(key) && !j.at(key).is_null())
        out = j.at(key).get<T>();
}

std::vector<ModuleChannelConfig> readModuleChannels(const json& j) {
    std::vector<ModuleChannelConfig> out;
    if (!j.contains("module_channels") || j.at("module_channels").is_null()) {
        return out;
    }
    if (!j.at("module_channels").is_array()) {
        throw std::runtime_error("ConfigLoader: module_channels must be an array");
    }

    for (const auto& item : j.at("module_channels")) {
        ModuleChannelConfig channel;
        readOptional(item, "enabled", channel.enabled);
        readOptional(item, "name", channel.name);
        readOptional(item, "rfid_uid", channel.rfidUid);
        readOptional(item, "kind", channel.kind);
        readOptional(item, "stm32_device", channel.stm32Device);
        readOptional(item, "slot_count", channel.slotCount);
        readOptional(item, "switch_tracked_channels", channel.trackedChannels);
        readOptional(item, "switch_ignored_channels", channel.ignoredChannels);
        readOptional(item, "switch_channel_to_slot_map", channel.channelToSlotMap);
        out.push_back(std::move(channel));
    }

    return out;
}
} // namespace

AppConfig ConfigLoader::loadFromFile(const std::string& path) {
    std::ifstream in(path);
    if (!in.is_open())
        throw std::runtime_error("ConfigLoader: cannot open file: " + path);

    json j;
    in >> j;

    AppConfig cfg{};
    readOptional(j, "log_file",              cfg.logFile);
    readOptional(j, "sqlite_path",           cfg.sqlitePath);
    readOptional(j, "completed_orders_dir",  cfg.completedOrdersDir);
    readOptional(j, "stm32_device",          cfg.stm32Device);
    readOptional(j, "rfid_spi_device",     cfg.rfidSpiDevice);
    readOptional(j, "rfid_spi_devices",    cfg.rfidSpiDevices);
    readOptional(j, "rfid_module_roles",   cfg.rfidModuleKinds);
    cfg.moduleChannels = readModuleChannels(j);
    readOptional(j, "demo_mode",           cfg.demoMode);
    readOptional(j, "rfid_enabled",        cfg.rfidEnabled);
    readOptional(j, "stm32_poll_ms",       cfg.stm32PollMs);
    readOptional(j, "debounce_ms",         cfg.debounceMs);
    readOptional(j, "stable_confirm_ms",   cfg.stableConfirmMs);
    readOptional(j, "rfid_read_timeout_ms", cfg.rfidReadTimeoutMs);
    readOptional(j, "rfid_poll_ms",        cfg.rfidPollMs);
    readOptional(j, "rfid_offline_timeout_ms", cfg.rfidOfflineTimeoutMs);
    readOptional(j, "led_mapping_path",    cfg.ledMappingPath);
    readOptional(j, "module_profile_path", cfg.moduleProfilePath);
    readOptional(j, "slot_to_led_map",     cfg.slotToLedMap);
    readOptional(j, "switch_tracked_channels", cfg.switchTrackedChannels);
    readOptional(j, "switch_ignored_channels", cfg.switchIgnoredChannels);
    readOptional(j, "switch_channel_to_slot_map", cfg.switchChannelToSlotMap);

    validate(cfg);
    return cfg;
}

void ConfigLoader::validate(AppConfig& cfg) {
    if (cfg.logFile.empty())
        throw std::runtime_error("ConfigLoader: log_file is empty");
    if (cfg.sqlitePath.empty())
        throw std::runtime_error("ConfigLoader: sqlite_path is empty");
    if (!cfg.demoMode && cfg.stm32Device.empty())
        throw std::runtime_error("ConfigLoader: stm32_device is empty");
    if (cfg.rfidEnabled && cfg.rfidSpiDevice.empty() && cfg.rfidSpiDevices.empty())
        throw std::runtime_error(
            "ConfigLoader: rfid_spi_device or rfid_spi_devices is required");
    if (cfg.rfidSpiDevices.empty() && !cfg.rfidSpiDevice.empty()) {
        cfg.rfidSpiDevices.push_back(cfg.rfidSpiDevice);
    }
    for (const auto& device : cfg.rfidSpiDevices) {
        if (device.empty())
            throw std::runtime_error(
                "ConfigLoader: rfid_spi_devices must not contain empty values");
    }
    for (const auto& [uid, kind] : cfg.rfidModuleKinds) {
        if (uid.empty()) {
            throw std::runtime_error(
                "ConfigLoader: rfid_module_roles contains empty uid");
        }
        if (kind != "REEL" && kind != "reel" &&
            kind != "FEEDER" && kind != "feeder" &&
            kind != "UNKNOWN" && kind != "unknown")
        {
            throw std::runtime_error(
                "ConfigLoader: rfid_module_roles values must be REEL/FEEDER/UNKNOWN");
        }
    }
    for (auto& channel : cfg.moduleChannels) {
        if (!channel.enabled) {
            continue;
        }
        if (channel.name.empty()) {
            channel.name = !channel.rfidUid.empty()
                ? channel.rfidUid
                : channel.stm32Device;
        }
        if (channel.name.empty()) {
            throw std::runtime_error(
                "ConfigLoader: enabled module channel must have name/rfid_uid/stm32_device");
        }
        if (channel.rfidUid.empty()) {
            throw std::runtime_error(
                "ConfigLoader: enabled module channel must have rfid_uid");
        }
        if (channel.stm32Device.empty()) {
            throw std::runtime_error(
                "ConfigLoader: enabled module channel must have stm32_device");
        }
        if (channel.kind != "REEL" && channel.kind != "reel" &&
            channel.kind != "FEEDER" && channel.kind != "feeder" &&
            channel.kind != "UNKNOWN" && channel.kind != "unknown")
        {
            throw std::runtime_error(
                "ConfigLoader: module_channels.kind must be REEL/FEEDER/UNKNOWN");
        }
        if (channel.slotCount <= 0 || channel.slotCount > 24) {
            throw std::runtime_error(
                "ConfigLoader: module_channels.slot_count must be 1..24");
        }
        if (channel.trackedChannels.empty()) {
            channel.trackedChannels = cfg.switchTrackedChannels;
        }
        if (channel.ignoredChannels.empty()) {
            channel.ignoredChannels = cfg.switchIgnoredChannels;
        }
        if (channel.channelToSlotMap.empty()) {
            channel.channelToSlotMap = cfg.switchChannelToSlotMap;
        }
        for (const int channelIndex : channel.trackedChannels) {
            if (channelIndex < 0 || channelIndex >= channel.slotCount) {
                throw std::runtime_error(
                    "ConfigLoader: module_channels.switch_tracked_channels values must fit slot_count");
            }
        }
        for (const int channelIndex : channel.ignoredChannels) {
            if (channelIndex < 0 || channelIndex >= channel.slotCount) {
                throw std::runtime_error(
                    "ConfigLoader: module_channels.switch_ignored_channels values must fit slot_count");
            }
        }
        if (!channel.channelToSlotMap.empty() &&
            channel.channelToSlotMap.size() != 24)
        {
            throw std::runtime_error(
                "ConfigLoader: module_channels.switch_channel_to_slot_map must contain 24 entries");
        }
    }
    if (cfg.stm32PollMs == 0)
        throw std::runtime_error("ConfigLoader: stm32_poll_ms must be > 0");
    if (cfg.debounceMs == 0)
        throw std::runtime_error("ConfigLoader: debounce_ms must be > 0");
    if (cfg.stableConfirmMs < cfg.debounceMs)
        throw std::runtime_error(
            "ConfigLoader: stable_confirm_ms must be >= debounce_ms");
    if (cfg.rfidReadTimeoutMs == 0)
        throw std::runtime_error(
            "ConfigLoader: rfid_read_timeout_ms must be > 0");
    if (cfg.rfidPollMs == 0)
        throw std::runtime_error(
            "ConfigLoader: rfid_poll_ms must be > 0");
    if (cfg.rfidOfflineTimeoutMs < cfg.rfidPollMs)
        throw std::runtime_error(
            "ConfigLoader: rfid_offline_timeout_ms must be >= rfid_poll_ms");
    if (!cfg.slotToLedMap.empty() && cfg.slotToLedMap.size() != 24)
        throw std::runtime_error(
            "ConfigLoader: slot_to_led_map must contain 24 entries");
    if (cfg.switchTrackedChannels.empty())
        throw std::runtime_error(
            "ConfigLoader: switch_tracked_channels must not be empty");
    for (const int channel : cfg.switchTrackedChannels) {
        if (channel < 0 || channel >= 24)
            throw std::runtime_error(
                "ConfigLoader: switch_tracked_channels values must be 0..23");
    }
    for (const int channel : cfg.switchIgnoredChannels) {
        if (channel < 0 || channel >= 24)
            throw std::runtime_error(
                "ConfigLoader: switch_ignored_channels values must be 0..23");
    }
    if (!cfg.switchChannelToSlotMap.empty() &&
        cfg.switchChannelToSlotMap.size() != 24)
    {
        throw std::runtime_error(
            "ConfigLoader: switch_channel_to_slot_map must contain 24 entries");
    }
    for (const int slotIndex : cfg.switchChannelToSlotMap) {
        if (slotIndex < 0 || slotIndex > 24)
            throw std::runtime_error(
                "ConfigLoader: switch_channel_to_slot_map values must be 0..24");
    }
}

} // namespace smartcart::infrastructure::config
