// ===== src/infrastructure/config/ConfigLoader.cpp =====
// Исправлено: убраны backward-compat поля
#include "ConfigLoader.hpp"
#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>

namespace smartcart::infrastructure::config {
namespace {
using json = nlohmann::json;

template <typename T>
void readOptional(const json& j, const char* key, T& out) {
    if (j.contains(key) && !j.at(key).is_null())
        out = j.at(key).get<T>();
}
} // namespace

AppConfig ConfigLoader::loadFromFile(const std::string& path) {
    std::ifstream in(path);
    if (!in.is_open())
        throw std::runtime_error("ConfigLoader: cannot open file: " + path);

    json j;
    in >> j;

    AppConfig cfg{};
    readOptional(j, "log_file",            cfg.logFile);
    readOptional(j, "sqlite_path",         cfg.sqlitePath);
    readOptional(j, "stm32_device",        cfg.stm32Device);
    readOptional(j, "demo_mode",           cfg.demoMode);
    readOptional(j, "stm32_poll_ms",       cfg.stm32PollMs);
    readOptional(j, "debounce_ms",         cfg.debounceMs);
    readOptional(j, "stable_confirm_ms",   cfg.stableConfirmMs);
    readOptional(j, "led_mapping_path",    cfg.ledMappingPath);
    readOptional(j, "module_profile_path", cfg.moduleProfilePath);
    readOptional(j, "slot_to_led_map",     cfg.slotToLedMap);

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
    if (cfg.stm32PollMs == 0)
        throw std::runtime_error("ConfigLoader: stm32_poll_ms must be > 0");
    if (cfg.debounceMs == 0)
        throw std::runtime_error("ConfigLoader: debounce_ms must be > 0");
    if (cfg.stableConfirmMs < cfg.debounceMs)
        throw std::runtime_error(
            "ConfigLoader: stable_confirm_ms must be >= debounce_ms");
    if (!cfg.slotToLedMap.empty() && cfg.slotToLedMap.size() != 24)
        throw std::runtime_error(
            "ConfigLoader: slot_to_led_map must contain 24 entries");
}

} // namespace smartcart::infrastructure::config
