#pragma once
#include <string>
#include <cstdint>

namespace smartcart::domain {

// Статус модуля (физическое состояние на шине)
enum class ModuleStatus : std::uint8_t {
    Online  = 0,
    Offline = 1,
    Maint   = 2,
};

inline std::string_view toString(ModuleStatus s) {
    switch (s) {
        case ModuleStatus::Online:  return "ONLINE";
        case ModuleStatus::Offline: return "OFFLINE";
        case ModuleStatus::Maint:   return "MAINT";
    }
    return "UNKNOWN";
}

inline ModuleStatus moduleStatusFromString(std::string_view s) {
    if (s == "ONLINE")  return ModuleStatus::Online;
    if (s == "OFFLINE") return ModuleStatus::Offline;
    if (s == "MAINT")   return ModuleStatus::Maint;
    return ModuleStatus::Offline;
}

// Основная доменная сущность модуля (тележки/лотка)
struct ModuleInfo {
    int         id{0};
    std::string serial;
    int         slotCount{24};
    std::string firmware;
    ModuleStatus status{ModuleStatus::Offline};
};

} // namespace smartcart::domain
