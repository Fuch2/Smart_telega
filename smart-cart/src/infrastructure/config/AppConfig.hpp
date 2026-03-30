// ===== src/infrastructure/config/AppConfig.hpp =====
// Исправлено: убрано несуществующее поле logFilePath — используется logFile
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace smartcart::infrastructure::config {

struct AppConfig {
    std::string logFile;            // путь к лог-файлу
    std::string sqlitePath;         // путь к БД

    bool demoMode{true};

    std::uint32_t stm32PollMs{500};
    std::uint32_t debounceMs{50};
    std::uint32_t stableConfirmMs{1000};

    std::string ledMappingPath{"config/led_mapping.default.json"};
    std::string moduleProfilePath{"config/module_profiles/tray24.json"};

    std::vector<int> slotToLedMap;
};

} // namespace smartcart::infrastructure::config
