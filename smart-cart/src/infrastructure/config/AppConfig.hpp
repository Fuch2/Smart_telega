#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace smartcart::infrastructure::config {

struct AppConfig {
    std::string logFile;
    std::string sqlitePath;

    std::string logFilePath;
    std::string sqliteDbPath;

    bool demoMode{true};

    std::uint32_t stm32PollMs{500};
    std::uint32_t debounceMs{50};
    std::uint32_t stableConfirmMs{1000};

    // baseline config files
    std::string ledMappingPath{"config/led_mapping.default.json"};
    std::string moduleProfilePath{"config/module_profiles/tray24.json"};

    // fallback map из appsettings (может быть пустым, если используется ledMappingPath)
    std::vector<int> slotToLedMap;
};

} // namespace smartcart::infrastructure::config
