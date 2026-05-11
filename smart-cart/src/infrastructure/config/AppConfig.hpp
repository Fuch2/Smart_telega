// ===== src/infrastructure/config/AppConfig.hpp =====
// Исправлено: убрано несуществующее поле logFilePath — используется logFile
#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace smartcart::infrastructure::config {

struct ModuleChannelConfig {
    bool enabled{true};
    std::string name;
    std::string rfidUid;
    std::string kind{"UNKNOWN"};
    std::string stm32Device;
    int slotCount{24};
    std::vector<int> trackedChannels;
    std::vector<int> ignoredChannels;
    std::vector<int> channelToSlotMap;
};

struct AppConfig {
    std::string logFile;               // путь к лог-файлу
    std::string sqlitePath;            // путь к БД
    std::string completedOrdersDir;    // куда записывать JSON выполненных заказов (пусто = не писать)
    std::string ordersDir;             // директория с JSON/XLSX заказами для экрана выбора (пусто = только FileDialog)
    std::string batterySysfsPath;      // путь к файлу заряда батареи, напр. /sys/class/power_supply/BAT0/capacity
    std::string stm32Device{"/dev/ttyAMA0"};
    std::string rfidSpiDevice{"/dev/spidev0.0"};
    std::vector<std::string> rfidSpiDevices;
    std::unordered_map<std::string, std::string> rfidModuleKinds;
    std::vector<ModuleChannelConfig> moduleChannels;

    bool demoMode{true};
    bool rfidEnabled{false};

    std::uint32_t stm32PollMs{500};
    std::uint32_t debounceMs{50};
    std::uint32_t stableConfirmMs{1000};
    std::uint32_t rfidReadTimeoutMs{3000};
    std::uint32_t rfidPollMs{500};
    std::uint32_t rfidOfflineTimeoutMs{3000};

    std::string ledMappingPath{"config/led_mapping.default.json"};
    std::string moduleProfilePath{"config/module_profiles/tray24.json"};

    std::vector<int> slotToLedMap;
    // LED-индексы статусной RGB-подсветки тележки (ТЗ п.5.2).
    // Пустой список = подсветка отключена.
    std::vector<int> cartStatusLeds;
    std::vector<int> switchTrackedChannels{1, 3};
    std::vector<int> switchIgnoredChannels{11};
    std::vector<int> switchChannelToSlotMap;
};

} // namespace smartcart::infrastructure::config
