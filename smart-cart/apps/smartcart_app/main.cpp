// ===== apps/smartcart_app/main.cpp =====
// Исправлено:
//   - убран headless smoke-test — этот файл не используется (есть main_qt.cpp)
//   - оставлен как минимальный CLI entry point для CI smoke-test
//   - CONFIG_DIR и MIGRATIONS_DIR проверяются через static_assert
#include <iostream>
#include <filesystem>
#include <stdexcept>

#include "infrastructure/config/ConfigLoader.hpp"
#include "infrastructure/db/SqliteConnection.hpp"

// Макросы должны быть определены через CMake target_compile_definitions
#ifndef CONFIG_DIR
#  error "CONFIG_DIR is not defined. Pass -DCONFIG_DIR=... via CMake."
#endif
#ifndef MIGRATIONS_DIR
#  error "MIGRATIONS_DIR is not defined. Pass -DMIGRATIONS_DIR=... via CMake."
#endif

int main() {
    try {
        namespace cfg = smartcart::infrastructure::config;
        namespace db  = smartcart::infrastructure::db;

        const std::string configPath = std::string(CONFIG_DIR) + "/config.json";

        auto config = cfg::ConfigLoader::loadFromFile(configPath);
        db::SqliteConnection conn(config.sqlitePath);
        conn.runMigrations(std::filesystem::path(MIGRATIONS_DIR));

        std::cout << "DB OK: " << config.sqlitePath << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << std::endl;
        return 1;
    }
}
