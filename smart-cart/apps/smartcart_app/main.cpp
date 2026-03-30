// Headless smoke-test: открывает БД и применяет миграции без UI.
// Используется в CI для проверки схемы.
#include <iostream>
#include "../../src/infrastructure/config/ConfigLoader.hpp"
#include "../../src/infrastructure/db/SqliteConnection.hpp"

int main() {
    try {
        namespace cfg = smartcart::infrastructure::config;
        namespace db  = smartcart::infrastructure::db;

        auto config = cfg::ConfigLoader::loadFromFile(CONFIG_DIR "/appsettings.json");
        db::SqliteConnection conn(config.sqlitePath);
        conn.runMigrations(MIGRATIONS_DIR);

        std::cout << "DB OK: " << config.sqlitePath << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << std::endl;
        return 1;
    }
}
