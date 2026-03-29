#include <iostream>
#include "../../src/infrastructure/config/ConfigLoader.hpp"
#include "../../src/infrastructure/logging/LoggerFactory.hpp"
#include "../../src/infrastructure/db/SqliteConnection.hpp"


int main() {
    try {
        namespace cfg_ns = smartcart::infrastructure::config;
        namespace log_ns = smartcart::infrastructure::logging;
        namespace db_ns  = smartcart::infrastructure::db;

        auto cfg = cfg_ns::ConfigLoader::loadFromFile("config/appsettings.json");

        // 👇 ВАЖНО: применяем миграции при старте
        db_ns::SqliteConnection conn(cfg.sqlitePath.empty() ? cfg.sqliteDbPath : cfg.sqlitePath);
        conn.runMigrations("src/infrastructure/db/migrations");

        auto logger = log_ns::LoggerFactory::createFileLogger(cfg);
        logger->info("Startup complete, demo_mode={}", cfg.demoMode);

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Fatal startup error: " << e.what() << std::endl;
        return 1;
    }
}
