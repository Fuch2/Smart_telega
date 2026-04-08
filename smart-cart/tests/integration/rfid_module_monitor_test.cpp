#include "application/services/RfidModuleMonitorService.hpp"
#include "infrastructure/db/SqliteConnection.hpp"
#include "infrastructure/db/repositories/ModuleRepositorySqlite.hpp"
#include "infrastructure/hw/rfid/MockRfidProvider.hpp"
#include "infrastructure/logging/SqliteEventLogger.hpp"
#include "domain/entities/ModuleInfo.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

using namespace smartcart;

TEST(RfidModuleMonitorTest, OnlineModuleBecomesOfflineWhenTagDisappears) {
    infrastructure::db::SqliteConnection conn(":memory:");
    conn.runMigrations(std::string{MIGRATIONS_DIR});

    infrastructure::db::ModuleRepositorySqlite moduleRepo(conn);
    infrastructure::logging::SqliteEventLogger eventLogger(conn.handle());
    infrastructure::hw::rfid::MockRfidProvider rfidProvider;

    domain::ModuleInfo module;
    module.serial = "RFID-TEST";
    module.slotCount = 24;
    module.firmware = "test";
    module.status = domain::ModuleStatus::Online;
    const int moduleId = moduleRepo.add(module);
    ASSERT_GT(moduleId, 0);

    application::services::RfidModuleMonitorConfig cfg;
    cfg.moduleId = moduleId;
    cfg.pollMs = 10;
    cfg.readTimeoutMs = 5;
    cfg.offlineTimeoutMs = 40;
    cfg.expectedUid = "TEST";

    application::services::RfidModuleMonitorService svc(
        rfidProvider, moduleRepo, eventLogger, cfg);

    svc.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    svc.stop();

    const auto updated = moduleRepo.getById(moduleId);
    ASSERT_TRUE(updated.has_value());
    EXPECT_EQ(updated->status, domain::ModuleStatus::Offline);
}

TEST(RfidModuleMonitorTest, MatchingTagReturnsOfflineModuleToOnline) {
    infrastructure::db::SqliteConnection conn(":memory:");
    conn.runMigrations(std::string{MIGRATIONS_DIR});

    infrastructure::db::ModuleRepositorySqlite moduleRepo(conn);
    infrastructure::logging::SqliteEventLogger eventLogger(conn.handle());
    infrastructure::hw::rfid::MockRfidProvider rfidProvider;

    domain::ModuleInfo module;
    module.serial = "RFID-TEST";
    module.slotCount = 24;
    module.firmware = "test";
    module.status = domain::ModuleStatus::Offline;
    const int moduleId = moduleRepo.add(module);
    ASSERT_GT(moduleId, 0);

    application::services::RfidModuleMonitorConfig cfg;
    cfg.moduleId = moduleId;
    cfg.pollMs = 10;
    cfg.readTimeoutMs = 20;
    cfg.offlineTimeoutMs = 250;
    cfg.expectedUid = "TEST";

    application::services::RfidModuleMonitorService svc(
        rfidProvider, moduleRepo, eventLogger, cfg);

    svc.start();
    rfidProvider.inject("TEST");
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    svc.stop();

    const auto updated = moduleRepo.getById(moduleId);
    ASSERT_TRUE(updated.has_value());
    EXPECT_EQ(updated->status, domain::ModuleStatus::Online);
}
