#include "application/services/RfidModuleMonitorService.hpp"
#include "infrastructure/db/SqliteConnection.hpp"
#include "infrastructure/db/repositories/ModuleRepositorySqlite.hpp"
#include "infrastructure/hw/rfid/MockRfidProvider.hpp"
#include "infrastructure/logging/SqliteEventLogger.hpp"
#include "domain/entities/ModuleInfo.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

using namespace smartcart;

namespace {

class MultiTagRfidProvider final
    : public application::ports::IRfidProvider
{
public:
    void setRfidCallback(RfidCallback cb) override { cb_ = std::move(cb); }
    void start() override { active_ = true; }
    void stop() override { active_ = false; }
    bool isActive() const override { return active_; }

    std::optional<std::string> readOnce(int) override {
        const auto all = readAllOnce(0);
        if (all.empty()) {
            return std::nullopt;
        }
        return all.front();
    }

    std::vector<std::string> readAllOnce(int) override {
        std::lock_guard lock(mtx_);
        return uids_;
    }

    void setVisibleUids(std::vector<std::string> uids) {
        std::lock_guard lock(mtx_);
        uids_ = std::move(uids);
    }

private:
    RfidCallback cb_;
    bool active_ = false;
    std::mutex mtx_;
    std::vector<std::string> uids_;
};

} // namespace

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

TEST(RfidModuleMonitorTest, EmptyExpectedUidRequestsSwitchWhenTagAppears) {
    infrastructure::db::SqliteConnection conn(":memory:");
    conn.runMigrations(std::string{MIGRATIONS_DIR});

    infrastructure::db::ModuleRepositorySqlite moduleRepo(conn);
    infrastructure::logging::SqliteEventLogger eventLogger(conn.handle());
    infrastructure::hw::rfid::MockRfidProvider rfidProvider;

    domain::ModuleInfo module;
    module.serial = "SMARTCART-MODULE-1";
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
    cfg.expectedUid = "";

    application::services::RfidModuleMonitorService svc(
        rfidProvider, moduleRepo, eventLogger, cfg);

    std::atomic<int> switchCalls{0};
    std::string switchedUid;
    svc.setModuleSwitchCallback([&](std::string uid) {
        switchedUid = std::move(uid);
        ++switchCalls;
    });

    svc.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    EXPECT_EQ(switchCalls.load(), 0);

    rfidProvider.inject("NEWUID");
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    svc.stop();

    EXPECT_GE(switchCalls.load(), 1);
    EXPECT_EQ(switchedUid, "NEWUID");
}

TEST(RfidModuleMonitorTest, ExpectedUidGoesOfflineEvenWhenOtherTagIsVisible) {
    infrastructure::db::SqliteConnection conn(":memory:");
    conn.runMigrations(std::string{MIGRATIONS_DIR});

    infrastructure::db::ModuleRepositorySqlite moduleRepo(conn);
    infrastructure::logging::SqliteEventLogger eventLogger(conn.handle());
    MultiTagRfidProvider rfidProvider;

    domain::ModuleInfo module;
    module.serial = "RFID-EXPECTED";
    module.slotCount = 24;
    module.firmware = "test";
    module.status = domain::ModuleStatus::Online;
    const int moduleId = moduleRepo.add(module);
    ASSERT_GT(moduleId, 0);

    application::services::RfidModuleMonitorConfig cfg;
    cfg.moduleId = moduleId;
    cfg.pollMs = 10;
    cfg.readTimeoutMs = 5;
    cfg.offlineTimeoutMs = 50;
    cfg.expectedUid = "EXPECTED";

    application::services::RfidModuleMonitorService svc(
        rfidProvider, moduleRepo, eventLogger, cfg);

    rfidProvider.setVisibleUids({"EXPECTED"});
    svc.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(40));

    rfidProvider.setVisibleUids({"OTHER"});
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    svc.stop();

    const auto updated = moduleRepo.getById(moduleId);
    ASSERT_TRUE(updated.has_value());
    EXPECT_EQ(updated->status, domain::ModuleStatus::Offline);
}
