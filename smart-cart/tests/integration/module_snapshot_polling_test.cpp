#include "application/services/ModuleSnapshotPollingService.hpp"
#include "domain/entities/Slot.hpp"
#include "infrastructure/db/SqliteConnection.hpp"
#include "infrastructure/db/repositories/ModuleRepositorySqlite.hpp"
#include "infrastructure/db/repositories/ReelRepositorySqlite.hpp"
#include "infrastructure/hw/stm32/MockStm32Link.hpp"
#include "infrastructure/logging/SqliteEventLogger.hpp"

#include <gtest/gtest.h>

using namespace smartcart;
using namespace smartcart::application::services;
using namespace smartcart::infrastructure::hw::stm32;

TEST(ModuleSnapshotPollingServiceTest, PollOnceWritesModuleScopedSlotStates) {
    infrastructure::db::SqliteConnection conn(":memory:");
    conn.runMigrations(std::string{MIGRATIONS_DIR});

    infrastructure::db::ModuleRepositorySqlite moduleRepo(conn);
    infrastructure::db::ReelRepositorySqlite reelRepo(conn);
    infrastructure::logging::SqliteEventLogger eventLogger(conn.handle());

    domain::ModuleInfo module;
    module.serial = "RFID-FEEDER-TEST";
    module.slotCount = 24;
    module.firmware = "test";
    module.status = domain::ModuleStatus::Offline;
    module.kind = domain::ModuleKind::Feeder;
    const int moduleId = moduleRepo.add(module);
    ASSERT_GT(moduleId, 0);

    MockStm32Link link([](const Frame& cmd) -> std::optional<Frame> {
        Frame resp;
        resp.seq = cmd.seq;
        resp.cmdId = cmd.cmdId;
        if (cmd.cmdId == CommandId::GetSwitchSnapshot) {
            resp.type = FrameType::Resp;
            resp.payload = {0x02, 0x00, 0x00};
            return resp;
        }
        resp.type = FrameType::Ack;
        return resp;
    });
    ASSERT_TRUE(link.open());

    ModuleSnapshotPollingConfig cfg;
    cfg.moduleId = moduleId;
    cfg.channelName = "feeder-test";
    cfg.stm32Device = "mock";
    cfg.trackedChannels = {1};

    ModuleSnapshotPollingService svc(
        link, moduleRepo, reelRepo, eventLogger, cfg);
    svc.pollOnce();

    const auto slots = reelRepo.getSlotStates(moduleId);
    ASSERT_FALSE(slots.empty());
    ASSERT_EQ(slots[0].slotIndex, 2);
    EXPECT_EQ(slots[0].state, domain::SlotState::Occupied);

    const auto updatedModule = moduleRepo.getById(moduleId);
    ASSERT_TRUE(updatedModule.has_value());
    EXPECT_EQ(updatedModule->status, domain::ModuleStatus::Online);
}
