// ===== tests/integration/startup_flow_test.cpp =====
// Исправлено:
//   - комментарии на английском
//   - добавлен тест для случая когда физически занят, но БД пуста → Error state
#include "infrastructure/hw/stm32/MockStm32Link.hpp"
#include "infrastructure/db/SqliteConnection.hpp"
#include "infrastructure/db/repositories/ReelRepositorySqlite.hpp"
#include "infrastructure/db/repositories/ModuleRepositorySqlite.hpp"
#include "application/services/StartupService.hpp"
#include "domain/entities/Slot.hpp"

#include <gtest/gtest.h>
#include <variant>

using namespace smartcart;
using namespace smartcart::infrastructure::hw::stm32;
using namespace smartcart::application::services;

class StartupFlowTest : public ::testing::Test {
protected:
    void SetUp() override {
        conn_       = std::make_unique<infrastructure::db::SqliteConnection>(":memory:");
        moduleRepo_ = std::make_unique<infrastructure::db::ModuleRepositorySqlite>(*conn_);
        reelRepo_   = std::make_unique<infrastructure::db::ReelRepositorySqlite>(*conn_);

        // All 24 slots free initially
        for (int i = 1; i <= 24; ++i)
            reelRepo_->setSlotState(1, i, domain::SlotState::Free);
    }

    std::unique_ptr<infrastructure::db::SqliteConnection>       conn_;
    std::unique_ptr<infrastructure::db::ModuleRepositorySqlite> moduleRepo_;
    std::unique_ptr<infrastructure::db::ReelRepositorySqlite>   reelRepo_;
};

TEST_F(StartupFlowTest, AllSlotsFree_ReturnsReadySlots) {
    MockStm32Link link([](const Frame& cmd) -> std::optional<Frame> {
        Frame resp;
        resp.seq   = cmd.seq;
        resp.cmdId = cmd.cmdId;
        if (cmd.cmdId == CommandId::GetSwitchSnapshot) {
            resp.type    = FrameType::Resp;
            resp.payload = {0x00, 0x00, 0x00}; // all 24 bits = 0 (free)
        } else if (cmd.cmdId == CommandId::GetReadyState) {
            resp.type    = FrameType::Resp;
            resp.payload = {0x01};
        } else {
            resp.type = FrameType::Ack;
        }
        return resp;
    });
    link.open();

    StartupConfig cfg;
    cfg.moduleId       = 1;
    cfg.slotCount      = 24;
    cfg.readyTimeoutMs = 1000;
    cfg.slotToLedMap   = {};

    StartupService svc(link, *reelRepo_, *moduleRepo_, cfg);
    const auto result = svc.run();

    ASSERT_TRUE(std::holds_alternative<std::vector<domain::Slot>>(result));
    const auto& slots = std::get<std::vector<domain::Slot>>(result);
    ASSERT_EQ(static_cast<int>(slots.size()), 24);

    for (const auto& s : slots)
        EXPECT_EQ(s.state, domain::SlotState::Free);
}

TEST_F(StartupFlowTest, PingFails_ReturnsError) {
    MockStm32Link link([](const Frame&) -> std::optional<Frame> {
        return std::nullopt; // no response
    });
    link.open();

    StartupConfig cfg;
    cfg.moduleId       = 1;
    cfg.slotCount      = 24;
    cfg.readyTimeoutMs = 100;

    StartupService svc(link, *reelRepo_, *moduleRepo_, cfg);
    const auto result = svc.run();

    ASSERT_TRUE(std::holds_alternative<domain::ErrorCode>(result));
    EXPECT_EQ(std::get<domain::ErrorCode>(result),
              domain::ErrorCode::Stm32CommunicationError);
}

TEST_F(StartupFlowTest, PhysOccupied_DbEmpty_SlotsInErrorState) {
    // Slot 1 is physically occupied but DB has no reel record → Error state
    MockStm32Link link([](const Frame& cmd) -> std::optional<Frame> {
        Frame resp;
        resp.seq   = cmd.seq;
        resp.cmdId = cmd.cmdId;
        if (cmd.cmdId == CommandId::GetSwitchSnapshot) {
            resp.type    = FrameType::Resp;
            resp.payload = {0x01, 0x00, 0x00}; // bit 0 = slot 1 occupied
        } else if (cmd.cmdId == CommandId::GetReadyState) {
            resp.type    = FrameType::Resp;
            resp.payload = {0x01};
        } else {
            resp.type = FrameType::Ack;
        }
        return resp;
    });
    link.open();

    StartupConfig cfg;
    cfg.moduleId       = 1;
    cfg.slotCount      = 24;
    cfg.readyTimeoutMs = 1000;
    cfg.slotToLedMap   = {};

    StartupService svc(link, *reelRepo_, *moduleRepo_, cfg);
    const auto result = svc.run();

    ASSERT_TRUE(std::holds_alternative<std::vector<domain::Slot>>(result));
    const auto& slots = std::get<std::vector<domain::Slot>>(result);

    // Slot 1 should be Error (physical=occupied, db=empty)
    EXPECT_EQ(slots[0].state, domain::SlotState::Error);
    // Remaining slots should be Free
    for (int i = 1; i < static_cast<int>(slots.size()); ++i)
        EXPECT_EQ(slots[i].state, domain::SlotState::Free);
}
