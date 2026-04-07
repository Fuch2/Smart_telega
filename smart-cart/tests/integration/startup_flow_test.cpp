// ===== tests/integration/startup_flow_test.cpp =====
#include "infrastructure/hw/stm32/MockStm32Link.hpp"
#include "infrastructure/db/SqliteConnection.hpp"
#include "infrastructure/db/repositories/ReelRepositorySqlite.hpp"
#include "infrastructure/db/repositories/ModuleRepositorySqlite.hpp"
#include "application/services/StartupService.hpp"
#include "domain/entities/Slot.hpp"

#include <gtest/gtest.h>
#include <variant>
#include <vector>

using namespace smartcart;
using namespace smartcart::infrastructure::hw::stm32;
using namespace smartcart::application::services;

class StartupFlowTest : public ::testing::Test {
protected:
    void SetUp() override {
        conn_       = std::make_unique<infrastructure::db::SqliteConnection>(":memory:");
        moduleRepo_ = std::make_unique<infrastructure::db::ModuleRepositorySqlite>(*conn_);
        reelRepo_   = std::make_unique<infrastructure::db::ReelRepositorySqlite>(*conn_);

        // FK: slot_states.module_id → modules.id
        conn_->execute(
            "INSERT INTO modules(id,serial,slot_count,firmware,status)"
            " VALUES(1,'TEST-MODULE',24,'','ONLINE');"
        );

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
            resp.payload = {0x00, 0x00, 0x00};
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
        return std::nullopt;
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
    MockStm32Link link([](const Frame& cmd) -> std::optional<Frame> {
        Frame resp;
        resp.seq   = cmd.seq;
        resp.cmdId = cmd.cmdId;
        if (cmd.cmdId == CommandId::GetSwitchSnapshot) {
            resp.type    = FrameType::Resp;
            resp.payload = {0x01, 0x00, 0x00};
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

    EXPECT_EQ(slots[0].state, domain::SlotState::Error);
    for (int i = 1; i < static_cast<int>(slots.size()); ++i)
        EXPECT_EQ(slots[i].state, domain::SlotState::Free);
}

TEST_F(StartupFlowTest, ChannelMappingRoutesPa1ToFirstSlot) {
    MockStm32Link link([](const Frame& cmd) -> std::optional<Frame> {
        Frame resp;
        resp.seq   = cmd.seq;
        resp.cmdId = cmd.cmdId;
        if (cmd.cmdId == CommandId::GetSwitchSnapshot) {
            resp.type    = FrameType::Resp;
            resp.payload = {0x02, 0x00, 0x00}; // channel 1 active
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
    cfg.channelToSlotMap = {
        3, 1, 4, 2, 5, 6, 7, 8,
        9, 10, 11, 0, 12, 13, 14, 15,
        16, 17, 18, 19, 20, 21, 22, 23
    };

    StartupService svc(link, *reelRepo_, *moduleRepo_, cfg);
    const auto result = svc.run();

    ASSERT_TRUE(std::holds_alternative<std::vector<domain::Slot>>(result));
    const auto& slots = std::get<std::vector<domain::Slot>>(result);

    EXPECT_EQ(slots[0].slotIndex, 1);
    EXPECT_EQ(slots[0].state, domain::SlotState::Error);
    EXPECT_EQ(slots[1].slotIndex, 2);
    EXPECT_EQ(slots[1].state, domain::SlotState::Free);
}
