// ===== tests/integration/add_reel_flow_test.cpp =====
#include "infrastructure/hw/stm32/MockStm32Link.hpp"
#include "infrastructure/db/SqliteConnection.hpp"
#include "infrastructure/db/repositories/ReelRepositorySqlite.hpp"
#include "infrastructure/db/repositories/OperationRepositorySqlite.hpp"
#include "infrastructure/logging/SqliteEventLogger.hpp"
#include "application/services/AddReelService.hpp"
#include "application/services/Stm32PollingService.hpp"
#include "domain/entities/Operation.hpp"

#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <thread>

using namespace smartcart;
using namespace smartcart::infrastructure::hw::stm32;
using namespace smartcart::application::services;

class AddReelFlowTest : public ::testing::Test {
protected:
    void SetUp() override {
        conn_     = std::make_unique<infrastructure::db::SqliteConnection>(":memory:");
        reelRepo_ = std::make_unique<infrastructure::db::ReelRepositorySqlite>(*conn_);
        opRepo_   = std::make_unique<infrastructure::db::OperationRepositorySqlite>(*conn_);

        // FK: slot_states.module_id → modules.id
        conn_->execute(
            "INSERT INTO modules(id,serial,slot_count,firmware,status)"
            " VALUES(1,'TEST-MODULE',24,'','ONLINE');"
        );

        for (int i = 1; i <= 24; ++i)
            reelRepo_->setSlotState(1, i, domain::SlotState::Free);
    }

    std::unique_ptr<infrastructure::db::SqliteConnection>          conn_;
    std::unique_ptr<infrastructure::db::ReelRepositorySqlite>      reelRepo_;
    std::unique_ptr<infrastructure::db::OperationRepositorySqlite> opRepo_;
};

TEST_F(AddReelFlowTest, SuccessfulAdd_CompletesOperation) {
    MockStm32Link link;
    link.open();

    AddReelConfig cfg;
    cfg.moduleId        = 1;
    cfg.slotCount       = 24;
    cfg.stableConfirmMs = 200;
    cfg.slotToLedMap    = {};

    AddReelService svc(link, *reelRepo_, *opRepo_, cfg);

    domain::OperationStatus finalStatus = domain::OperationStatus::InProgress;
    svc.setCompletionCallback([&](int, domain::OperationStatus s) {
        finalStatus = s;
    });

    const int opId = svc.start("BARCODE-001");
    ASSERT_GT(opId, 0);

    std::thread([&link]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        Frame evt;
        evt.type    = FrameType::Evt;
        evt.cmdId   = CommandId::EvtSwitchChanged;
        evt.payload = {0x00, 0x01};
        link.injectEvent(evt);
    }).detach();

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (finalStatus == domain::OperationStatus::InProgress &&
           std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_EQ(finalStatus, domain::OperationStatus::Completed);
    EXPECT_TRUE(reelRepo_->hasActiveRecord(1, 1));
}

TEST_F(AddReelFlowTest, InvalidBarcode_ReturnsMinusOne) {
    MockStm32Link link;
    link.open();

    AddReelConfig cfg;
    cfg.moduleId     = 1;
    cfg.slotCount    = 24;
    cfg.slotToLedMap = {};

    AddReelService svc(link, *reelRepo_, *opRepo_, cfg);

    domain::ErrorCode gotCode = domain::ErrorCode::None;
    svc.setErrorCallback([&](domain::ErrorCode c, std::string) { gotCode = c; });

    EXPECT_EQ(svc.start(""), -1);
    EXPECT_EQ(gotCode, domain::ErrorCode::InvalidBarcode);
}

TEST_F(AddReelFlowTest, NoFreeSlots_ReturnsError) {
    MockStm32Link link;
    link.open();

    for (int i = 1; i <= 24; ++i)
        reelRepo_->setSlotState(1, i, domain::SlotState::Occupied);

    AddReelConfig cfg;
    cfg.moduleId     = 1;
    cfg.slotCount    = 24;
    cfg.slotToLedMap = {};

    AddReelService svc(link, *reelRepo_, *opRepo_, cfg);

    domain::ErrorCode gotCode = domain::ErrorCode::None;
    svc.setErrorCallback([&](domain::ErrorCode c, std::string) { gotCode = c; });

    EXPECT_EQ(svc.start("BARCODE-002"), -1);
    EXPECT_EQ(gotCode, domain::ErrorCode::NoFreeSlot);
}

TEST_F(AddReelFlowTest, PollingScanThenPa1_CreatesRecordAndCompletesOperation) {
    std::atomic<uint8_t> mask0{0x00};

    MockStm32Link link([&mask0](const Frame& cmd) -> std::optional<Frame> {
        Frame resp;
        resp.seq   = cmd.seq;
        resp.cmdId = cmd.cmdId;

        if (cmd.cmdId == CommandId::GetSwitchSnapshot ||
            static_cast<uint8_t>(cmd.cmdId) == 0x04)
        {
            resp.type    = FrameType::Resp;
            resp.payload = {mask0.load(), 0x00, 0x00};
        } else {
            resp.type = FrameType::Ack;
        }
        return resp;
    });
    link.open();

    infrastructure::logging::SqliteEventLogger logger(conn_->handle());

    Stm32PollingConfig cfg;
    cfg.moduleId = 1;
    cfg.slotCount = 24;
    cfg.pollMs = 10;
    cfg.trackedChannels = {1, 3};
    cfg.ignoredChannels = {11};

    Stm32PollingService svc(link, *reelRepo_, *opRepo_, logger, cfg);
    const auto opId = svc.recordBarcodeScan("REEL-PA1");
    ASSERT_TRUE(opId.has_value());

    mask0.store(0x02); // channel 1 -> domain slot 2
    svc.pollOnce();

    const auto reel = reelRepo_->getBySlot(1, 2);
    ASSERT_TRUE(reel.has_value());
    EXPECT_EQ(reel->barcode, "REEL-PA1");

    const auto op = opRepo_->getById(*opId);
    ASSERT_TRUE(op.has_value());
    EXPECT_EQ(op->slotIndex, 2);
    EXPECT_EQ(op->status, domain::OperationStatus::Completed);
}
