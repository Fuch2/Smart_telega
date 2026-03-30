// ===== tests/integration/replace_reel_flow_test.cpp =====
// Исправлено:
//   - SetUp: addRecord создаёт запись в reels + setSlotState — порядок важен
//   - комментарии на английском
#include "infrastructure/hw/stm32/MockStm32Link.hpp"
#include "infrastructure/db/SqliteConnection.hpp"
#include "infrastructure/db/repositories/ReelRepositorySqlite.hpp"
#include "infrastructure/db/repositories/OperationRepositorySqlite.hpp"
#include "application/services/ReplaceReelService.hpp"

#include <gtest/gtest.h>
#include <chrono>
#include <thread>

using namespace smartcart;
using namespace smartcart::infrastructure::hw::stm32;
using namespace smartcart::application::services;

class ReplaceReelFlowTest : public ::testing::Test {
protected:
    void SetUp() override {
        conn_     = std::make_unique<infrastructure::db::SqliteConnection>(":memory:");
        reelRepo_ = std::make_unique<infrastructure::db::ReelRepositorySqlite>(*conn_);
        opRepo_   = std::make_unique<infrastructure::db::OperationRepositorySqlite>(*conn_);

        // Slot 3 is occupied with reel OLD-001
        // Order matters: addRecord first, then setSlotState
        reelRepo_->addRecord(1, 3, "OLD-001");
        reelRepo_->setSlotState(1, 3, domain::SlotState::Occupied);
    }

    std::unique_ptr<infrastructure::db::SqliteConnection>          conn_;
    std::unique_ptr<infrastructure::db::ReelRepositorySqlite>      reelRepo_;
    std::unique_ptr<infrastructure::db::OperationRepositorySqlite> opRepo_;
};

TEST_F(ReplaceReelFlowTest, ReelNotFound_ReturnsError) {
    MockStm32Link link;
    link.open();

    ReplaceReelConfig cfg;
    cfg.moduleId        = 1;
    cfg.stableConfirmMs = 200;

    ReplaceReelService svc(link, *reelRepo_, *opRepo_, cfg);

    domain::ErrorCode gotCode = domain::ErrorCode::None;
    svc.setErrorCallback([&](domain::ErrorCode c, std::string) { gotCode = c; });

    EXPECT_EQ(svc.start("NONEXISTENT"), -1);
    EXPECT_EQ(gotCode, domain::ErrorCode::ReelNotFound);
}

TEST_F(ReplaceReelFlowTest, SuccessfulReplace_CompletesOperation) {
    MockStm32Link link;
    link.open();

    ReplaceReelConfig cfg;
    cfg.moduleId        = 1;
    cfg.stableConfirmMs = 300;

    ReplaceReelService svc(link, *reelRepo_, *opRepo_, cfg);

    domain::OperationStatus finalStatus = domain::OperationStatus::InProgress;
    svc.setCompletionCallback([&](int, domain::OperationStatus s) {
        finalStatus = s;
    });

    // OLD-001 is in slot 3 — start() should find it via findActiveByBarcode
    const int opId = svc.start("OLD-001");
    ASSERT_GT(opId, 0);

    // Step A: operator removes old reel (slot 3, not occupied)
    // Step B: operator inserts new reel (slot 3, occupied)
    std::thread([&link]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        Frame evt;
        evt.type    = FrameType::Evt;
        evt.cmdId   = CommandId::EvtSwitchChanged;
        evt.payload = {0x02, 0x00}; // slot index 2 (0-based) = slot 3, occupied=false
        link.injectEvent(evt);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        evt.payload = {0x02, 0x01}; // slot 3, occupied=true
        link.injectEvent(evt);
    }).detach();

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(800);
    while (finalStatus == domain::OperationStatus::InProgress &&
           std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_EQ(finalStatus, domain::OperationStatus::Completed);
    // Old reel should be marked removed, new record should exist
    EXPECT_TRUE(reelRepo_->hasActiveRecord(1, 3));
}
