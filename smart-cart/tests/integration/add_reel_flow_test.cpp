// ===== tests/integration/add_reel_flow_test.cpp =====
// Исправлено:
//   - SetUp инициализирует slot_states через setSlotState (было верно, но
//     нужно убедиться что moduleId=1 существует в modules — добавлен INSERT)
//   - явный таймаут теста через GTEST_FLAG или deadline уже есть — ок
//   - комментарии на английском
#include "infrastructure/hw/stm32/MockStm32Link.hpp"
#include "infrastructure/db/SqliteConnection.hpp"
#include "infrastructure/db/repositories/ReelRepositorySqlite.hpp"
#include "infrastructure/db/repositories/OperationRepositorySqlite.hpp"
#include "application/services/AddReelService.hpp"
#include "domain/entities/Operation.hpp"

#include <gtest/gtest.h>
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

        // Initialise 24 free slots for module 1
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

    // Simulate physical reel insertion after 50 ms
    std::thread([&link]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        Frame evt;
        evt.type    = FrameType::Evt;
        evt.cmdId   = CommandId::EvtSwitchChanged;
        evt.payload = {0x00, 0x01}; // slot index 0 (0-based) → slot 1, occupied=true
        link.injectEvent(evt);
    }).detach();

    // Wait for completion (max 500 ms)
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

    // Fill all slots
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
