// ===== tests/integration/uart_recovery_test.cpp =====
#include "infrastructure/hw/stm32/MockStm32Link.hpp"
#include "infrastructure/db/SqliteConnection.hpp"
#include "infrastructure/db/repositories/ReelRepositorySqlite.hpp"
#include "infrastructure/db/repositories/OperationRepositorySqlite.hpp"
#include "application/services/RecoveryService.hpp"
#include "domain/entities/Operation.hpp"

#include <gtest/gtest.h>

using namespace smartcart;
using namespace smartcart::infrastructure::hw::stm32;
using namespace smartcart::application::services;

class RecoveryFlowTest : public ::testing::Test {
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
    }

    static MockStm32Link::Handler snapshotHandler(uint8_t b0, uint8_t b1, uint8_t b2) {
        return [b0, b1, b2](const Frame& cmd) -> std::optional<Frame> {
            Frame resp;
            resp.seq   = cmd.seq;
            resp.cmdId = cmd.cmdId;
            if (cmd.cmdId == CommandId::GetSwitchSnapshot) {
                resp.type    = FrameType::Resp;
                resp.payload = {b0, b1, b2};
            } else {
                resp.type = FrameType::Ack;
            }
            return resp;
        };
    }

    std::unique_ptr<infrastructure::db::SqliteConnection>          conn_;
    std::unique_ptr<infrastructure::db::ReelRepositorySqlite>      reelRepo_;
    std::unique_ptr<infrastructure::db::OperationRepositorySqlite> opRepo_;
};

TEST_F(RecoveryFlowTest, AddReel_PhysOccupied_Completes) {
    domain::Operation op;
    op.type      = domain::OperationType::AddReel;
    op.status    = domain::OperationStatus::InProgress;
    op.moduleId  = 1;
    op.slotIndex = 5;
    op.barcode   = "REEL-RECOVER-001";
    const int opId = opRepo_->add(op);

    reelRepo_->setSlotState(1, 5, domain::SlotState::Reserved);

    MockStm32Link link(snapshotHandler(0x10, 0x00, 0x00));
    link.open();

    RecoveryConfig cfg;
    cfg.moduleId  = 1;
    cfg.slotCount = 24;

    RecoveryService svc(*opRepo_, *reelRepo_, link, cfg);
    EXPECT_TRUE(svc.run());

    const auto recovered = opRepo_->getById(opId);
    ASSERT_TRUE(recovered.has_value());
    EXPECT_EQ(recovered->status, domain::OperationStatus::Completed);
    EXPECT_TRUE(reelRepo_->hasActiveRecord(1, 5));
}

TEST_F(RecoveryFlowTest, AddReel_PhysEmpty_Cancels) {
    domain::Operation op;
    op.type      = domain::OperationType::AddReel;
    op.status    = domain::OperationStatus::InProgress;
    op.moduleId  = 1;
    op.slotIndex = 7;
    op.barcode   = "REEL-RECOVER-002";
    const int opId = opRepo_->add(op);

    reelRepo_->setSlotState(1, 7, domain::SlotState::Reserved);

    MockStm32Link link(snapshotHandler(0x00, 0x00, 0x00));
    link.open();

    RecoveryConfig cfg;
    cfg.moduleId  = 1;
    cfg.slotCount = 24;

    RecoveryService svc(*opRepo_, *reelRepo_, link, cfg);
    EXPECT_TRUE(svc.run());

    const auto recovered = opRepo_->getById(opId);
    ASSERT_TRUE(recovered.has_value());
    EXPECT_EQ(recovered->status, domain::OperationStatus::Cancelled);

    const auto slotStates = reelRepo_->getSlotStates(1);
    for (const auto& s : slotStates) {
        if (s.slotIndex == 7)
            EXPECT_EQ(s.state, domain::SlotState::Free);
    }
}

TEST_F(RecoveryFlowTest, RemoveReel_PhysEmpty_Completes) {
    reelRepo_->addRecord(1, 2, "REEL-REMOVE-001");
    reelRepo_->setSlotState(1, 2, domain::SlotState::Occupied);

    domain::Operation op;
    op.type      = domain::OperationType::RemoveReel;
    op.status    = domain::OperationStatus::InProgress;
    op.moduleId  = 1;
    op.slotIndex = 2;
    op.barcode   = "REEL-REMOVE-001";
    const int opId = opRepo_->add(op);

    MockStm32Link link(snapshotHandler(0x00, 0x00, 0x00));
    link.open();

    RecoveryConfig cfg;
    cfg.moduleId  = 1;
    cfg.slotCount = 24;

    RecoveryService svc(*opRepo_, *reelRepo_, link, cfg);
    EXPECT_TRUE(svc.run());

    const auto recovered = opRepo_->getById(opId);
    ASSERT_TRUE(recovered.has_value());
    EXPECT_EQ(recovered->status, domain::OperationStatus::Completed);
    EXPECT_FALSE(reelRepo_->hasActiveRecord(1, 2));
}
