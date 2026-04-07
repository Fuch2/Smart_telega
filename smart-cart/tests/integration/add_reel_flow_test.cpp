// ===== tests/integration/add_reel_flow_test.cpp =====
#include "infrastructure/hw/stm32/MockStm32Link.hpp"
#include "infrastructure/db/SqliteConnection.hpp"
#include "infrastructure/db/repositories/ReelRepositorySqlite.hpp"
#include "infrastructure/db/repositories/OperationRepositorySqlite.hpp"
#include "infrastructure/db/repositories/OrderRepositorySqlite.hpp"
#include "infrastructure/db/repositories/WorkflowRepositorySqlite.hpp"
#include "infrastructure/logging/SqliteEventLogger.hpp"
#include "application/services/AddReelService.hpp"
#include "application/services/Stm32PollingService.hpp"
#include "application/services/WorkflowService.hpp"
#include "domain/entities/CartWorkflow.hpp"
#include "domain/entities/Operation.hpp"

#include <sqlite3.h>

#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>

using namespace smartcart;
using namespace smartcart::infrastructure::hw::stm32;
using namespace smartcart::application::services;

namespace {

int countEvents(sqlite3* db, const std::string& code) {
    const char* sql = "SELECT COUNT(1) FROM event_log WHERE code = ?;";
    sqlite3_stmt* stmt = nullptr;
    sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr);
    sqlite3_bind_text(stmt, 1, code.c_str(), -1, SQLITE_TRANSIENT);

    int count = 0;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        count = sqlite3_column_int(stmt, 0);
    }
    sqlite3_finalize(stmt);
    return count;
}

} // namespace

class AddReelFlowTest : public ::testing::Test {
protected:
    void SetUp() override {
        conn_     = std::make_unique<infrastructure::db::SqliteConnection>(":memory:");
        reelRepo_ = std::make_unique<infrastructure::db::ReelRepositorySqlite>(*conn_);
        opRepo_   = std::make_unique<infrastructure::db::OperationRepositorySqlite>(*conn_);
        orderRepo_ = std::make_unique<infrastructure::db::OrderRepositorySqlite>(*conn_);
        workflowRepo_ = std::make_unique<infrastructure::db::WorkflowRepositorySqlite>(*conn_);

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
    std::unique_ptr<infrastructure::db::OrderRepositorySqlite>     orderRepo_;
    std::unique_ptr<infrastructure::db::WorkflowRepositorySqlite>  workflowRepo_;
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
    WorkflowService workflowSvc(
        *orderRepo_,
        *workflowRepo_,
        *reelRepo_,
        logger,
        1
    );

    Stm32PollingConfig cfg;
    cfg.moduleId = 1;
    cfg.slotCount = 24;
    cfg.pollMs = 10;
    cfg.debounceMs = 0;
    cfg.trackedChannels = {1, 3};
    cfg.ignoredChannels = {11};

    domain::OrderInfo order;
    order.externalOrderId = "ORDER-PA1";
    order.title = "PA1 order";
    const int orderId = orderRepo_->addOrder(order);

    domain::OrderItem item;
    item.orderId = orderId;
    item.barcode = "REEL-PA1";
    item.materialType = "reel";
    item.targetSlot = 2;
    orderRepo_->addItem(item);
    workflowRepo_->setCurrentOrder(orderId, domain::CartWorkflowState::OrderLoaded);

    Stm32PollingService svc(link,
                            *reelRepo_,
                            *opRepo_,
                            *orderRepo_,
                            *workflowRepo_,
                            workflowSvc,
                            logger,
                            cfg);
    const auto scan = svc.recordBarcodeScan("REEL-PA1");
    ASSERT_TRUE(scan.success);
    EXPECT_EQ(scan.targetSlot, 2);

    mask0.store(0x02); // channel 1 -> domain slot 2
    svc.pollOnce();

    const auto reel = reelRepo_->getBySlot(1, 2);
    ASSERT_TRUE(reel.has_value());
    EXPECT_EQ(reel->barcode, "REEL-PA1");

    const auto op = opRepo_->getById(scan.operationId);
    ASSERT_TRUE(op.has_value());
    EXPECT_EQ(op->slotIndex, 2);
    EXPECT_EQ(op->status, domain::OperationStatus::Completed);

    const auto placed = orderRepo_->findItemByBarcode(orderId, "REEL-PA1");
    ASSERT_TRUE(placed.has_value());
    EXPECT_EQ(placed->status, domain::OrderItemStatus::Placed);
}

TEST_F(AddReelFlowTest, PollingScanThenWrongPa2_DoesNotCreateReel) {
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
    WorkflowService workflowSvc(
        *orderRepo_,
        *workflowRepo_,
        *reelRepo_,
        logger,
        1
    );

    Stm32PollingConfig cfg;
    cfg.moduleId = 1;
    cfg.slotCount = 24;
    cfg.pollMs = 10;
    cfg.debounceMs = 0;
    cfg.trackedChannels = {1, 3};
    cfg.ignoredChannels = {11};

    domain::OrderInfo order;
    order.externalOrderId = "ORDER-WRONG-SLOT";
    order.title = "Wrong slot order";
    const int orderId = orderRepo_->addOrder(order);

    domain::OrderItem item;
    item.orderId = orderId;
    item.barcode = "REEL-WRONG";
    item.materialType = "reel";
    item.targetSlot = 2;
    orderRepo_->addItem(item);
    workflowRepo_->setCurrentOrder(orderId, domain::CartWorkflowState::OrderLoaded);

    Stm32PollingService svc(link,
                            *reelRepo_,
                            *opRepo_,
                            *orderRepo_,
                            *workflowRepo_,
                            workflowSvc,
                            logger,
                            cfg);
    const auto scan = svc.recordBarcodeScan("REEL-WRONG");
    ASSERT_TRUE(scan.success);
    EXPECT_EQ(scan.targetSlot, 2);

    mask0.store(0x08); // channel 3 -> domain slot 4
    svc.pollOnce();

    EXPECT_FALSE(reelRepo_->getBySlot(1, 4).has_value());

    const auto op = opRepo_->getById(scan.operationId);
    ASSERT_TRUE(op.has_value());
    EXPECT_EQ(op->slotIndex, 4);
    EXPECT_EQ(op->status, domain::OperationStatus::Failed);

    const auto placed = orderRepo_->findItemByBarcode(orderId, "REEL-WRONG");
    ASSERT_TRUE(placed.has_value());
    ASSERT_TRUE(placed->currentSlot.has_value());
    EXPECT_EQ(*placed->currentSlot, 4);
    EXPECT_EQ(placed->status, domain::OrderItemStatus::WrongSlot);
    EXPECT_EQ(countEvents(conn_->handle(), "WrongSlotInteraction"), 1);
}

TEST_F(AddReelFlowTest, PollingDebounce_IgnoresFastReleaseBounce) {
    std::atomic<uint8_t> mask0{0x02}; // channel 1 -> domain slot 2

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
    WorkflowService workflowSvc(
        *orderRepo_,
        *workflowRepo_,
        *reelRepo_,
        logger,
        1
    );

    Stm32PollingConfig cfg;
    cfg.moduleId = 1;
    cfg.slotCount = 24;
    cfg.pollMs = 10;
    cfg.debounceMs = 50;
    cfg.trackedChannels = {1, 3};
    cfg.ignoredChannels = {11};

    reelRepo_->addRecord(1, 2, "REEL-STABLE");
    reelRepo_->setSlotState(1, 2, domain::SlotState::Occupied);

    Stm32PollingService svc(link,
                            *reelRepo_,
                            *opRepo_,
                            *orderRepo_,
                            *workflowRepo_,
                            workflowSvc,
                            logger,
                            cfg);

    svc.pollOnce();
    ASSERT_TRUE(reelRepo_->getBySlot(1, 2).has_value());

    mask0.store(0x00);
    svc.pollOnce();
    mask0.store(0x02);
    svc.pollOnce();

    EXPECT_TRUE(reelRepo_->getBySlot(1, 2).has_value());
    EXPECT_EQ(countEvents(conn_->handle(), "ReelRemovedBySwitch"), 0);

    mask0.store(0x00);
    svc.pollOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(70));
    svc.pollOnce();

    EXPECT_FALSE(reelRepo_->getBySlot(1, 2).has_value());
    EXPECT_EQ(countEvents(conn_->handle(), "ReelRemovedBySwitch"), 1);
}
