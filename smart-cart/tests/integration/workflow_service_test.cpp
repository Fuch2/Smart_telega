#include "application/services/Stm32PollingService.hpp"
#include "application/services/WorkflowService.hpp"
#include "infrastructure/db/SqliteConnection.hpp"
#include "infrastructure/db/repositories/OperationRepositorySqlite.hpp"
#include "infrastructure/db/repositories/OrderRepositorySqlite.hpp"
#include "infrastructure/db/repositories/ReelRepositorySqlite.hpp"
#include "infrastructure/db/repositories/WorkflowRepositorySqlite.hpp"
#include "infrastructure/hw/stm32/MockStm32Link.hpp"
#include "infrastructure/logging/SqliteEventLogger.hpp"
#include "domain/entities/CartWorkflow.hpp"
#include "domain/entities/Slot.hpp"

#include <sqlite3.h>

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <utility>

using namespace smartcart;
using namespace smartcart::application::services;
using namespace smartcart::infrastructure::hw::stm32;

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

domain::OrderItem makeItem(int orderId,
                           std::string barcode,
                           int targetSlot,
                           domain::OrderItemStatus status =
                               domain::OrderItemStatus::Pending,
                           std::optional<int> currentSlot = std::nullopt) {
    domain::OrderItem item;
    item.orderId = orderId;
    item.barcode = std::move(barcode);
    item.materialType = "reel";
    item.targetSlot = targetSlot;
    item.status = status;
    item.currentSlot = currentSlot;
    return item;
}

} // namespace

class WorkflowServiceTest : public ::testing::Test {
protected:
    void SetUp() override {
        conn_ = std::make_unique<infrastructure::db::SqliteConnection>(":memory:");
        conn_->runMigrations(std::string{MIGRATIONS_DIR});

        reelRepo_ =
            std::make_unique<infrastructure::db::ReelRepositorySqlite>(*conn_);
        opRepo_ =
            std::make_unique<infrastructure::db::OperationRepositorySqlite>(*conn_);
        orderRepo_ =
            std::make_unique<infrastructure::db::OrderRepositorySqlite>(*conn_);
        workflowRepo_ =
            std::make_unique<infrastructure::db::WorkflowRepositorySqlite>(*conn_);
        logger_ =
            std::make_unique<infrastructure::logging::SqliteEventLogger>(
                conn_->handle());
        workflowSvc_ = std::make_unique<WorkflowService>(
            *orderRepo_,
            *workflowRepo_,
            *reelRepo_,
            *logger_,
            1
        );

        conn_->execute(
            "INSERT INTO modules(id,serial,slot_count,firmware,status)"
            " VALUES(1,'TEST-MODULE',24,'','ONLINE');"
        );

        for (int i = 1; i <= 24; ++i) {
            reelRepo_->setSlotState(1, i, domain::SlotState::Free);
        }
    }

    int addOrder(std::string externalOrderId) {
        domain::OrderInfo order;
        order.externalOrderId = std::move(externalOrderId);
        order.title = "Workflow test order";
        order.priority = "normal";
        return orderRepo_->addOrder(order);
    }

    Stm32PollingConfig pollingConfig() const {
        Stm32PollingConfig cfg;
        cfg.moduleId = 1;
        cfg.slotCount = 24;
        cfg.pollMs = 10;
        cfg.debounceMs = 0;
        cfg.trackedChannels = {1, 3};
        cfg.ignoredChannels = {11};
        return cfg;
    }

    std::unique_ptr<infrastructure::db::SqliteConnection> conn_;
    std::unique_ptr<infrastructure::db::ReelRepositorySqlite> reelRepo_;
    std::unique_ptr<infrastructure::db::OperationRepositorySqlite> opRepo_;
    std::unique_ptr<infrastructure::db::OrderRepositorySqlite> orderRepo_;
    std::unique_ptr<infrastructure::db::WorkflowRepositorySqlite> workflowRepo_;
    std::unique_ptr<infrastructure::logging::SqliteEventLogger> logger_;
    std::unique_ptr<WorkflowService> workflowSvc_;
};

TEST_F(WorkflowServiceTest, PickingCompletesAfterAllOrderItemsPlaced) {
    std::atomic<uint8_t> mask0{0x00};
    MockStm32Link link([&mask0](const Frame& cmd) -> std::optional<Frame> {
        Frame resp;
        resp.seq = cmd.seq;
        resp.cmdId = cmd.cmdId;

        if (cmd.cmdId == CommandId::GetSwitchSnapshot ||
            static_cast<uint8_t>(cmd.cmdId) == 0x04)
        {
            resp.type = FrameType::Resp;
            resp.payload = {mask0.load(), 0x00, 0x00};
        } else {
            resp.type = FrameType::Ack;
        }
        return resp;
    });
    link.open();

    const int orderId = addOrder("ORDER-PICKING-COMPLETE");
    orderRepo_->addItem(makeItem(orderId, "R-150 10%", 2));
    orderRepo_->addItem(makeItem(orderId, "R-250 20%", 4));
    workflowRepo_->setCurrentOrder(orderId, domain::CartWorkflowState::OrderLoaded);

    Stm32PollingService polling(link,
                                *reelRepo_,
                                *opRepo_,
                                *orderRepo_,
                                *workflowRepo_,
                                *workflowSvc_,
                                *logger_,
                                pollingConfig());

    ASSERT_TRUE(polling.recordBarcodeScan("R-150 10%"));
    mask0.store(0x02); // PA1 -> slot 2
    polling.pollOnce();
    EXPECT_EQ(workflowRepo_->get().state,
              domain::CartWorkflowState::PickingMaterials);

    ASSERT_TRUE(polling.recordBarcodeScan("R-250 20%"));
    mask0.store(0x0A); // PA1 + PA2, changed PA2 -> slot 4
    polling.pollOnce();
    EXPECT_EQ(workflowRepo_->get().state,
              domain::CartWorkflowState::ReadyForFeederPrep);
    EXPECT_EQ(countEvents(conn_->handle(), "PickingCompleted"), 1);
}

TEST_F(WorkflowServiceTest, FeederPrepTransitionsValidateState) {
    EXPECT_FALSE(workflowSvc_->startFeederPrep());
    EXPECT_EQ(countEvents(conn_->handle(), "InvalidWorkflowTransition"), 1);

    const int orderId = addOrder("ORDER-FEEDER-PREP");
    workflowRepo_->setCurrentOrder(
        orderId,
        domain::CartWorkflowState::ReadyForFeederPrep
    );

    EXPECT_TRUE(workflowSvc_->markCartArrivedToFeederPrep());
    EXPECT_TRUE(workflowSvc_->startFeederPrep());
    EXPECT_EQ(workflowRepo_->get().state, domain::CartWorkflowState::FeederPrep);

    EXPECT_TRUE(workflowSvc_->markFeederPrepCompleted());
    EXPECT_EQ(workflowRepo_->get().state, domain::CartWorkflowState::ReadyForLine);
    EXPECT_EQ(countEvents(conn_->handle(), "FeederPrepCompleted"), 1);
}

TEST_F(WorkflowServiceTest, IssuingMarksMaterialRemovedAndCartFree) {
    const int orderId = addOrder("ORDER-ISSUING");
    const int itemId = orderRepo_->addItem(
        makeItem(orderId,
                 "R-ISSUE",
                 2,
                 domain::OrderItemStatus::Placed,
                 2)
    );
    reelRepo_->addRecord(1, 2, "R-ISSUE");
    reelRepo_->setSlotState(1, 2, domain::SlotState::Occupied);
    workflowRepo_->setCurrentOrder(orderId, domain::CartWorkflowState::ReadyForLine);

    EXPECT_TRUE(workflowSvc_->markCartArrivedToLine());
    EXPECT_TRUE(workflowSvc_->startIssuingToLine());
    EXPECT_EQ(workflowRepo_->get().state, domain::CartWorkflowState::IssuingToLine);

    EXPECT_TRUE(workflowSvc_->markItemIssued("R-ISSUE"));
    EXPECT_FALSE(reelRepo_->getBySlot(1, 2).has_value());

    const auto item = orderRepo_->findItemByBarcode(orderId, "R-ISSUE");
    ASSERT_TRUE(item.has_value());
    EXPECT_EQ(item->id, itemId);
    EXPECT_EQ(item->status, domain::OrderItemStatus::Issued);

    const auto slotStates = reelRepo_->getSlotStates(1);
    ASSERT_GE(slotStates.size(), 2U);
    EXPECT_EQ(slotStates[1].state, domain::SlotState::Free);

    EXPECT_TRUE(workflowSvc_->completeIssuing());
    EXPECT_EQ(workflowRepo_->get().state, domain::CartWorkflowState::Free);
    EXPECT_EQ(countEvents(conn_->handle(), "MaterialIssued"), 1);
    EXPECT_EQ(countEvents(conn_->handle(), "OrderReport"), 1);
    EXPECT_EQ(countEvents(conn_->handle(), "CartFree"), 1);
}

TEST_F(WorkflowServiceTest, CompleteIssuingWithLeftoverStartsReturnFlow) {
    const int orderId = addOrder("ORDER-ISSUING-LEFTOVER");
    orderRepo_->addItem(
        makeItem(orderId,
                 "R-ISSUED",
                 2,
                 domain::OrderItemStatus::Issued,
                 2)
    );
    reelRepo_->addRecord(1, 5, "EXTRA-LEFTOVER");
    reelRepo_->setSlotState(1, 5, domain::SlotState::Occupied);
    workflowRepo_->setCurrentOrder(orderId, domain::CartWorkflowState::IssuingToLine);

    EXPECT_TRUE(workflowSvc_->completeIssuing());
    EXPECT_EQ(workflowRepo_->get().state,
              domain::CartWorkflowState::ReturningLeftovers);
    EXPECT_TRUE(workflowRepo_->get().currentOrderId.has_value());
    EXPECT_EQ(countEvents(conn_->handle(), "OrderReport"), 1);
    EXPECT_EQ(countEvents(conn_->handle(), "LeftoversDetected"), 1);
    EXPECT_EQ(countEvents(conn_->handle(), "ReturningLeftoversStarted"), 1);

    EXPECT_TRUE(workflowSvc_->markLeftoverReturnedByBarcode("EXTRA-LEFTOVER"));
    EXPECT_FALSE(workflowRepo_->get().currentOrderId.has_value());
    EXPECT_EQ(workflowRepo_->get().state, domain::CartWorkflowState::Free);
    EXPECT_EQ(countEvents(conn_->handle(), "LeftoverReturned"), 1);
    EXPECT_EQ(countEvents(conn_->handle(), "CartFree"), 1);
}

TEST_F(WorkflowServiceTest, LeftoversDetectedAndReturned) {
    const int orderId = addOrder("ORDER-LEFTOVERS");
    orderRepo_->updateOrderStatus(orderId, domain::OrderStatus::Completed);
    reelRepo_->addRecord(1, 2, "LEFTOVER-REEL");
    reelRepo_->setSlotState(1, 2, domain::SlotState::Occupied);
    workflowRepo_->setCurrentOrder(orderId, domain::CartWorkflowState::OrderCompleted);

    EXPECT_TRUE(workflowSvc_->inspectLeftoversAfterOrderCompleted());
    EXPECT_EQ(workflowRepo_->get().state,
              domain::CartWorkflowState::LeftoversDetected);
    EXPECT_EQ(countEvents(conn_->handle(), "LeftoversDetected"), 1);

    EXPECT_TRUE(workflowSvc_->startReturningLeftovers());
    EXPECT_EQ(workflowRepo_->get().state,
              domain::CartWorkflowState::ReturningLeftovers);

    EXPECT_TRUE(workflowSvc_->markLeftoverReturnedBySlot(2));
    EXPECT_FALSE(workflowRepo_->get().currentOrderId.has_value());
    EXPECT_EQ(workflowRepo_->get().state, domain::CartWorkflowState::Free);
    EXPECT_EQ(countEvents(conn_->handle(), "LeftoverReturned"), 1);
    EXPECT_EQ(countEvents(conn_->handle(), "CartFree"), 1);
}
