#include "infrastructure/db/SqliteConnection.hpp"
#include "infrastructure/db/repositories/DiagnosticsRepositorySqlite.hpp"
#include "infrastructure/db/repositories/OrderRepositorySqlite.hpp"
#include "infrastructure/db/repositories/ReelRepositorySqlite.hpp"
#include "infrastructure/db/repositories/WorkflowRepositorySqlite.hpp"
#include "domain/entities/CartWorkflow.hpp"
#include "domain/entities/Slot.hpp"

#include <gtest/gtest.h>

#include <string>

using namespace smartcart;

TEST(OrderWorkflowRepositoryTest, MigrationsCreateOrderWorkflowTables) {
    infrastructure::db::SqliteConnection conn(":memory:");
    conn.runMigrations(std::string{MIGRATIONS_DIR});

    infrastructure::db::OrderRepositorySqlite orderRepo(conn);
    infrastructure::db::WorkflowRepositorySqlite workflowRepo(conn);

    domain::OrderInfo order;
    order.externalOrderId = "ORDER-001";
    order.title = "Demo order";
    order.priority = "normal";
    order.durationMinutes = 120;
    order.status = domain::OrderStatus::Loaded;

    const int orderId = orderRepo.addOrder(order);
    ASSERT_GT(orderId, 0);

    domain::OrderItem reel;
    reel.orderId = orderId;
    reel.barcode = "R-150 10%";
    reel.materialType = "reel";
    reel.targetSlot = 2;

    const int itemId = orderRepo.addItem(reel);
    ASSERT_GT(itemId, 0);

    ASSERT_TRUE(workflowRepo.setCurrentOrder(
        orderId,
        domain::CartWorkflowState::OrderLoaded));

    const auto workflow = workflowRepo.get();
    ASSERT_TRUE(workflow.currentOrderId.has_value());
    EXPECT_EQ(*workflow.currentOrderId, orderId);
    EXPECT_EQ(workflow.state, domain::CartWorkflowState::OrderLoaded);

    const auto activeOrder = orderRepo.getActiveOrder();
    ASSERT_TRUE(activeOrder.has_value());
    EXPECT_EQ(activeOrder->externalOrderId, "ORDER-001");
    EXPECT_EQ(activeOrder->status, domain::OrderStatus::Loaded);

    const auto items = orderRepo.getItems(orderId);
    ASSERT_EQ(items.size(), 1U);
    EXPECT_EQ(items.front().barcode, "R-150 10%");
    EXPECT_EQ(items.front().targetSlot, 2);
    EXPECT_EQ(items.front().status, domain::OrderItemStatus::Pending);

    ASSERT_TRUE(orderRepo.updateItemPlacement(
        itemId,
        2,
        domain::OrderItemStatus::Placed));

    const auto placed = orderRepo.findItemByBarcode(orderId, "R-150 10%");
    ASSERT_TRUE(placed.has_value());
    ASSERT_TRUE(placed->currentSlot.has_value());
    EXPECT_EQ(*placed->currentSlot, 2);
    EXPECT_EQ(placed->status, domain::OrderItemStatus::Placed);
}

TEST(OrderWorkflowRepositoryTest, DiagnosticsResetTestCartClearsWorkflowData) {
    infrastructure::db::SqliteConnection conn(":memory:");
    conn.runMigrations(std::string{MIGRATIONS_DIR});

    infrastructure::db::OrderRepositorySqlite orderRepo(conn);
    infrastructure::db::ReelRepositorySqlite reelRepo(conn);
    infrastructure::db::WorkflowRepositorySqlite workflowRepo(conn);
    infrastructure::db::DiagnosticsRepositorySqlite diagnosticsRepo(conn);

    conn.execute(
        "INSERT INTO modules(id,serial,slot_count,firmware,status)"
        " VALUES(1,'TEST-MODULE',24,'','ONLINE');"
    );

    domain::OrderInfo order;
    order.externalOrderId = "ORDER-RESET";
    order.title = "Reset order";
    order.priority = "normal";
    const int orderId = orderRepo.addOrder(order);

    domain::OrderItem item;
    item.orderId = orderId;
    item.barcode = "R-RESET";
    item.materialType = "reel";
    item.targetSlot = 1;
    orderRepo.addItem(item);
    workflowRepo.setCurrentOrder(orderId, domain::CartWorkflowState::PickingMaterials);

    reelRepo.addRecord(1, 1, "R-RESET");
    reelRepo.setSlotState(1, 1, domain::SlotState::Occupied);

    ASSERT_TRUE(diagnosticsRepo.resetTestCart(1));

    const auto workflow = workflowRepo.get();
    EXPECT_FALSE(workflow.currentOrderId.has_value());
    EXPECT_EQ(workflow.state, domain::CartWorkflowState::Free);
    EXPECT_FALSE(orderRepo.getActiveOrder().has_value());
    EXPECT_FALSE(reelRepo.getBySlot(1, 1).has_value());

    const auto slotStates = reelRepo.getSlotStates(1);
    ASSERT_FALSE(slotStates.empty());
    EXPECT_EQ(slotStates.front().state, domain::SlotState::Free);

    const auto events = diagnosticsRepo.recentEvents(5);
    ASSERT_FALSE(events.empty());
    EXPECT_EQ(events.front().code, "TestCartReset");
}
