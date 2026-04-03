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

TEST(OrderWorkflowRepositoryTest, OrderAndWorkflowAreScopedByModuleId) {
    infrastructure::db::SqliteConnection conn(":memory:");
    conn.runMigrations(std::string{MIGRATIONS_DIR});

    infrastructure::db::OrderRepositorySqlite orderRepoModule1(conn, 1);
    infrastructure::db::OrderRepositorySqlite orderRepoModule2(conn, 2);
    infrastructure::db::WorkflowRepositorySqlite workflowRepoModule1(conn, 1);
    infrastructure::db::WorkflowRepositorySqlite workflowRepoModule2(conn, 2);

    domain::OrderInfo order1;
    order1.externalOrderId = "ORDER-SHARED";
    order1.title = "Module 1 order";
    order1.priority = "normal";
    const int orderId1 = orderRepoModule1.addOrder(order1);
    ASSERT_GT(orderId1, 0);

    domain::OrderInfo order2;
    order2.externalOrderId = "ORDER-SHARED";
    order2.title = "Module 2 order";
    order2.priority = "high";
    const int orderId2 = orderRepoModule2.addOrder(order2);
    ASSERT_GT(orderId2, 0);
    EXPECT_NE(orderId1, orderId2);

    ASSERT_TRUE(workflowRepoModule1.setCurrentOrder(
        orderId1,
        domain::CartWorkflowState::OrderLoaded));
    ASSERT_TRUE(workflowRepoModule2.setCurrentOrder(
        orderId2,
        domain::CartWorkflowState::PickingMaterials));

    const auto workflow1 = workflowRepoModule1.get();
    const auto workflow2 = workflowRepoModule2.get();
    ASSERT_TRUE(workflow1.currentOrderId.has_value());
    ASSERT_TRUE(workflow2.currentOrderId.has_value());
    EXPECT_EQ(*workflow1.currentOrderId, orderId1);
    EXPECT_EQ(*workflow2.currentOrderId, orderId2);
    EXPECT_EQ(workflow1.state, domain::CartWorkflowState::OrderLoaded);
    EXPECT_EQ(workflow2.state, domain::CartWorkflowState::PickingMaterials);

    const auto active1 = orderRepoModule1.getActiveOrder();
    const auto active2 = orderRepoModule2.getActiveOrder();
    ASSERT_TRUE(active1.has_value());
    ASSERT_TRUE(active2.has_value());
    EXPECT_EQ(active1->title, "Module 1 order");
    EXPECT_EQ(active2->title, "Module 2 order");
}
