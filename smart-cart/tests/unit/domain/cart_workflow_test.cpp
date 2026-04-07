#include "domain/entities/CartWorkflow.hpp"

#include <gtest/gtest.h>

using namespace smartcart::domain;

TEST(CartWorkflowDomainTest, WorkflowStateConvertsToStableDbString) {
    EXPECT_EQ(toString(CartWorkflowState::Free), "FREE");
    EXPECT_EQ(toString(CartWorkflowState::OrderLoaded), "ORDER_LOADED");
    EXPECT_EQ(toString(CartWorkflowState::PickingMaterials), "PICKING_MATERIALS");
    EXPECT_EQ(toString(CartWorkflowState::ReadyForFeederPrep), "READY_FOR_FEEDER_PREP");
    EXPECT_EQ(toString(CartWorkflowState::FeederPrep), "FEEDER_PREP");
    EXPECT_EQ(toString(CartWorkflowState::ReadyForLine), "READY_FOR_LINE");
    EXPECT_EQ(toString(CartWorkflowState::IssuingToLine), "ISSUING_TO_LINE");
    EXPECT_EQ(toString(CartWorkflowState::OrderCompleted), "ORDER_COMPLETED");
    EXPECT_EQ(toString(CartWorkflowState::LeftoversDetected), "LEFTOVERS_DETECTED");
    EXPECT_EQ(toString(CartWorkflowState::ReturningLeftovers), "RETURNING_LEFTOVERS");
}

TEST(CartWorkflowDomainTest, WorkflowStateParsesDbString) {
    EXPECT_EQ(cartWorkflowStateFromString("FREE"), CartWorkflowState::Free);
    EXPECT_EQ(cartWorkflowStateFromString("ORDER_LOADED"), CartWorkflowState::OrderLoaded);
    EXPECT_EQ(cartWorkflowStateFromString("PICKING_MATERIALS"), CartWorkflowState::PickingMaterials);
    EXPECT_EQ(cartWorkflowStateFromString("READY_FOR_FEEDER_PREP"), CartWorkflowState::ReadyForFeederPrep);
    EXPECT_EQ(cartWorkflowStateFromString("FEEDER_PREP"), CartWorkflowState::FeederPrep);
    EXPECT_EQ(cartWorkflowStateFromString("READY_FOR_LINE"), CartWorkflowState::ReadyForLine);
    EXPECT_EQ(cartWorkflowStateFromString("ISSUING_TO_LINE"), CartWorkflowState::IssuingToLine);
    EXPECT_EQ(cartWorkflowStateFromString("ORDER_COMPLETED"), CartWorkflowState::OrderCompleted);
    EXPECT_EQ(cartWorkflowStateFromString("LEFTOVERS_DETECTED"), CartWorkflowState::LeftoversDetected);
    EXPECT_EQ(cartWorkflowStateFromString("RETURNING_LEFTOVERS"), CartWorkflowState::ReturningLeftovers);
    EXPECT_EQ(cartWorkflowStateFromString("UNKNOWN"), CartWorkflowState::Free);
}

TEST(CartWorkflowDomainTest, OrderItemStatusConvertsToStableDbString) {
    EXPECT_EQ(toString(OrderItemStatus::Pending), "PENDING");
    EXPECT_EQ(toString(OrderItemStatus::Placed), "PLACED");
    EXPECT_EQ(toString(OrderItemStatus::Issued), "ISSUED");
    EXPECT_EQ(toString(OrderItemStatus::Returned), "RETURNED");
    EXPECT_EQ(toString(OrderItemStatus::Missing), "MISSING");
    EXPECT_EQ(toString(OrderItemStatus::WrongSlot), "WRONG_SLOT");
}

TEST(CartWorkflowDomainTest, OrderItemStatusParsesDbString) {
    EXPECT_EQ(orderItemStatusFromString("PENDING"), OrderItemStatus::Pending);
    EXPECT_EQ(orderItemStatusFromString("PLACED"), OrderItemStatus::Placed);
    EXPECT_EQ(orderItemStatusFromString("ISSUED"), OrderItemStatus::Issued);
    EXPECT_EQ(orderItemStatusFromString("RETURNED"), OrderItemStatus::Returned);
    EXPECT_EQ(orderItemStatusFromString("MISSING"), OrderItemStatus::Missing);
    EXPECT_EQ(orderItemStatusFromString("WRONG_SLOT"), OrderItemStatus::WrongSlot);
    EXPECT_EQ(orderItemStatusFromString("UNKNOWN"), OrderItemStatus::Pending);
}

TEST(CartWorkflowDomainTest, OrderStatusConvertsToStableDbString) {
    EXPECT_EQ(toString(OrderStatus::Loaded), "LOADED");
    EXPECT_EQ(toString(OrderStatus::InProgress), "IN_PROGRESS");
    EXPECT_EQ(toString(OrderStatus::Completed), "COMPLETED");
    EXPECT_EQ(toString(OrderStatus::Cancelled), "CANCELLED");
}

TEST(CartWorkflowDomainTest, OrderStatusParsesDbString) {
    EXPECT_EQ(orderStatusFromString("LOADED"), OrderStatus::Loaded);
    EXPECT_EQ(orderStatusFromString("IN_PROGRESS"), OrderStatus::InProgress);
    EXPECT_EQ(orderStatusFromString("COMPLETED"), OrderStatus::Completed);
    EXPECT_EQ(orderStatusFromString("CANCELLED"), OrderStatus::Cancelled);
    EXPECT_EQ(orderStatusFromString("UNKNOWN"), OrderStatus::Loaded);
}
