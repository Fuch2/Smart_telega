#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace smartcart::domain {

// Бизнес-состояние тележки по процессу из ТЗ 13.2.
enum class CartWorkflowState : std::uint8_t {
    Free               = 0,
    OrderLoaded        = 1,
    PickingMaterials   = 2,
    ReadyForFeederPrep = 3,
    FeederPrep         = 4,
    ReadyForLine       = 5,
    IssuingToLine      = 6,
    OrderCompleted     = 7,
    LeftoversDetected  = 8,
    ReturningLeftovers = 9,
};

inline std::string_view toString(CartWorkflowState state) {
    switch (state) {
        case CartWorkflowState::Free:               return "FREE";
        case CartWorkflowState::OrderLoaded:        return "ORDER_LOADED";
        case CartWorkflowState::PickingMaterials:   return "PICKING_MATERIALS";
        case CartWorkflowState::ReadyForFeederPrep: return "READY_FOR_FEEDER_PREP";
        case CartWorkflowState::FeederPrep:         return "FEEDER_PREP";
        case CartWorkflowState::ReadyForLine:       return "READY_FOR_LINE";
        case CartWorkflowState::IssuingToLine:      return "ISSUING_TO_LINE";
        case CartWorkflowState::OrderCompleted:     return "ORDER_COMPLETED";
        case CartWorkflowState::LeftoversDetected:  return "LEFTOVERS_DETECTED";
        case CartWorkflowState::ReturningLeftovers: return "RETURNING_LEFTOVERS";
    }
    return "UNKNOWN";
}

inline CartWorkflowState cartWorkflowStateFromString(std::string_view value) {
    if (value == "FREE")                  return CartWorkflowState::Free;
    if (value == "ORDER_LOADED")          return CartWorkflowState::OrderLoaded;
    if (value == "PICKING_MATERIALS")     return CartWorkflowState::PickingMaterials;
    if (value == "READY_FOR_FEEDER_PREP") return CartWorkflowState::ReadyForFeederPrep;
    if (value == "FEEDER_PREP")           return CartWorkflowState::FeederPrep;
    if (value == "READY_FOR_LINE")        return CartWorkflowState::ReadyForLine;
    if (value == "ISSUING_TO_LINE")       return CartWorkflowState::IssuingToLine;
    if (value == "ORDER_COMPLETED")       return CartWorkflowState::OrderCompleted;
    if (value == "LEFTOVERS_DETECTED")    return CartWorkflowState::LeftoversDetected;
    if (value == "RETURNING_LEFTOVERS")   return CartWorkflowState::ReturningLeftovers;
    return CartWorkflowState::Free;
}

enum class OrderItemStatus : std::uint8_t {
    Pending   = 0,
    Placed    = 1,
    Issued    = 2,
    Returned  = 3,
    Missing   = 4,
    WrongSlot = 5,
};

inline std::string_view toString(OrderItemStatus status) {
    switch (status) {
        case OrderItemStatus::Pending:   return "PENDING";
        case OrderItemStatus::Placed:    return "PLACED";
        case OrderItemStatus::Issued:    return "ISSUED";
        case OrderItemStatus::Returned:  return "RETURNED";
        case OrderItemStatus::Missing:   return "MISSING";
        case OrderItemStatus::WrongSlot: return "WRONG_SLOT";
    }
    return "UNKNOWN";
}

inline OrderItemStatus orderItemStatusFromString(std::string_view value) {
    if (value == "PENDING")    return OrderItemStatus::Pending;
    if (value == "PLACED")     return OrderItemStatus::Placed;
    if (value == "ISSUED")     return OrderItemStatus::Issued;
    if (value == "RETURNED")   return OrderItemStatus::Returned;
    if (value == "MISSING")    return OrderItemStatus::Missing;
    if (value == "WRONG_SLOT") return OrderItemStatus::WrongSlot;
    return OrderItemStatus::Pending;
}

enum class OrderStatus : std::uint8_t {
    Loaded     = 0,
    InProgress = 1,
    Completed  = 2,
    Cancelled  = 3,
};

inline std::string_view toString(OrderStatus status) {
    switch (status) {
        case OrderStatus::Loaded:     return "LOADED";
        case OrderStatus::InProgress: return "IN_PROGRESS";
        case OrderStatus::Completed:  return "COMPLETED";
        case OrderStatus::Cancelled:  return "CANCELLED";
    }
    return "UNKNOWN";
}

inline OrderStatus orderStatusFromString(std::string_view value) {
    if (value == "LOADED")      return OrderStatus::Loaded;
    if (value == "IN_PROGRESS") return OrderStatus::InProgress;
    if (value == "COMPLETED")   return OrderStatus::Completed;
    if (value == "CANCELLED")   return OrderStatus::Cancelled;
    return OrderStatus::Loaded;
}

struct OrderInfo {
    int id{0};
    std::string externalOrderId;
    std::string title;
    std::string priority;
    int durationMinutes{0};
    OrderStatus status{OrderStatus::Loaded};
    std::string createdAt;
    std::string updatedAt;
};

struct OrderItem {
    int id{0};
    int orderId{0};
    std::string barcode;
    std::string materialType;
    int targetSlot{0}; // 1..N
    std::optional<int> currentSlot;
    OrderItemStatus status{OrderItemStatus::Pending};
    std::string updatedAt;
};

struct CartWorkflow {
    std::optional<int> currentOrderId;
    CartWorkflowState state{CartWorkflowState::Free};
    std::string updatedAt;
};

} // namespace smartcart::domain
