#pragma once

#include "domain/entities/Operation.hpp"
#include "domain/entities/Slot.hpp"

#include <string>
#include <vector>

namespace smartcart::application::dto {

// Событие изменения физического состояния слота (от STM32)
struct SlotPhysicalChangedEvent {
    int slotIndex {0};   // 0-based
    bool occupied {false};
};

// Событие завершения операции
struct OperationFinishedEvent {
    int operationId {0};
    smartcart::domain::OperationStatus status {smartcart::domain::OperationStatus::Failed};
    std::string message {};
};

// Снимок слотов для UI/презентации
struct SlotsSnapshotEvent {
    std::vector<smartcart::domain::Slot> slots;
};

} // namespace smartcart::application::dto
