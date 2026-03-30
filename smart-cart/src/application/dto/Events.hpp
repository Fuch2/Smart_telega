// ===== src/application/dto/Events.hpp =====
// Исправлено: убран encoding-мусор в комментариях
#pragma once

#include "domain/entities/Slot.hpp"
#include "domain/entities/Operation.hpp"
#include "domain/errors/ErrorCode.hpp"

#include <string>
#include <vector>

namespace smartcart::application::dto {

/// Снимок состояния одного слота (для передачи в UI)
struct SlotSnapshot {
    int  moduleId   = 0;
    int  slotIndex  = 0;
    int  ledIndex   = 0;
    domain::SlotState state = domain::SlotState::Free;
    std::string barcode;   // пусто если слот свободен
};

/// Событие завершения / изменения операции
struct OperationEvent {
    int                        operationId = 0;
    domain::OperationType      type        = domain::OperationType::AddReel;
    domain::OperationStatus    status      = domain::OperationStatus::InProgress;
    int         moduleId  = 0;
    int         slotIndex = 0;
    std::string barcode;
    std::string finishedAt;
};

/// Событие ошибки
struct ErrorEvent {
    domain::ErrorCode code    = domain::ErrorCode::None;
    std::string       message;
};

} // namespace smartcart::application::dto
