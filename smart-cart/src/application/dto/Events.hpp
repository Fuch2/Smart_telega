#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "src/domain/errors/ErrorCode.hpp"

namespace smartcart::application::dto {

struct SlotSnapshot {
    std::string slotId;   // S01..S24
    bool occupied{false};
    std::uint64_t tsMs{0};
};

struct Stm32SnapshotEvent {
    std::vector<SlotSnapshot> slots;
    std::uint64_t tsMs{0};
};

struct OperationEvent {
    std::string operationId;
    std::string type;     // ADD_REEL / REPLACE_REEL / RECOVERY
    std::string state;    // state machine state
    std::string message;
    std::uint64_t tsMs{0};
};

struct ErrorEvent {
    smartcart::domain::errors::ErrorCode code{smartcart::domain::errors::ErrorCode::Unknown};
    std::string message;
    std::uint64_t tsMs{0};
};

} // namespace smartcart::application::dto
