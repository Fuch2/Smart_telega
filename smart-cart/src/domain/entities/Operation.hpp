#pragma once
#include <string>
#include <cstdint>

namespace smartcart::domain {

enum class OperationType : std::uint8_t {
    AddReel     = 0,
    RemoveReel  = 1,
    ReplaceReel = 2,
};

enum class OperationStatus : std::uint8_t {
    InProgress = 0,
    Completed  = 1,
    Cancelled  = 2,
    Failed     = 3,
};

inline std::string_view toString(OperationType t) {
    switch (t) {
        case OperationType::AddReel:     return "ADD_REEL";
        case OperationType::RemoveReel:  return "REMOVE_REEL";
        case OperationType::ReplaceReel: return "REPLACE_REEL";
    }
    return "UNKNOWN";
}

inline std::string_view toString(OperationStatus s) {
    switch (s) {
        case OperationStatus::InProgress: return "IN_PROGRESS";
        case OperationStatus::Completed:  return "COMPLETED";
        case OperationStatus::Cancelled:  return "CANCELLED";
        case OperationStatus::Failed:     return "FAILED";
    }
    return "UNKNOWN";
}

struct Operation {
    int             id{0};
    OperationType   type{OperationType::AddReel};
    OperationStatus status{OperationStatus::InProgress};
    int             moduleId{0};
    int             slotIndex{0};
    std::string     barcode;
    std::string     startedAt;   // ISO-8601
    std::string     finishedAt;  // пусто, если InProgress
};

} // namespace smartcart::domain
