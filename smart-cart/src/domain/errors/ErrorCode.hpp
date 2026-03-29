#pragma once

#include <string_view>

namespace smartcart::domain::errors {

enum class ErrorCode {
    None = 0,
    InvalidConfig,
    InvalidBarcode,
    SlotNotFound,
    SlotOccupied,
    NoFreeSlot,
    ReelNotFound,
    WrongSlotInteraction,
    Stm32CommunicationError,
    RfidReadError,
    ScannerReadError,
    PersistenceError,
    RecoveryFailed,
    Timeout,
    Unknown
};

inline std::string_view toString(ErrorCode code) {
    switch (code) {
        case ErrorCode::None: return "None";
        case ErrorCode::InvalidConfig: return "InvalidConfig";
        case ErrorCode::InvalidBarcode: return "InvalidBarcode";
        case ErrorCode::SlotNotFound: return "SlotNotFound";
        case ErrorCode::SlotOccupied: return "SlotOccupied";
        case ErrorCode::NoFreeSlot: return "NoFreeSlot";
        case ErrorCode::ReelNotFound: return "ReelNotFound";
        case ErrorCode::WrongSlotInteraction: return "WrongSlotInteraction";
        case ErrorCode::Stm32CommunicationError: return "Stm32CommunicationError";
        case ErrorCode::RfidReadError: return "RfidReadError";
        case ErrorCode::ScannerReadError: return "ScannerReadError";
        case ErrorCode::PersistenceError: return "PersistenceError";
        case ErrorCode::RecoveryFailed: return "RecoveryFailed";
        case ErrorCode::Timeout: return "Timeout";
        case ErrorCode::Unknown: return "Unknown";
    }
    return "Unknown";
}

} // namespace smartcart::domain::errors
