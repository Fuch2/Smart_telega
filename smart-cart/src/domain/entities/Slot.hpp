#pragma once
#include <cstdint>
#include <string>

namespace smartcart::domain {

enum class SlotState : std::uint8_t {
    Free     = 0,  // физически пуст
    Occupied = 1,  // катушка установлена
    Reserved = 2,  // зарезервирован (идёт операция)
    Error    = 3,  // ошибка датчика
};

inline std::string_view toString(SlotState s) {
    switch (s) {
        case SlotState::Free:     return "FREE";
        case SlotState::Occupied: return "OCCUPIED";
        case SlotState::Reserved: return "RESERVED";
        case SlotState::Error:    return "ERROR";
    }
    return "UNKNOWN";
}

struct Slot {
    int      moduleId{0};
    int      slotIndex{0};   // 1..N
    int      ledIndex{0};    // физический индекс LED на STM32
    SlotState state{SlotState::Free};
};

} // namespace smartcart::domain
