#pragma once
#include "../entities/Slot.hpp"
#include "../entities/Operation.hpp"

namespace smartcart::domain {

// Чистая доменная логика: можно ли начать операцию над слотом?
// Не знает ни о БД, ни о Qt, ни о STM32.
struct SlotStatePolicy {

    // Можно ли положить катушку в слот?
    static bool canPlace(SlotState state) {
        return state == SlotState::Free;
    }

    // Можно ли забрать катушку из слота?
    static bool canRemove(SlotState state) {
        return state == SlotState::Occupied;
    }

    // Можно ли заменить катушку в слоте?
    static bool canReplace(SlotState state) {
        return state == SlotState::Occupied;
    }

    // Переход состояния при начале операции
    static SlotState stateOnOperationStart(OperationType op) {
        (void)op;
        return SlotState::Reserved; // любая операция резервирует слот
    }

    // Переход состояния при завершении операции
    static SlotState stateOnOperationComplete(OperationType op) {
        switch (op) {
            case OperationType::AddReel:
            case OperationType::ReplaceReel:
                return SlotState::Occupied;
            case OperationType::RemoveReel:
                return SlotState::Free;
        }
        return SlotState::Free;
    }

    // Переход состояния при отмене/ошибке
    static SlotState stateOnOperationCancel(OperationType op, SlotState before) {
        (void)op;
        return before; // откат к предыдущему состоянию
    }
};

} // namespace smartcart::domain
