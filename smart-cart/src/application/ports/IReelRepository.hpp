#pragma once

#include "domain/entities/ReelRecord.hpp"
#include "domain/entities/Slot.hpp"

#include <optional>
#include <string>
#include <vector>

namespace smartcart::application::ports {

class IReelRepository {
public:
    virtual ~IReelRepository() = default;

    // ── Чтение ────────────────────────────────────────────────────────────
    virtual std::vector<domain::ReelRecord> getAll() = 0;
    virtual std::vector<domain::ReelRecord> getByModule(int moduleId) = 0;
    virtual std::optional<domain::ReelRecord> getBySlot(int moduleId,
                                                         int slotIndex) = 0;

    /// Активные записи (removedAt пустой) по всем модулям
    virtual std::vector<domain::ReelRecord> getActive() = 0;

    /// Активные записи конкретного модуля
    virtual std::vector<domain::ReelRecord> getActiveByModule(int moduleId) = 0;

    /// Найти активную запись по штрихкоду
    virtual std::optional<domain::ReelRecord> findActiveByBarcode(
        const std::string& barcode) = 0;

    /// Проверить, есть ли активная запись в слоте
    virtual bool hasActiveRecord(int moduleId, int slotIndex) = 0;

    /// Получить состояния всех слотов модуля
    virtual std::vector<domain::Slot> getSlotStates(int moduleId) = 0;

    // ── Запись ────────────────────────────────────────────────────────────
    virtual int  add(const domain::ReelRecord& r) = 0;
    virtual bool markRemoved(int id, const std::string& removedAt) = 0;
    virtual bool remove(int id) = 0;

    /// Пометить слот как удалённый по moduleId + slotIndex (текущее время)
    virtual bool markRemovedBySlot(int moduleId, int slotIndex) = 0;

    /// Обновить состояние слота
    virtual bool setSlotState(int moduleId,
                              int slotIndex,
                              domain::SlotState state) = 0;

    /// Добавить запись о катушке (краткая форма)
    virtual int addRecord(int moduleId,
                          int slotIndex,
                          const std::string& barcode) = 0;
};

} // namespace smartcart::application::ports
