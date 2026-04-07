// ===== src/application/ports/IOperationRepository.hpp =====
// Исправлено: убран finishedAt из updateStatus — генерируется в репозитории
#pragma once

#include "domain/entities/Operation.hpp"

#include <optional>
#include <vector>

namespace smartcart::application::ports {

class IOperationRepository {
public:
    virtual ~IOperationRepository() = default;

    /// Создать операцию, вернуть новый id.
    virtual int  add(const domain::Operation& op) = 0;

    /// Обновить статус. finished_at проставляется репозиторием автоматически
    /// для терминальных статусов (Completed / Cancelled / Failed).
    virtual bool updateStatus(int id, domain::OperationStatus status) = 0;

    /// Назначить слот операции после физического выбора PA1/PA2.
    virtual bool updateSlot(int id, int moduleId, int slotIndex) = 0;

    /// Незавершённые операции — для RecoveryService.
    virtual std::vector<domain::Operation> getUnfinished() = 0;
    virtual std::optional<domain::Operation> getById(int id) = 0;
};

} // namespace smartcart::application::ports
