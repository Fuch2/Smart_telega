#pragma once

#include "domain/entities/ModuleInfo.hpp"
#include "domain/entities/Slot.hpp"

#include <optional>
#include <string>
#include <vector>

namespace smartcart::application::ports {

class IModuleRepository {
public:
    virtual ~IModuleRepository() = default;

    virtual std::vector<domain::ModuleInfo> getAll() = 0;
    virtual std::optional<domain::ModuleInfo> getById(int id) = 0;
    virtual bool existsBySerial(const std::string& serial,
                                int exceptId = 0) = 0;

    virtual int  add(const domain::ModuleInfo& m) = 0;
    virtual bool update(const domain::ModuleInfo& m) = 0;
    virtual bool remove(int id) = 0;

    /// Обновить состояние конкретного слота модуля
    virtual bool updateSlotState(int moduleId,
                                 int slotIndex,
                                 domain::SlotState state) = 0;
};

} // namespace smartcart::application::ports
