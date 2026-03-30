#pragma once
#include "../../domain/entities/ReelRecord.hpp"
#include <vector>
#include <optional>
#include <string>

namespace smartcart::application::ports {

class IReelRepository {
public:
    virtual ~IReelRepository() = default;

    virtual std::vector<domain::ReelRecord> getAll() = 0;
    virtual std::vector<domain::ReelRecord> getByModule(int moduleId) = 0;
    virtual std::optional<domain::ReelRecord> getBySlot(int moduleId,
                                                         int slotIndex) = 0;

    // Возвращает активные записи (removedAt пустой)
    virtual std::vector<domain::ReelRecord> getActive() = 0;

    virtual int  add(const domain::ReelRecord& r) = 0;
    virtual bool markRemoved(int id, const std::string& removedAt) = 0;
    virtual bool remove(int id) = 0;
};

} // namespace smartcart::application::ports
