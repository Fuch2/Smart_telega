#pragma once
#include "../../domain/entities/Operation.hpp"
#include <vector>
#include <optional>

namespace smartcart::application::ports {

class IOperationRepository {
public:
    virtual ~IOperationRepository() = default;

    virtual int  add(const domain::Operation& op) = 0;
    virtual bool updateStatus(int id,
                              domain::OperationStatus status,
                              const std::string& finishedAt) = 0;

    // Незавершённые операции — для RecoveryService
    virtual std::vector<domain::Operation> getUnfinished() = 0;
    virtual std::optional<domain::Operation> getById(int id) = 0;
};

} // namespace smartcart::application::ports
