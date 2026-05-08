#pragma once

#include "domain/entities/CartWorkflow.hpp"

namespace smartcart::application::ports {

class IWorkflowRepository {
public:
    virtual ~IWorkflowRepository() = default;

    virtual domain::CartWorkflow get() = 0;
    virtual bool setState(domain::CartWorkflowState state) = 0;
    virtual bool setCurrentOrder(int orderId,
                                 domain::CartWorkflowState state) = 0;
    virtual bool clearCurrentOrder(domain::CartWorkflowState state) = 0;

    /// Переносит состояние воркфлоу (этап + заказ) из fromModuleId в текущий
    /// модуль и сбрасывает fromModuleId в FREE.
    /// Ничего не делает если текущий модуль уже не FREE.
    virtual bool adoptWorkflowFrom(int fromModuleId) = 0;
};

} // namespace smartcart::application::ports
