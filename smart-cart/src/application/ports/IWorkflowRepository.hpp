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
};

} // namespace smartcart::application::ports
