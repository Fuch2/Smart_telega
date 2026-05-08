#pragma once

#include "application/ports/IWorkflowRepository.hpp"
#include "infrastructure/db/SqliteConnection.hpp"

struct sqlite3_stmt;

namespace smartcart::infrastructure::db {

class WorkflowRepositorySqlite final
    : public application::ports::IWorkflowRepository
{
public:
    explicit WorkflowRepositorySqlite(SqliteConnection& conn, int moduleId = 1);

    domain::CartWorkflow get() override;
    bool setState(domain::CartWorkflowState state) override;
    bool setCurrentOrder(int orderId,
                         domain::CartWorkflowState state) override;
    bool clearCurrentOrder(domain::CartWorkflowState state) override;

    bool adoptWorkflowFrom(int fromModuleId) override;

private:
    SqliteConnection& conn_;
    int moduleId_{1};

    void ensureSchema();
    void ensureRow();
};

} // namespace smartcart::infrastructure::db
