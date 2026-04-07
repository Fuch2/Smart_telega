#pragma once

#include "application/ports/IWorkflowRepository.hpp"
#include "infrastructure/db/SqliteConnection.hpp"

struct sqlite3_stmt;

namespace smartcart::infrastructure::db {

class WorkflowRepositorySqlite final
    : public application::ports::IWorkflowRepository
{
public:
    explicit WorkflowRepositorySqlite(SqliteConnection& conn);

    domain::CartWorkflow get() override;
    bool setState(domain::CartWorkflowState state) override;
    bool setCurrentOrder(int orderId,
                         domain::CartWorkflowState state) override;
    bool clearCurrentOrder(domain::CartWorkflowState state) override;

private:
    SqliteConnection& conn_;

    void ensureSchema();
};

} // namespace smartcart::infrastructure::db
