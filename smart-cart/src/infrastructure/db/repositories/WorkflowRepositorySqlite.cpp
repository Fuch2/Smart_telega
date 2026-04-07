#include "infrastructure/db/repositories/WorkflowRepositorySqlite.hpp"

#include <sqlite3.h>

#include <stdexcept>
#include <string>

namespace smartcart::infrastructure::db {

namespace domain = smartcart::domain;

namespace {

void throwOnPrepareError(sqlite3* db, int rc, const char* where) {
    if (rc != SQLITE_OK) {
        throw std::runtime_error(std::string(where) + ": " + sqlite3_errmsg(db));
    }
}

} // namespace

WorkflowRepositorySqlite::WorkflowRepositorySqlite(SqliteConnection& conn)
    : conn_(conn)
{
    ensureSchema();
}

void WorkflowRepositorySqlite::ensureSchema() {
    conn_.execute(
        "CREATE TABLE IF NOT EXISTS cart_workflow ("
        "  id               INTEGER PRIMARY KEY CHECK (id = 1),"
        "  current_order_id INTEGER,"
        "  state            TEXT    NOT NULL DEFAULT 'FREE',"
        "  updated_at       DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,"
        "  FOREIGN KEY(current_order_id) REFERENCES orders(id)"
        "    ON DELETE SET NULL ON UPDATE CASCADE"
        ");"
    );

    conn_.execute(
        "INSERT INTO cart_workflow(id, current_order_id, state) "
        "VALUES(1, NULL, 'FREE') "
        "ON CONFLICT(id) DO NOTHING;"
    );
}

domain::CartWorkflow WorkflowRepositorySqlite::get() {
    const char* sql =
        "SELECT current_order_id, state, updated_at "
        "FROM cart_workflow WHERE id = 1;";
    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr),
                        "WorkflowRepositorySqlite::get prepare");

    domain::CartWorkflow workflow;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        if (sqlite3_column_type(stmt, 0) != SQLITE_NULL) {
            workflow.currentOrderId = sqlite3_column_int(stmt, 0);
        }
        workflow.state = domain::cartWorkflowStateFromString(
            reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1)));
        workflow.updatedAt =
            reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
    }

    sqlite3_finalize(stmt);
    return workflow;
}

bool WorkflowRepositorySqlite::setState(domain::CartWorkflowState state) {
    const char* sql =
        "UPDATE cart_workflow "
        "SET state = ?, updated_at = datetime('now') "
        "WHERE id = 1;";
    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr),
                        "WorkflowRepositorySqlite::setState prepare");

    const std::string stateStr{domain::toString(state)};
    sqlite3_bind_text(stmt, 1, stateStr.c_str(), -1, SQLITE_TRANSIENT);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE && sqlite3_changes(conn_.handle()) > 0;
}

bool WorkflowRepositorySqlite::setCurrentOrder(
    int orderId,
    domain::CartWorkflowState state)
{
    const char* sql =
        "UPDATE cart_workflow "
        "SET current_order_id = ?, state = ?, updated_at = datetime('now') "
        "WHERE id = 1;";
    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr),
                        "WorkflowRepositorySqlite::setCurrentOrder prepare");

    const std::string stateStr{domain::toString(state)};
    sqlite3_bind_int(stmt, 1, orderId);
    sqlite3_bind_text(stmt, 2, stateStr.c_str(), -1, SQLITE_TRANSIENT);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE && sqlite3_changes(conn_.handle()) > 0;
}

bool WorkflowRepositorySqlite::clearCurrentOrder(domain::CartWorkflowState state) {
    const char* sql =
        "UPDATE cart_workflow "
        "SET current_order_id = NULL, state = ?, updated_at = datetime('now') "
        "WHERE id = 1;";
    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr),
                        "WorkflowRepositorySqlite::clearCurrentOrder prepare");

    const std::string stateStr{domain::toString(state)};
    sqlite3_bind_text(stmt, 1, stateStr.c_str(), -1, SQLITE_TRANSIENT);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE && sqlite3_changes(conn_.handle()) > 0;
}

} // namespace smartcart::infrastructure::db
