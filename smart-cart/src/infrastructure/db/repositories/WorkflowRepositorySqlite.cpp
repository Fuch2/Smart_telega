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

WorkflowRepositorySqlite::WorkflowRepositorySqlite(SqliteConnection& conn,
                                                   int moduleId)
    : conn_(conn)
    , moduleId_(moduleId)
{
    ensureSchema();
    ensureRow();
}

void WorkflowRepositorySqlite::ensureSchema() {
    conn_.execute(
        "CREATE TABLE IF NOT EXISTS cart_workflow ("
        "  module_id        INTEGER PRIMARY KEY,"
        "  current_order_id INTEGER,"
        "  state            TEXT    NOT NULL DEFAULT 'FREE',"
        "  updated_at       DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,"
        "  FOREIGN KEY(current_order_id) REFERENCES orders(id)"
        "    ON DELETE SET NULL ON UPDATE CASCADE"
        ");"
    );
}

void WorkflowRepositorySqlite::ensureRow() {
    conn_.execute(
        "INSERT INTO cart_workflow(module_id, current_order_id, state) "
        "VALUES(" + std::to_string(moduleId_) + ", NULL, 'FREE') "
        "ON CONFLICT(module_id) DO NOTHING;"
    );
}

domain::CartWorkflow WorkflowRepositorySqlite::get() {
    const char* sql =
        "SELECT current_order_id, state, updated_at "
        "FROM cart_workflow WHERE module_id = ?;";
    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr),
                        "WorkflowRepositorySqlite::get prepare");

    sqlite3_bind_int(stmt, 1, moduleId_);

    domain::CartWorkflow workflow;
    workflow.moduleId = moduleId_;
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
        "WHERE module_id = ?;";
    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr),
                        "WorkflowRepositorySqlite::setState prepare");

    const std::string stateStr{domain::toString(state)};
    sqlite3_bind_text(stmt, 1, stateStr.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 2, moduleId_);

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
        "WHERE module_id = ?;";
    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr),
                        "WorkflowRepositorySqlite::setCurrentOrder prepare");

    const std::string stateStr{domain::toString(state)};
    sqlite3_bind_int(stmt, 1, orderId);
    sqlite3_bind_text(stmt, 2, stateStr.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 3, moduleId_);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE && sqlite3_changes(conn_.handle()) > 0;
}

bool WorkflowRepositorySqlite::clearCurrentOrder(domain::CartWorkflowState state) {
    const char* sql =
        "UPDATE cart_workflow "
        "SET current_order_id = NULL, state = ?, updated_at = datetime('now') "
        "WHERE module_id = ?;";
    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sql, -1, &stmt, nullptr),
                        "WorkflowRepositorySqlite::clearCurrentOrder prepare");

    const std::string stateStr{domain::toString(state)};
    sqlite3_bind_text(stmt, 1, stateStr.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 2, moduleId_);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE && sqlite3_changes(conn_.handle()) > 0;
}

bool WorkflowRepositorySqlite::adoptWorkflowFrom(int fromModuleId) {
    // Nothing to adopt if we're already not FREE
    {
        auto current = get();
        if (current.state != domain::CartWorkflowState::Free) {
            return true;
        }
    }

    // Copy current_order_id AND state from fromModuleId → moduleId_.
    // Preserving state ensures workflow continuity (e.g. READY_FOR_LINE on FEEDER
    // is inherited by REEL, which can then immediately press startIssuingToLine).
    // Then reset fromModuleId back to FREE / NULL.
    const char* sqlCopy =
        "UPDATE cart_workflow "
        "SET current_order_id = (SELECT current_order_id FROM cart_workflow WHERE module_id = ?),"
        "    state             = (SELECT state FROM cart_workflow WHERE module_id = ?),"
        "    updated_at        = datetime('now') "
        "WHERE module_id = ?;";

    sqlite3_stmt* stmt = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sqlCopy, -1, &stmt, nullptr),
                        "WorkflowRepositorySqlite::adoptWorkflowFrom copy prepare");

    sqlite3_bind_int(stmt, 1, fromModuleId);
    sqlite3_bind_int(stmt, 2, fromModuleId);
    sqlite3_bind_int(stmt, 3, moduleId_);

    const int rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    if (rc != SQLITE_DONE) {
        return false;
    }

    // Reset the source module to FREE / NULL
    const char* sqlReset =
        "UPDATE cart_workflow "
        "SET current_order_id = NULL, state = 'FREE', updated_at = datetime('now') "
        "WHERE module_id = ?;";

    sqlite3_stmt* stmt2 = nullptr;
    throwOnPrepareError(conn_.handle(),
                        sqlite3_prepare_v2(conn_.handle(), sqlReset, -1, &stmt2, nullptr),
                        "WorkflowRepositorySqlite::adoptWorkflowFrom reset prepare");

    sqlite3_bind_int(stmt2, 1, fromModuleId);

    const int rc2 = sqlite3_step(stmt2);
    sqlite3_finalize(stmt2);
    return rc2 == SQLITE_DONE;
}

} // namespace smartcart::infrastructure::db
