#pragma once
#include "../../../application/ports/IOperationRepository.hpp"
#include "../SqliteConnection.hpp"
struct sqlite3_stmt;

namespace smartcart::infrastructure::db {

class OperationRepositorySqlite final
    : public application::ports::IOperationRepository {
public:
    explicit OperationRepositorySqlite(SqliteConnection& conn);

    int  add(const domain::Operation& op) override;
    bool updateStatus(int id,
                      domain::OperationStatus status,
                      const std::string& finishedAt) override;

    std::vector<domain::Operation>        getUnfinished() override;
    std::optional<domain::Operation>      getById(int id) override;

private:
    SqliteConnection& conn_;

    static domain::Operation rowToOp(sqlite3_stmt* stmt);
    void ensureSchema();
};

} // namespace smartcart::infrastructure::db
