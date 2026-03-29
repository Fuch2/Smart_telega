#pragma once
#include <optional>
#include <string>
#include <vector>

struct sqlite3;
namespace smartcart::infrastructure::db::repositories {

struct OperationRow {
    int id{};
    std::string opType;
    std::string state;
    std::string targetBarcode;
    std::string payloadJson;
    bool finished{false};
};

class OperationRepositorySqlite {
public:
    explicit OperationRepositorySqlite(sqlite3* db);

    int create(const std::string& opType, const std::string& state,
               const std::string& targetBarcode, const std::string& payloadJson);

    void updateState(int id, const std::string& state, const std::string& payloadJson);
    void finish(int id, const std::string& finalState, const std::string& payloadJson);

    std::optional<OperationRow> findById(int id);
    std::vector<OperationRow> listUnfinished();

private:
    sqlite3* db_{nullptr};
};

} // namespace smartcart::infrastructure::db::repositories
