#pragma once
#include <optional>
#include <string>
#include <vector>

struct sqlite3;
namespace smartcart::infrastructure::db::repositories {

struct ReelRow {
    int id{};
    std::string barcode;
    std::string slotId;
    std::string status;
};

class ReelRepositorySqlite {
public:
    explicit ReelRepositorySqlite(sqlite3* db);

    int create(const std::string& barcode, const std::string& slotId, const std::string& status);
    std::optional<ReelRow> findById(int id);
    std::optional<ReelRow> findActiveByBarcode(const std::string& barcode);
    std::optional<ReelRow> findBySlot(const std::string& slotId);
    std::vector<ReelRow> listByBarcode(const std::string& barcode);
    void updateStatus(int id, const std::string& status);
    void moveToSlot(int id, const std::string& slotId);

private:
    sqlite3* db_{nullptr};
};

} // namespace smartcart::infrastructure::db::repositories
