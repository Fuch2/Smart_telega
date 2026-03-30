#pragma once
#include "../../../application/ports/IReelRepository.hpp"
#include "../SqliteConnection.hpp"

namespace smartcart::infrastructure::db {

class ReelRepositorySqlite final
    : public application::ports::IReelRepository {
public:
    explicit ReelRepositorySqlite(SqliteConnection& conn);

    std::vector<domain::ReelRecord>        getAll() override;
    std::vector<domain::ReelRecord>        getByModule(int moduleId) override;
    std::optional<domain::ReelRecord>      getBySlot(int moduleId,
                                                      int slotIndex) override;
    std::vector<domain::ReelRecord>        getActive() override;

    int  add(const domain::ReelRecord& r) override;
    bool markRemoved(int id, const std::string& removedAt) override;
    bool remove(int id) override;

private:
    SqliteConnection& conn_;

    static domain::ReelRecord rowToRecord(sqlite3_stmt* stmt);
    void ensureSchema();
};

} // namespace smartcart::infrastructure::db
