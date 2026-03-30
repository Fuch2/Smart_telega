// ===== src/infrastructure/db/repositories/ReelRepositorySqlite.hpp =====
#pragma once

#include "../../../application/ports/IReelRepository.hpp"
#include "../SqliteConnection.hpp"

struct sqlite3_stmt;

namespace smartcart::infrastructure::db {

class ReelRepositorySqlite final
    : public application::ports::IReelRepository
{
public:
    explicit ReelRepositorySqlite(SqliteConnection& conn);

    // ── Чтение ──────────────────────────────────────────────────────────
    std::vector<domain::ReelRecord>       getAll()                          override;
    std::vector<domain::ReelRecord>       getByModule(int moduleId)         override;
    std::optional<domain::ReelRecord>     getBySlot(int moduleId,
                                                     int slotIndex)         override;
    std::vector<domain::ReelRecord>       getActive()                       override;
    std::vector<domain::ReelRecord>       getActiveByModule(int moduleId)   override;
    std::optional<domain::ReelRecord>     findActiveByBarcode(
                                              const std::string& barcode)   override;
    bool                                  hasActiveRecord(int moduleId,
                                                          int slotIndex)    override;
    std::vector<domain::Slot>             getSlotStates(int moduleId)       override;

    // ── Запись ──────────────────────────────────────────────────────────
    int  add(const domain::ReelRecord& r)                                   override;
    bool markRemoved(int id, const std::string& removedAt)                  override;
    bool remove(int id)                                                      override;
    bool markRemovedBySlot(int moduleId, int slotIndex)                     override;
    bool setSlotState(int moduleId,
                      int slotIndex,
                      domain::SlotState state)                              override;
    int  addRecord(int moduleId,
                   int slotIndex,
                   const std::string& barcode)                              override;

private:
    SqliteConnection& conn_;

    static domain::ReelRecord rowToRecord(sqlite3_stmt* stmt);
    void ensureSchema();
};

} // namespace smartcart::infrastructure::db
