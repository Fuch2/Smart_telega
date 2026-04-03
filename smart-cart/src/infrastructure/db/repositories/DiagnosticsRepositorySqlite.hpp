#pragma once

#include "application/ports/IDiagnosticsRepository.hpp"
#include "infrastructure/db/SqliteConnection.hpp"

namespace smartcart::infrastructure::db {

class DiagnosticsRepositorySqlite final
    : public smartcart::application::ports::IDiagnosticsRepository {
public:
    explicit DiagnosticsRepositorySqlite(SqliteConnection& conn);

    std::vector<smartcart::application::ports::EventLogRecord>
        recentEvents(int limit) override;
    bool resetTestCart(int moduleId) override;

private:
    SqliteConnection& conn_;
};

} // namespace smartcart::infrastructure::db
