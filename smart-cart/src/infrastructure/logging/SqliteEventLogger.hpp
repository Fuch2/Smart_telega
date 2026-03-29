#pragma once

#include <memory>
#include <string>
#include "../../application/ports/IEventLogger.hpp"

struct sqlite3;

namespace smartcart::infrastructure::logging {

class SqliteEventLogger final : public smartcart::application::ports::IEventLogger {
public:
    explicit SqliteEventLogger(sqlite3* db);
    void log(std::string_view level, std::string_view code, std::string_view message) override;

private:
    sqlite3* db_{nullptr};
};

} // namespace smartcart::infrastructure::logging
