#pragma once

#include <string>
#include <vector>

namespace smartcart::application::ports {

struct EventLogRecord {
    int id{0};
    std::string ts;
    std::string level;
    std::string code;
    std::string message;
};

class IDiagnosticsRepository {
public:
    virtual ~IDiagnosticsRepository() = default;

    virtual std::vector<EventLogRecord> recentEvents(int limit) = 0;
    virtual bool resetTestCart(int moduleId) = 0;
};

} // namespace smartcart::application::ports
