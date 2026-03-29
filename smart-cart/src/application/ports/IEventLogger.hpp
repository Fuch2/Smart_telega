#pragma once
#include <string>
#include <string_view>

namespace smartcart::application::ports {

class IEventLogger {
public:
    virtual ~IEventLogger() = default;
    virtual void log(std::string_view level,
                     std::string_view code,
                     std::string_view message) = 0;
};

} // namespace smartcart::application::ports
