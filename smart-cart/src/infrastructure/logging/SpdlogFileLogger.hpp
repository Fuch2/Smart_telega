// ===== src/infrastructure/logging/SpdlogFileLogger.hpp =====
#pragma once

#include "application/ports/IEventLogger.hpp"
#include <memory>
#include <spdlog/logger.h>

namespace smartcart::infrastructure::logging {

/// Адаптер IEventLogger → spdlog (пишет в rotating file).
class SpdlogFileLogger final
    : public application::ports::IEventLogger
{
public:
    explicit SpdlogFileLogger(std::shared_ptr<spdlog::logger> logger);

    void log(std::string_view level,
             std::string_view code,
             std::string_view message) override;

private:
    std::shared_ptr<spdlog::logger> logger_;
};

} // namespace smartcart::infrastructure::logging
