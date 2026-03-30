// ===== src/infrastructure/time/SteadyClock.hpp =====
#pragma once

#include "IClock.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace smartcart::infrastructure::time {

class SteadyClock final : public IClock {
public:
    std::chrono::steady_clock::time_point now() const override {
        return std::chrono::steady_clock::now();
    }

    std::string nowIso() const override {
        const auto t  = std::chrono::system_clock::now();
        const auto tt = std::chrono::system_clock::to_time_t(t);
        std::ostringstream oss;
        oss << std::put_time(std::gmtime(&tt), "%Y-%m-%dT%H:%M:%SZ");
        return oss.str();
    }
};

} // namespace smartcart::infrastructure::time
