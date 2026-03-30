// ===== src/infrastructure/time/IClock.hpp =====
#pragma once

#include <chrono>
#include <string>

namespace smartcart::infrastructure::time {

class IClock {
public:
    virtual ~IClock() = default;

    /// Текущее время в виде строки ISO 8601 (UTC).
    virtual std::string nowIso() const = 0;

    /// Монотонное время (для измерения интервалов).
    virtual std::chrono::steady_clock::time_point now() const = 0;
};

} // namespace smartcart::infrastructure::time
