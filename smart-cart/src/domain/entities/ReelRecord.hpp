// ===== src/domain/entities/ReelRecord.hpp =====
#pragma once

#include <optional>
#include <string>
#include <cstdint>

namespace smartcart::domain {

/// Запись о катушке, размещённой в слоте.
/// removedAt == nullopt  →  катушка ещё в слоте.
/// removedAt == строка   →  ISO-8601 timestamp снятия.
struct ReelRecord {
    int                      id{0};
    std::string              barcode;
    int                      moduleId{0};
    int                      slotIndex{0};   // 1..N
    std::string              placedAt;       // ISO-8601
    std::optional<std::string> removedAt;   // nullopt = активна
};

} // namespace smartcart::domain
