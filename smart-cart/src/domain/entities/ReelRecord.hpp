#pragma once
#include <string>
#include <cstdint>

namespace smartcart::domain {

// Запись о катушке, размещённой в слоте
struct ReelRecord {
    int         id{0};
    std::string barcode;      // отсканированный штрихкод
    int         moduleId{0};
    int         slotIndex{0}; // 1..N
    std::string placedAt;     // ISO-8601 timestamp
    std::string removedAt;    // пусто, если катушка ещё в слоте
};

} // namespace smartcart::domain
