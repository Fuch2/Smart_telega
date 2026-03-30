#pragma once
#include <vector>
#include <stdexcept>

namespace smartcart::domain {

// Отображение slotIndex (1..N) → ledIndex (0..M-1) на STM32
// Загружается из config/led_mapping.json
class LedMappingPolicy {
public:
    // map[i] = ledIndex для slotIndex = i+1
    explicit LedMappingPolicy(std::vector<int> map)
        : map_(std::move(map)) {}

    // Создаёт тождественное отображение: slot i → led i-1
    static LedMappingPolicy identity(int slotCount) {
        std::vector<int> m(slotCount);
        for (int i = 0; i < slotCount; ++i) m[i] = i;
        return LedMappingPolicy{std::move(m)};
    }

    int toLed(int slotIndex) const {
        if (slotIndex < 1 || slotIndex > static_cast<int>(map_.size()))
            throw std::out_of_range("slotIndex out of range");
        return map_[slotIndex - 1];
    }

    int slotCount() const {
        return static_cast<int>(map_.size());
    }

    const std::vector<int>& raw() const { return map_; }

private:
    std::vector<int> map_;
};

} // namespace smartcart::domain
