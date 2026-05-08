#pragma once

// Общие утилиты для обработки snapshot-фрейма от STM32.
// Выделены в отдельный заголовок, чтобы StartupService и Stm32PollingService
// использовали единый источник истины и не дублировали логику разбора.

#include <cstdint>
#include <optional>
#include <vector>

namespace smartcart::application::services::snapshot_util {

/// Распаковывает payload фрейма GetSwitchSnapshot в булев вектор размером
/// slotCount. Каждый бит payload соответствует одному каналу: бит i в байте
/// (i / 8), смещение (i % 8). Возвращает пустой вектор, если payload короче
/// минимального размера (3 байта).
inline std::vector<bool> parseSnapshotPayload(
    const std::vector<std::uint8_t>& payload, int slotCount)
{
    std::vector<bool> result(static_cast<std::size_t>(slotCount), false);
    if (payload.size() < 3) {
        return {};
    }

    for (int channel = 0; channel < slotCount; ++channel) {
        const int byteIdx = channel / 8;
        const int bitIdx  = channel % 8;
        result[channel] = ((payload[byteIdx] >> bitIdx) & 0x01) != 0;
    }
    return result;
}

/// Преобразует индекс канала STM32 (0-based) в индекс слота (1-based).
/// - Если channel вне диапазона [0, slotCount) → std::nullopt.
/// - Если channelToSlotMap содержит явное соответствие — берётся оно (и
///   валидируется на 1..slotCount).
/// - Иначе — fallback к (channel + 1) для случая идентичного маппинга.
inline std::optional<int> slotIndexForChannel(
    int channel,
    int slotCount,
    const std::vector<int>& channelToSlotMap)
{
    if (channel < 0 || channel >= slotCount) {
        return std::nullopt;
    }
    if (channel < static_cast<int>(channelToSlotMap.size())) {
        const int slotIndex = channelToSlotMap[channel];
        if (slotIndex > 0 && slotIndex <= slotCount) {
            return slotIndex;
        }
        return std::nullopt;
    }
    return channel + 1;
}

} // namespace smartcart::application::services::snapshot_util
