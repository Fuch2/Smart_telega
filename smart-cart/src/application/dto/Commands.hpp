// ===== src/application/dto/Commands.hpp =====
// Исправлено: убран encoding-мусор в комментариях
#pragma once

#include <optional>
#include <string>

namespace smartcart::application::dto {

struct AddReelCommand {
    std::string        barcode;
    std::optional<int> preferredSlotIndex; // MVP: обычно пусто
};

struct ReplaceReelCommand {
    std::string barcode; // штрихкод новой катушки
};

struct RemoveReelCommand {
    int moduleId  = 0;
    int slotIndex = 0;
};

struct ConfirmSlotActionCommand {
    int moduleId  = 0;
    int slotIndex = 0;
};

} // namespace smartcart::application::dto
