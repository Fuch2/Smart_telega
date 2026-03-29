#pragma once

#include <optional>
#include <string>

namespace smartcart::application::dto {

struct AddReelCommand {
    std::string barcode;
    std::optional<std::string> preferredSlotId; // MVP: обычно пусто
};

struct ReplaceReelCommand {
    std::string barcode; // barcode катушки, которую нужно заменить на станке
};

struct RemoveReelCommand {
    std::string slotId; // явное указание лотка для изъятия
};

struct ConfirmSlotActionCommand {
    std::string slotId;
};



} // namespace smartcart::application::dto
