// ===== src/application/services/RecoveryService.cpp =====
// Исправлено: updateStatus без finishedAt
#include "application/services/RecoveryService.hpp"
#include "infrastructure/hw/stm32/Protocol.hpp"

#include <optional>
#include <stdexcept>

namespace smartcart::application::services {

namespace domain = smartcart::domain;
namespace stm32  = smartcart::infrastructure::hw::stm32;

RecoveryService::RecoveryService(
    ports::IOperationRepository& opRepo,
    ports::IReelRepository&      reelRepo,
    ports::IStm32Link&           link,
    RecoveryConfig               config)
    : opRepo_(opRepo)
    , reelRepo_(reelRepo)
    , link_(link)
    , config_(std::move(config))
{}

std::vector<bool> RecoveryService::getPhysicalSnapshot() {
    auto requestSnapshot = [this](stm32::CommandId cmdId)
        -> std::optional<std::vector<bool>>
    {
        stm32::Frame cmd;
        cmd.type  = stm32::FrameType::Cmd;
        cmd.cmdId = cmdId;

        auto resp = link_.sendCommand(cmd);
        if (!resp.has_value() ||
            resp->type != stm32::FrameType::Resp ||
            resp->payload.size() < 3)
        {
            return std::nullopt;
        }

        std::vector<bool> result(config_.slotCount, false);
        for (int i = 0; i < config_.slotCount; ++i) {
            const int byteIdx = i / 8;
            const int bitIdx  = i % 8;
            result[i] = (resp->payload[byteIdx] >> bitIdx) & 0x01;
        }
        if (config_.slotCount > 11) {
            result[11] = false;
        }
        return result;
    };

    if (auto snapshot = requestSnapshot(stm32::CommandId::GetSwitchSnapshot)) {
        return *snapshot;
    }
    if (auto snapshot = requestSnapshot(static_cast<stm32::CommandId>(0x04))) {
        return *snapshot;
    }

    throw std::runtime_error(
        "RecoveryService: не удалось получить снимок слотов");
}

void RecoveryService::recoverOperation(
    const domain::Operation& op,
    const std::vector<bool>& physical)
{
    const int idx = op.slotIndex - 1;
    if (idx < 0 || idx >= static_cast<int>(physical.size())) {
        opRepo_.updateStatus(op.id, domain::OperationStatus::Failed);
        return;
    }

    const bool physOccupied = physical[idx];

    switch (op.type) {

        case domain::OperationType::AddReel:
            if (physOccupied) {
                reelRepo_.setSlotState(config_.moduleId, op.slotIndex,
                                       domain::SlotState::Occupied);
                if (!reelRepo_.hasActiveRecord(config_.moduleId, op.slotIndex))
                    reelRepo_.addRecord(config_.moduleId, op.slotIndex, op.barcode);
                opRepo_.updateStatus(op.id, domain::OperationStatus::Completed);
            } else {
                reelRepo_.setSlotState(config_.moduleId, op.slotIndex,
                                       domain::SlotState::Free);
                opRepo_.updateStatus(op.id, domain::OperationStatus::Cancelled);
            }
            break;

        case domain::OperationType::RemoveReel:
            if (!physOccupied) {
                reelRepo_.markRemovedBySlot(config_.moduleId, op.slotIndex);
                reelRepo_.setSlotState(config_.moduleId, op.slotIndex,
                                       domain::SlotState::Free);
                opRepo_.updateStatus(op.id, domain::OperationStatus::Completed);
            } else {
                reelRepo_.setSlotState(config_.moduleId, op.slotIndex,
                                       domain::SlotState::Occupied);
                opRepo_.updateStatus(op.id, domain::OperationStatus::Cancelled);
            }
            break;

        case domain::OperationType::ReplaceReel: {
            const bool hasActive =
                reelRepo_.hasActiveRecord(config_.moduleId, op.slotIndex);

            if (physOccupied && hasActive) {
                opRepo_.updateStatus(op.id, domain::OperationStatus::Completed);
            } else if (physOccupied && !hasActive) {
                reelRepo_.addRecord(config_.moduleId, op.slotIndex, op.barcode);
                reelRepo_.setSlotState(config_.moduleId, op.slotIndex,
                                       domain::SlotState::Occupied);
                opRepo_.updateStatus(op.id, domain::OperationStatus::Completed);
            } else {
                reelRepo_.setSlotState(config_.moduleId, op.slotIndex,
                                       domain::SlotState::Error);
                opRepo_.updateStatus(op.id, domain::OperationStatus::Failed);
            }
            break;
        }
    }
}

bool RecoveryService::run() {
    std::vector<bool> physical;
    try {
        physical = getPhysicalSnapshot();
    } catch (const std::exception&) {
        return false;
    }

    const auto unfinished = opRepo_.getUnfinished();
    for (const auto& op : unfinished) {
        try {
            recoverOperation(op, physical);
        } catch (const std::exception&) {
            return false;
        }
    }

    return true;
}

} // namespace smartcart::application::services
