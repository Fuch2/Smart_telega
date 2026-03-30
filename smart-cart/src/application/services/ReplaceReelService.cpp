// ===== src/application/services/ReplaceReelService.cpp =====
// Исправлено: updateStatus без finishedAt
#include "application/services/ReplaceReelService.hpp"
#include "infrastructure/hw/stm32/Protocol.hpp"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace smartcart::application::services {

namespace domain = smartcart::domain;
namespace stm32  = smartcart::infrastructure::hw::stm32;

ReplaceReelService::ReplaceReelService(
    ports::IStm32Link&           link,
    ports::IReelRepository&      reelRepo,
    ports::IOperationRepository& opRepo,
    ReplaceReelConfig            config)
    : link_(link)
    , reelRepo_(reelRepo)
    , opRepo_(opRepo)
    , config_(std::move(config))
{}

bool ReplaceReelService::isValidBarcode(const std::string& barcode) {
    return !barcode.empty() && barcode.size() <= 64;
}

void ReplaceReelService::setSlotLed(int slotIndex,
                                    uint8_t r, uint8_t g, uint8_t b) {
    const int idx      = slotIndex - 1;
    const int ledIndex = (idx < static_cast<int>(config_.slotToLedMap.size()))
                         ? config_.slotToLedMap[idx]
                         : idx * 2;

    stm32::Frame setCmd;
    setCmd.type    = stm32::FrameType::Cmd;
    setCmd.cmdId   = stm32::CommandId::LedSetSlot;
    setCmd.payload = { static_cast<uint8_t>(ledIndex), r, g, b };
    link_.sendCommand(setCmd);

    stm32::Frame applyCmd;
    applyCmd.type  = stm32::FrameType::Cmd;
    applyCmd.cmdId = stm32::CommandId::LedApply;
    link_.sendCommand(applyCmd);

    if (onHighlight_)
        onHighlight_(slotIndex, RgbColor{r, g, b});
}

void ReplaceReelService::clearAllLeds() {
    stm32::Frame clearCmd;
    clearCmd.type  = stm32::FrameType::Cmd;
    clearCmd.cmdId = stm32::CommandId::LedClearAll;
    link_.sendCommand(clearCmd);

    stm32::Frame applyCmd;
    applyCmd.type  = stm32::FrameType::Cmd;
    applyCmd.cmdId = stm32::CommandId::LedApply;
    link_.sendCommand(applyCmd);
}

bool ReplaceReelService::waitForSlotEvent(int slotIndex,
                                          bool expectedOccupied) {
    std::mutex              mtx;
    std::condition_variable cv;
    bool                    confirmed = false;

    link_.setEventCallback([&](const stm32::Frame& evt) {
        if (evt.type  == stm32::FrameType::Evt &&
            evt.cmdId == stm32::CommandId::EvtSwitchChanged &&
            evt.payload.size() >= 2)
        {
            const int  evtSlot     = evt.payload[0] + 1;
            const bool evtOccupied = evt.payload[1] == 0x01;

            if (evtSlot == slotIndex && evtOccupied == expectedOccupied) {
                std::lock_guard lock(mtx);
                confirmed = true;
                cv.notify_one();
            }
        }
    });

    std::unique_lock lock(mtx);
    const bool ok = cv.wait_for(
        lock,
        std::chrono::milliseconds(config_.stableConfirmMs),
        [&] { return confirmed || cancelled_.load(); }
    );

    link_.setEventCallback(nullptr);
    return ok && !cancelled_;
}

int ReplaceReelService::start(const std::string& newBarcode) {
    cancelled_ = false;

    if (!isValidBarcode(newBarcode)) {
        if (onError_) onError_(domain::ErrorCode::InvalidBarcode,
                               "Некорректный штрихкод: " + newBarcode);
        return -1;
    }

    const auto existingReel = reelRepo_.findActiveByBarcode(newBarcode);
    if (!existingReel.has_value()) {
        if (onError_) onError_(domain::ErrorCode::ReelNotFound,
                               "Катушка не найдена: " + newBarcode);
        return -1;
    }

    const int slotIndex = existingReel->slotIndex;

    domain::Operation op{};
    op.type      = domain::OperationType::ReplaceReel;
    op.status    = domain::OperationStatus::InProgress;
    op.moduleId  = config_.moduleId;
    op.slotIndex = slotIndex;
    op.barcode   = newBarcode;

    const int opId = opRepo_.add(op);

    // ← захват по значению
    std::thread([this, opId, slotIndex, newBarcode]() {

        setSlotLed(slotIndex, 255, 0, 0);

        if (!waitForSlotEvent(slotIndex, false)) {
            clearAllLeds();
            reelRepo_.setSlotState(config_.moduleId, slotIndex,
                                   domain::SlotState::Occupied);
            opRepo_.updateStatus(opId, domain::OperationStatus::Cancelled);
            if (onComplete_) onComplete_(opId, domain::OperationStatus::Cancelled);
            return;
        }

        reelRepo_.setSlotState(config_.moduleId, slotIndex,
                               domain::SlotState::Reserved);
        setSlotLed(slotIndex, 0, 255, 0);

        if (!waitForSlotEvent(slotIndex, true)) {
            clearAllLeds();
            reelRepo_.setSlotState(config_.moduleId, slotIndex,
                                   domain::SlotState::Free);
            opRepo_.updateStatus(opId, domain::OperationStatus::Cancelled);
            if (onComplete_) onComplete_(opId, domain::OperationStatus::Cancelled);
            return;
        }

        reelRepo_.markRemovedBySlot(config_.moduleId, slotIndex);
        reelRepo_.addRecord(config_.moduleId, slotIndex, newBarcode);
        reelRepo_.setSlotState(config_.moduleId, slotIndex,
                               domain::SlotState::Occupied);
        opRepo_.updateStatus(opId, domain::OperationStatus::Completed);
        clearAllLeds();

        if (onComplete_) onComplete_(opId, domain::OperationStatus::Completed);

    }).detach();

    return opId;
}

void ReplaceReelService::cancel() {
    cancelled_ = true;
}

} // namespace smartcart::application::services
