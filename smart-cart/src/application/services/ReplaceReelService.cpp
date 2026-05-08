// ===== src/application/services/ReplaceReelService.cpp =====
#include "application/services/ReplaceReelService.hpp"
#include "infrastructure/hw/stm32/Protocol.hpp"

#include <chrono>
#include <condition_variable>
#include <memory>
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

ReplaceReelService::~ReplaceReelService() {
    cancelAndJoinWorker();
}

// Будит и дожидается завершения текущей операции замены.
// После возврата метода рабочий поток гарантированно не имеет доступа к
// членам объекта — это корректно для деструктора и для повторного start().
void ReplaceReelService::cancelAndJoinWorker() {
    cancelled_.store(true);
    if (waitCv_ && waitMtx_) {
        std::lock_guard lk(*waitMtx_);
        waitCv_->notify_all();
    }
    if (workerThread_.joinable()) {
        workerThread_.join();
    }
}

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

// Ожидание физического события слота. Использует waitMtx_/waitCv_, инициализированные
// в start(): это позволяет cancel() разбудить ожидание мгновенно (без таймаута).
// confirmed — локальный для каждой фазы (снять, положить), сбрасывается при каждом вызове.
bool ReplaceReelService::waitForSlotEvent(int slotIndex,
                                          bool expectedOccupied) {
    auto mtx       = waitMtx_;
    auto cv        = waitCv_;
    auto confirmed = std::make_shared<bool>(false);
    waitConfirmed_ = confirmed;

    link_.setEventCallback([mtx, cv, confirmed, slotIndex, expectedOccupied]
                           (const stm32::Frame& evt) {
        if (evt.type  == stm32::FrameType::Evt &&
            evt.cmdId == stm32::CommandId::EvtSwitchChanged &&
            evt.payload.size() >= 2)
        {
            const int  evtSlot     = evt.payload[0] + 1;
            const bool evtOccupied = evt.payload[1] == 0x01;

            if (evtSlot == slotIndex && evtOccupied == expectedOccupied) {
                std::lock_guard lock(*mtx);
                *confirmed = true;
                cv->notify_one();
            }
        }
    });

    bool ok = false;
    {
        std::unique_lock lock(*mtx);
        ok = cv->wait_for(
            lock,
            std::chrono::milliseconds(config_.stableConfirmMs),
            [&] { return *confirmed || cancelled_.load(); }
        );
    }

    link_.setEventCallback(nullptr);
    return ok && !cancelled_.load();
}

int ReplaceReelService::start(const std::string& newBarcode) {
    // Завершаем предыдущую операцию, если она ещё активна. После join поток
    // гарантированно не имеет доступа к членам объекта.
    cancelAndJoinWorker();
    cancelled_.store(false);

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

    // Создаём свежие объекты синхронизации для новой операции. Используются
    // обоими фазами waitForSlotEvent внутри одного запуска и доступны cancel().
    waitMtx_ = std::make_shared<std::mutex>();
    waitCv_  = std::make_shared<std::condition_variable>();

    workerThread_ = std::thread([this, opId, slotIndex, newBarcode]() {

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
    });

    return opId;
}

void ReplaceReelService::cancel() {
    cancelled_.store(true);
    // Будим ожидающий поток, чтобы он не ждал stableConfirmMs до таймаута.
    if (waitCv_ && waitMtx_) {
        std::lock_guard lk(*waitMtx_);
        waitCv_->notify_all();
    }
}

} // namespace smartcart::application::services
