// ===== src/application/services/AddReelService.cpp =====
#include "application/services/AddReelService.hpp"
#include "infrastructure/hw/stm32/Protocol.hpp"

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

namespace smartcart::application::services {

namespace domain = smartcart::domain;
namespace stm32  = smartcart::infrastructure::hw::stm32;

AddReelService::AddReelService(
    ports::IStm32Link&           link,
    ports::IReelRepository&      reelRepo,
    ports::IOperationRepository& opRepo,
    AddReelConfig                config)
    : link_(link)
    , reelRepo_(reelRepo)
    , opRepo_(opRepo)
    , config_(std::move(config))
{}

AddReelService::~AddReelService() {
    cancelAndJoinWorker();
}

// Разбудить и дождаться завершения предыдущей операции.
// Вызывается из деструктора и из start() перед запуском новой операции.
// После возврата метода рабочий поток гарантированно завершён и не имеет
// доступа к членам объекта, что исключает use-after-free.
void AddReelService::cancelAndJoinWorker() {
    cancelled_.store(true);
    if (waitCv_ && waitMtx_) {
        std::lock_guard lk(*waitMtx_);
        waitCv_->notify_all();
    }
    if (workerThread_.joinable()) {
        workerThread_.join();
    }
}

bool AddReelService::isValidBarcode(const std::string& barcode) {
    return !barcode.empty() && barcode.size() <= 64;
}

int AddReelService::findFreeSlot() {
    const auto slots = reelRepo_.getSlotStates(config_.moduleId);
    for (const auto& slot : slots)
        if (slot.state == domain::SlotState::Free)
            return slot.slotIndex;
    return -1;
}

void AddReelService::setSlotLed(int slotIndex,
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

void AddReelService::clearAllLeds() {
    stm32::Frame clearCmd;
    clearCmd.type  = stm32::FrameType::Cmd;
    clearCmd.cmdId = stm32::CommandId::LedClearAll;
    link_.sendCommand(clearCmd);

    stm32::Frame applyCmd;
    applyCmd.type  = stm32::FrameType::Cmd;
    applyCmd.cmdId = stm32::CommandId::LedApply;
    link_.sendCommand(applyCmd);
}

int AddReelService::start(const std::string& barcode) {
    // Если предыдущая операция всё ещё выполняется — отменяем и ждём её завершения.
    // Это гарантирует, что в любой момент времени активен ровно один рабочий поток
    // и его ссылки на member-поля валидны.
    cancelAndJoinWorker();
    cancelled_.store(false);

    if (!isValidBarcode(barcode)) {
        if (onError_) onError_(domain::ErrorCode::InvalidBarcode,
                               "Некорректный штрихкод: " + barcode);
        return -1;
    }

    const int slotIndex = findFreeSlot();
    if (slotIndex < 0) {
        if (onError_) onError_(domain::ErrorCode::NoFreeSlot,
                               "Нет свободных слотов");
        return -1;
    }

    reelRepo_.setSlotState(config_.moduleId, slotIndex,
                           domain::SlotState::Reserved);

    domain::Operation op{};
    op.type      = domain::OperationType::AddReel;
    op.status    = domain::OperationStatus::InProgress;
    op.moduleId  = config_.moduleId;
    op.slotIndex = slotIndex;
    op.barcode   = barcode;

    const int opId = opRepo_.add(op);
    currentOpId_   = opId;
    currentSlot_   = slotIndex;

    setSlotLed(slotIndex, 0, 255, 0);

    // Объекты синхронизации хранятся как члены сервиса, чтобы cancel() мог
    // разбудить ожидающий поток без таймаута. Callback захватывает их по
    // shared_ptr, поэтому переживёт даже неожиданное переподключение UART.
    waitMtx_       = std::make_shared<std::mutex>();
    waitCv_        = std::make_shared<std::condition_variable>();
    auto confirmed = std::make_shared<bool>(false);

    workerThread_ = std::thread(
        [this, opId, slotIndex, barcode,
         mtx = waitMtx_, cv = waitCv_, confirmed]() {

            link_.setEventCallback([mtx, cv, confirmed, slotIndex]
                                   (const stm32::Frame& evt) {
                if (evt.type  == stm32::FrameType::Evt &&
                    evt.cmdId == stm32::CommandId::EvtSwitchChanged &&
                    evt.payload.size() >= 2)
                {
                    const int  evtSlot     = evt.payload[0] + 1;
                    const bool evtOccupied = evt.payload[1] == 0x01;

                    if (evtSlot == slotIndex && evtOccupied) {
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

            if (!ok || cancelled_.load()) {
                reelRepo_.setSlotState(config_.moduleId, slotIndex,
                                       domain::SlotState::Free);
                opRepo_.updateStatus(opId, domain::OperationStatus::Cancelled);
                clearAllLeds();
                if (onComplete_) onComplete_(opId, domain::OperationStatus::Cancelled);
                return;
            }

            reelRepo_.addRecord(config_.moduleId, slotIndex, barcode);
            reelRepo_.setSlotState(config_.moduleId, slotIndex,
                                   domain::SlotState::Occupied);
            opRepo_.updateStatus(opId, domain::OperationStatus::Completed);
            clearAllLeds();

            if (onComplete_) onComplete_(opId, domain::OperationStatus::Completed);
        });

    return opId;
}

void AddReelService::cancel() {
    cancelled_.store(true);
    // Будим рабочий поток, чтобы он не ждал таймаута stableConfirmMs.
    if (waitCv_ && waitMtx_) {
        std::lock_guard lk(*waitMtx_);
        waitCv_->notify_all();
    }
}

} // namespace smartcart::application::services
