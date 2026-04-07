#include "application/services/Stm32PollingService.hpp"
#include "infrastructure/hw/stm32/Protocol.hpp"
#include "domain/entities/CartWorkflow.hpp"
#include "domain/entities/Operation.hpp"
#include "domain/entities/Slot.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <exception>
#include <sstream>
#include <string>
#include <utility>

namespace smartcart::application::services {

namespace domain = smartcart::domain;
namespace stm32  = smartcart::infrastructure::hw::stm32;

namespace {

std::vector<bool> parseSnapshotPayload(const std::vector<uint8_t>& payload,
                                       int slotCount) {
    std::vector<bool> result(slotCount, false);
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

std::string trimCopy(const std::string& value) {
    auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    });
    auto end = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    }).base();

    if (begin >= end) {
        return {};
    }
    return std::string(begin, end);
}

} // namespace

Stm32PollingService::Stm32PollingService(
    ports::IStm32Link&      link,
    ports::IReelRepository& reelRepo,
    ports::IOperationRepository& opRepo,
    ports::IOrderRepository& orderRepo,
    ports::IWorkflowRepository& workflowRepo,
    WorkflowService& workflowSvc,
    ports::IEventLogger&    eventLogger,
    Stm32PollingConfig      config)
    : link_(link)
    , reelRepo_(reelRepo)
    , opRepo_(opRepo)
    , orderRepo_(orderRepo)
    , workflowRepo_(workflowRepo)
    , workflowSvc_(workflowSvc)
    , eventLogger_(eventLogger)
    , config_(std::move(config))
{}

Stm32PollingService::~Stm32PollingService() {
    stop();
}

void Stm32PollingService::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return;
    }

    thread_ = std::thread(&Stm32PollingService::pollLoop, this);
}

void Stm32PollingService::stop() {
    if (!running_.exchange(false)) {
        return;
    }

    if (thread_.joinable()) {
        thread_.join();
    }
}

void Stm32PollingService::pollLoop() {
    logSafe("INFO", "Stm32PollingStarted", "Опрос STM32 запущен");

    while (running_.load()) {
        pollOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(config_.pollMs));
    }

    logSafe("INFO", "Stm32PollingStopped", "Опрос STM32 остановлен");
}

void Stm32PollingService::pollOnce() {
    try {
        const auto snapshot = requestSnapshot();
        if (!snapshot.has_value()) {
            logSafe("WARN",
                    "Stm32SnapshotUnavailable",
                    "Не удалось получить snapshot STM32");
            return;
        }

        applySnapshot(*snapshot);
    } catch (const std::exception& ex) {
        logSafe("ERROR", "Stm32PollingError", ex.what());
    } catch (...) {
        logSafe("ERROR", "Stm32PollingError", "Неизвестная ошибка polling");
    }
}

void Stm32PollingService::handleEventFrame(const stm32::Frame& frame) {
    try {
        if (frame.type != stm32::FrameType::Evt) {
            return;
        }

        if (frame.cmdId == stm32::CommandId::EvtReady) {
            logSafe("INFO", "Stm32EvtReady", "STM32 готов");
            return;
        }

        if (frame.cmdId != stm32::CommandId::EvtSwitchChanged) {
            return;
        }

        if (frame.payload.size() < 2) {
            logSafe("WARN",
                    "Stm32EventBadPayload",
                    "EvtSwitchChanged payload слишком короткий");
            return;
        }

        const int channel = static_cast<int>(frame.payload[0]);
        const bool occupied = frame.payload[1] != 0;

        if (channel < 0 || channel >= config_.slotCount) {
            std::ostringstream msg;
            msg << "channel=" << channel;
            logSafe("WARN", "Stm32EventInvalidChannel", msg.str());
            return;
        }

        if (!isTrackedChannel(channel) || isIgnoredChannel(channel)) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        std::lock_guard lock(stateMtx_);

        if (!lastSnapshot_.has_value() ||
            static_cast<int>(lastSnapshot_->size()) != config_.slotCount)
        {
            lastSnapshot_ = std::vector<bool>(config_.slotCount, false);
            rawSnapshot_ = *lastSnapshot_;
            rawChangedAt_.assign(static_cast<size_t>(config_.slotCount), now);
        }
        if (!rawSnapshot_.has_value() ||
            static_cast<int>(rawSnapshot_->size()) != config_.slotCount)
        {
            rawSnapshot_ = *lastSnapshot_;
        }
        if (rawChangedAt_.size() != static_cast<size_t>(config_.slotCount)) {
            rawChangedAt_.assign(static_cast<size_t>(config_.slotCount), now);
        }

        if ((*lastSnapshot_)[channel] == occupied) {
            return;
        }

        (*lastSnapshot_)[channel] = occupied;
        (*rawSnapshot_)[channel] = occupied;
        rawChangedAt_[channel] = now;
        applyChannelStateLocked(channel, occupied, "event");
    } catch (const std::exception& ex) {
        logSafe("ERROR", "Stm32EventHandleFailed", ex.what());
    } catch (...) {
        logSafe("ERROR", "Stm32EventHandleFailed", "Неизвестная ошибка event");
    }
}

BarcodeScanResult Stm32PollingService::recordBarcodeScan(
    const std::string& barcode) {
    const std::string normalized = trimCopy(barcode);
    if (normalized.empty()) {
        return rejectBarcode(domain::ErrorCode::InvalidBarcode,
                             "Пустой штрихкод проигнорирован",
                             "BarcodeIgnored");
    }

    const auto workflow = workflowRepo_.get();
    if (!workflow.currentOrderId.has_value()) {
        return rejectBarcode(domain::ErrorCode::Unknown,
                             "Нет активного заказа",
                             "BarcodeRejected");
    }
    if (workflow.state != domain::CartWorkflowState::OrderLoaded &&
        workflow.state != domain::CartWorkflowState::PickingMaterials)
    {
        return rejectBarcode(domain::ErrorCode::Unknown,
                             "Заказ сейчас не находится на этапе подбора",
                             "BarcodeRejected");
    }

    const auto item =
        orderRepo_.findItemByBarcode(*workflow.currentOrderId, normalized);
    if (!item.has_value()) {
        return rejectBarcode(domain::ErrorCode::ReelNotFound,
                             "Материал не найден в текущем заказе: " + normalized,
                             "BarcodeRejected");
    }
    if (item->status == domain::OrderItemStatus::Placed ||
        item->status == domain::OrderItemStatus::Issued)
    {
        return rejectBarcode(domain::ErrorCode::SlotOccupied,
                             "Материал уже обработан: " + normalized,
                             "BarcodeAlreadyPlaced");
    }

    domain::Operation op;
    op.type = domain::OperationType::AddReel;
    op.status = domain::OperationStatus::InProgress;
    op.moduleId = config_.moduleId;
    op.slotIndex = item->targetSlot;
    op.barcode = normalized;

    try {
        const int opId = opRepo_.add(op);
        {
            std::lock_guard lock(pendingMtx_);
            if (pendingScan_.has_value()) {
                opRepo_.updateStatus(pendingScan_->operationId,
                                     domain::OperationStatus::Cancelled);
            }
            pendingScan_ = PendingScan{
                normalized,
                opId,
                item->id,
                item->targetSlot
            };
        }

        workflowRepo_.setState(domain::CartWorkflowState::PickingMaterials);

        std::ostringstream accepted;
        accepted << "barcode=" << normalized
                 << " target_slot=" << item->targetSlot
                 << " operation_id=" << opId;
        logSafe("INFO", "BarcodeAccepted", accepted.str());
        logSafe("INFO", "TargetSlotSuggested", accepted.str());

        return BarcodeScanResult{
            true,
            opId,
            item->targetSlot,
            domain::ErrorCode::None,
            "Скан принят"
        };
    } catch (const std::exception& ex) {
        return rejectBarcode(domain::ErrorCode::PersistenceError,
                             ex.what(),
                             "BarcodeScanSaveFailed");
    }
}

std::optional<std::vector<bool>> Stm32PollingService::requestSnapshot() {
    auto request = [this](stm32::CommandId cmdId)
        -> std::optional<std::vector<bool>>
    {
        stm32::Frame cmd;
        cmd.type  = stm32::FrameType::Cmd;
        cmd.cmdId = cmdId;

        const auto resp = link_.sendCommand(cmd);
        if (!resp.has_value() ||
            resp->type != stm32::FrameType::Resp ||
            resp->payload.size() < 3)
        {
            return std::nullopt;
        }

        auto snapshot = parseSnapshotPayload(resp->payload, config_.slotCount);
        if (static_cast<int>(snapshot.size()) != config_.slotCount) {
            return std::nullopt;
        }

        return snapshot;
    };

    if (auto snapshot = request(stm32::CommandId::GetSwitchSnapshot)) {
        return snapshot;
    }

    // Совместимость с текущим stm_query.py / ранней прошивкой.
    return request(static_cast<stm32::CommandId>(0x04));
}

void Stm32PollingService::applySnapshot(const std::vector<bool>& snapshot) {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard lock(stateMtx_);

    if (!lastSnapshot_.has_value()) {
        lastSnapshot_ = snapshot;
        rawSnapshot_ = snapshot;
        rawChangedAt_.assign(snapshot.size(), now);

        for (int channel = 0; channel < static_cast<int>(snapshot.size()); ++channel) {
            if (!isTrackedChannel(channel) || isIgnoredChannel(channel)) {
                continue;
            }

            const bool occupied = snapshot[channel];
            applyChannelStateLocked(channel, occupied, "snapshot");
        }
        return;
    }

    if (lastSnapshot_->size() != snapshot.size()) {
        lastSnapshot_ = snapshot;
        rawSnapshot_ = snapshot;
        rawChangedAt_.assign(snapshot.size(), now);
        return;
    }

    if (!rawSnapshot_.has_value() || rawSnapshot_->size() != snapshot.size()) {
        rawSnapshot_ = *lastSnapshot_;
        rawChangedAt_.assign(snapshot.size(), now);
    }
    if (rawChangedAt_.size() != snapshot.size()) {
        rawChangedAt_.assign(snapshot.size(), now);
    }

    for (int channel = 0; channel < static_cast<int>(snapshot.size()); ++channel) {
        if (!isTrackedChannel(channel) || isIgnoredChannel(channel)) {
            continue;
        }

        const bool occupied = snapshot[channel];
        if (occupied != (*rawSnapshot_)[channel]) {
            (*rawSnapshot_)[channel] = occupied;
            rawChangedAt_[channel] = now;
        }

        const bool changed = channel >= static_cast<int>(lastSnapshot_->size()) ||
            occupied != (*lastSnapshot_)[channel];
        if (!changed) {
            continue;
        }

        const auto stableFor = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - rawChangedAt_[channel]
        ).count();
        if (config_.debounceMs > 0 && stableFor < config_.debounceMs) {
            continue;
        }

        (*lastSnapshot_)[channel] = occupied;
        applyChannelStateLocked(channel, occupied, "snapshot");
    }
}

void Stm32PollingService::applyChannelStateLocked(int channel,
                                                  bool occupied,
                                                  std::string_view source) {
    const int slotIndex = channel + 1;
    const bool saved = reelRepo_.setSlotState(
        config_.moduleId,
        slotIndex,
        occupied ? domain::SlotState::Occupied : domain::SlotState::Free
    );

    std::ostringstream msg;
    msg << "channel=" << channel
        << " slot=" << slotIndex
        << " occupied=" << (occupied ? "true" : "false")
        << " source=" << source;

    logSafe(saved ? "INFO" : "ERROR",
            saved ? "SwitchChanged" : "SwitchStateSaveFailed",
            msg.str());

    if (occupied) {
        handleOccupiedSlot(channel, slotIndex);
    } else {
        handleFreedSlot(channel, slotIndex);
    }
}

void Stm32PollingService::handleOccupiedSlot(int channel, int slotIndex) {
    const auto pending = consumePendingScan();
    if (!pending.has_value()) {
        return;
    }

    try {
        if (slotIndex != pending->targetSlot) {
            orderRepo_.updateItemPlacement(
                pending->orderItemId,
                slotIndex,
                domain::OrderItemStatus::WrongSlot);
            reelRepo_.setSlotState(config_.moduleId,
                                   slotIndex,
                                   domain::SlotState::Error);
            opRepo_.updateSlot(pending->operationId, config_.moduleId, slotIndex);
            opRepo_.updateStatus(pending->operationId,
                                 domain::OperationStatus::Failed);

            std::ostringstream msg;
            msg << "channel=" << channel
                << " expected_slot=" << pending->targetSlot
                << " actual_slot=" << slotIndex
                << " barcode=" << pending->barcode
                << " operation_id=" << pending->operationId;
            logSafe("ERROR", "WrongSlotInteraction", msg.str());
            return;
        }

        if (reelRepo_.hasActiveRecord(config_.moduleId, slotIndex)) {
            reelRepo_.markRemovedBySlot(config_.moduleId, slotIndex);
        }

        reelRepo_.addRecord(config_.moduleId, slotIndex, pending->barcode);
        reelRepo_.setSlotState(config_.moduleId,
                               slotIndex,
                               domain::SlotState::Occupied);
        orderRepo_.updateItemPlacement(pending->orderItemId,
                                       slotIndex,
                                       domain::OrderItemStatus::Placed);
        opRepo_.updateSlot(pending->operationId, config_.moduleId, slotIndex);
        opRepo_.updateStatus(pending->operationId,
                             domain::OperationStatus::Completed);

        std::ostringstream msg;
        msg << "channel=" << channel
            << " slot=" << slotIndex
            << " barcode=" << pending->barcode
            << " operation_id=" << pending->operationId;
        logSafe("INFO", "ReelPlacedBySwitch", msg.str());
        logSafe("INFO", "MaterialPlaced", msg.str());
        const auto workflow = workflowSvc_.notifyMaterialPlaced();
        if (!workflow) {
            logSafe("WARN", "WorkflowAdvanceFailed", workflow.message);
        }
    } catch (const std::exception& ex) {
        opRepo_.updateStatus(pending->operationId,
                             domain::OperationStatus::Failed);
        logSafe("ERROR", "ReelPlaceFailed", ex.what());
    }
}

void Stm32PollingService::handleFreedSlot(int channel, int slotIndex) {
    try {
        const bool removed =
            reelRepo_.markRemovedBySlot(config_.moduleId, slotIndex);
        if (!removed) {
            return;
        }

        std::ostringstream msg;
        msg << "channel=" << channel
            << " slot=" << slotIndex;
        logSafe("INFO", "ReelRemovedBySwitch", msg.str());
    } catch (const std::exception& ex) {
        logSafe("ERROR", "ReelRemoveFailed", ex.what());
    }
}

std::optional<Stm32PollingService::PendingScan>
Stm32PollingService::consumePendingScan() {
    std::lock_guard lock(pendingMtx_);
    auto pending = pendingScan_;
    pendingScan_.reset();
    return pending;
}

BarcodeScanResult Stm32PollingService::rejectBarcode(
    domain::ErrorCode code,
    std::string message,
    std::string_view logCode) const
{
    logSafe("ERROR", logCode, message);
    return BarcodeScanResult{false, 0, 0, code, std::move(message)};
}

bool Stm32PollingService::isIgnoredChannel(int channel) const {
    return std::find(config_.ignoredChannels.begin(),
                     config_.ignoredChannels.end(),
                     channel) != config_.ignoredChannels.end();
}

bool Stm32PollingService::isTrackedChannel(int channel) const {
    return std::find(config_.trackedChannels.begin(),
                     config_.trackedChannels.end(),
                     channel) != config_.trackedChannels.end();
}

void Stm32PollingService::logSafe(std::string_view level,
                                  std::string_view code,
                                  std::string_view message) const {
    try {
        eventLogger_.log(level, code, message);
    } catch (...) {
        // Логирование не должно останавливать обмен с STM32.
    }
}

} // namespace smartcart::application::services
