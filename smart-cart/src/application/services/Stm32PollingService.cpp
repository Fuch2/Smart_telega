#include "application/services/Stm32PollingService.hpp"
#include "infrastructure/hw/stm32/Protocol.hpp"
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
    ports::IEventLogger&    eventLogger,
    Stm32PollingConfig      config)
    : link_(link)
    , reelRepo_(reelRepo)
    , opRepo_(opRepo)
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

std::optional<int> Stm32PollingService::recordBarcodeScan(
    const std::string& barcode)
{
    const std::string normalized = trimCopy(barcode);
    if (normalized.empty()) {
        logSafe("WARN", "BarcodeIgnored", "Пустой штрихкод проигнорирован");
        return std::nullopt;
    }

    domain::Operation op;
    op.type = domain::OperationType::AddReel;
    op.status = domain::OperationStatus::InProgress;
    op.moduleId = config_.moduleId;
    op.slotIndex = 0;
    op.barcode = normalized;

    try {
        const int opId = opRepo_.add(op);
        {
            std::lock_guard lock(pendingMtx_);
            if (pendingScan_.has_value()) {
                opRepo_.updateStatus(pendingScan_->operationId,
                                     domain::OperationStatus::Cancelled);
            }
            pendingScan_ = PendingScan{normalized, opId};
        }

        logSafe("INFO", "BarcodeScanned", normalized);
        return opId;
    } catch (const std::exception& ex) {
        logSafe("ERROR", "BarcodeScanSaveFailed", ex.what());
        return std::nullopt;
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
    for (int channel = 0; channel < static_cast<int>(snapshot.size()); ++channel) {
        if (!isTrackedChannel(channel) || isIgnoredChannel(channel)) {
            continue;
        }

        const bool occupied = snapshot[channel];
        const bool changed =
            !lastSnapshot_.has_value() ||
            channel >= static_cast<int>(lastSnapshot_->size()) ||
            occupied != (*lastSnapshot_)[channel];

        if (!changed) {
            continue;
        }

        const int slotIndex = channel + 1;
        const bool saved = reelRepo_.setSlotState(
            config_.moduleId,
            slotIndex,
            occupied ? domain::SlotState::Occupied : domain::SlotState::Free
        );

        std::ostringstream msg;
        msg << "channel=" << channel
            << " slot=" << slotIndex
            << " occupied=" << (occupied ? "true" : "false");

        logSafe(saved ? "INFO" : "ERROR",
                saved ? "SwitchChanged" : "SwitchStateSaveFailed",
                msg.str());

        if (occupied) {
            handleOccupiedSlot(channel, slotIndex);
        } else {
            handleFreedSlot(channel, slotIndex);
        }
    }

    lastSnapshot_ = snapshot;
}

void Stm32PollingService::handleOccupiedSlot(int channel, int slotIndex) {
    const auto pending = consumePendingScan();
    if (!pending.has_value()) {
        return;
    }

    try {
        if (reelRepo_.hasActiveRecord(config_.moduleId, slotIndex)) {
            reelRepo_.markRemovedBySlot(config_.moduleId, slotIndex);
        }

        reelRepo_.addRecord(config_.moduleId, slotIndex, pending->barcode);
        reelRepo_.setSlotState(config_.moduleId,
                               slotIndex,
                               domain::SlotState::Occupied);
        opRepo_.updateSlot(pending->operationId, config_.moduleId, slotIndex);
        opRepo_.updateStatus(pending->operationId,
                             domain::OperationStatus::Completed);

        std::ostringstream msg;
        msg << "channel=" << channel
            << " slot=" << slotIndex
            << " barcode=" << pending->barcode
            << " operation_id=" << pending->operationId;
        logSafe("INFO", "ReelPlacedBySwitch", msg.str());
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
