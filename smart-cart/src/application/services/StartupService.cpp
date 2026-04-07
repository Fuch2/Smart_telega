#include "application/services/StartupService.hpp"
#include "infrastructure/hw/stm32/Protocol.hpp"
#include "domain/entities/ModuleInfo.hpp"

#include <chrono>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <string>
#include <unordered_set>
#include <utility>

namespace smartcart::application::services {

namespace domain = smartcart::domain;
namespace stm32  = smartcart::infrastructure::hw::stm32;

namespace {

bool isPositiveReply(const stm32::Frame& frame) {
    return frame.type == stm32::FrameType::Ack ||
           frame.type == stm32::FrameType::Resp;
}

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

} // namespace

StartupService::StartupService(
    ports::IStm32Link&        link,
    ports::IReelRepository&   reelRepo,
    ports::IModuleRepository& moduleRepo,
    StartupConfig             config)
    : link_(link)
    , reelRepo_(reelRepo)
    , moduleRepo_(moduleRepo)
    , config_(std::move(config))
{}

StartupResult StartupService::run() {
    try {
        ensureModuleExists();
        ensureSlotsInitialized();

        if (!link_.open()) {
            return domain::ErrorCode::Stm32CommunicationError;
        }
        if (!ping()) {
            return domain::ErrorCode::Stm32CommunicationError;
        }
        if (!waitReady()) {
            return domain::ErrorCode::Stm32CommunicationError;
        }

        const auto physical = getSnapshot();
        if (static_cast<int>(physical.size()) != config_.slotCount) {
            return domain::ErrorCode::Stm32CommunicationError;
        }

        return reconcile(physical);
    } catch (const std::exception&) {
        return domain::ErrorCode::PersistenceError;
    }
}

void StartupService::ensureModuleExists() {
    if (moduleRepo_.getById(config_.moduleId).has_value()) {
        return;
    }

    domain::ModuleInfo module;
    module.serial = "SMARTCART-MODULE-" + std::to_string(config_.moduleId);
    module.slotCount = config_.slotCount;
    module.firmware = "";
    module.status = domain::ModuleStatus::Online;

    moduleRepo_.add(module);
}

void StartupService::ensureSlotsInitialized() {
    const auto existingSlots = reelRepo_.getSlotStates(config_.moduleId);
    std::unordered_set<int> existingIndexes;
    for (const auto& slot : existingSlots) {
        existingIndexes.insert(slot.slotIndex);
    }

    for (int slotIndex = 1; slotIndex <= config_.slotCount; ++slotIndex) {
        if (existingIndexes.find(slotIndex) == existingIndexes.end()) {
            reelRepo_.setSlotState(config_.moduleId,
                                   slotIndex,
                                   domain::SlotState::Free);
        }
    }
}

bool StartupService::ping() {
    stm32::Frame cmd;
    cmd.type  = stm32::FrameType::Cmd;
    cmd.cmdId = stm32::CommandId::Ping;

    const auto resp = link_.sendCommand(cmd);
    return resp.has_value() && isPositiveReply(*resp);
}

bool StartupService::waitReady() {
    std::mutex mtx;
    std::condition_variable cv;
    bool readyEvent = false;

    link_.setEventCallback([&mtx, &cv, &readyEvent](const stm32::Frame& evt) {
        if (evt.type == stm32::FrameType::Evt &&
            evt.cmdId == stm32::CommandId::EvtReady)
        {
            std::lock_guard lock(mtx);
            readyEvent = true;
            cv.notify_one();
        }
    });

    stm32::Frame cmd;
    cmd.type  = stm32::FrameType::Cmd;
    cmd.cmdId = stm32::CommandId::GetReadyState;

    const auto resp = link_.sendCommand(cmd);
    if (!resp.has_value() || resp->type == stm32::FrameType::Nack) {
        link_.setEventCallback(nullptr);
        return false;
    }

    if (resp->type == stm32::FrameType::Resp &&
        !resp->payload.empty() &&
        resp->payload[0] == 0x01)
    {
        link_.setEventCallback(nullptr);
        return true;
    }

    std::unique_lock lock(mtx);
    cv.wait_for(lock,
                std::chrono::milliseconds(config_.readyTimeoutMs),
                [&readyEvent] { return readyEvent; });
    link_.setEventCallback(nullptr);

    // Текущая firmware может отвечать NOT_READY и не слать EvtReady,
    // но при этом корректно отдавать snapshot. Не блокируем запуск навсегда.
    return true;
}

std::vector<bool> StartupService::getSnapshot() {
    auto requestSnapshot = [this](stm32::CommandId cmdId) -> std::vector<bool> {
        stm32::Frame cmd;
        cmd.type  = stm32::FrameType::Cmd;
        cmd.cmdId = cmdId;

        const auto resp = link_.sendCommand(cmd);
        if (!resp.has_value() ||
            resp->type != stm32::FrameType::Resp ||
            resp->payload.size() < 3)
        {
            return {};
        }

        return parseSnapshotPayload(resp->payload, config_.slotCount);
    };

    auto snapshot = requestSnapshot(stm32::CommandId::GetSwitchSnapshot);
    if (static_cast<int>(snapshot.size()) == config_.slotCount) {
        return snapshot;
    }

    // Совместимость с ранней прошивкой/утилитой stm_query.py.
    return requestSnapshot(static_cast<stm32::CommandId>(0x04));
}

std::vector<domain::Slot> StartupService::reconcile(
    const std::vector<bool>& physical)
{
    const auto activeRecords = reelRepo_.getActiveByModule(config_.moduleId);
    std::unordered_set<int> activeSlots;
    for (const auto& record : activeRecords) {
        activeSlots.insert(record.slotIndex);
    }

    std::vector<domain::Slot> slots;
    slots.reserve(static_cast<size_t>(config_.slotCount));

    for (int slotIndex = 1; slotIndex <= config_.slotCount; ++slotIndex) {
        const int channel = slotIndex - 1;
        bool physicallyOccupied =
            channel >= 0 &&
            channel < static_cast<int>(physical.size()) &&
            physical[channel];

        if (channel == 11) {
            physicallyOccupied = false;
        }

        const bool hasRecord =
            activeSlots.find(slotIndex) != activeSlots.end();

        domain::SlotState state = domain::SlotState::Free;
        if (physicallyOccupied && hasRecord) {
            state = domain::SlotState::Occupied;
        } else if (!physicallyOccupied && !hasRecord) {
            state = domain::SlotState::Free;
        } else {
            state = domain::SlotState::Error;
        }

        reelRepo_.setSlotState(config_.moduleId, slotIndex, state);

        const int ledIndex =
            channel < static_cast<int>(config_.slotToLedMap.size())
            ? config_.slotToLedMap[channel]
            : channel * 2;

        slots.push_back(domain::Slot{
            config_.moduleId,
            slotIndex,
            ledIndex,
            state
        });
    }

    return slots;
}

} // namespace smartcart::application::services
