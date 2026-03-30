// ===== src/application/services/StartupService.cpp =====
// Исправлено: добавлен #include <unordered_map>
#include "application/services/StartupService.hpp"
#include "infrastructure/hw/stm32/Protocol.hpp"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <unordered_map>

namespace smartcart::application::services {

using namespace smartcart::domain;
using namespace smartcart::infrastructure::hw::stm32;

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
    if (!ping())
        return ErrorCode::Stm32CommunicationError;

    if (!waitReady())
        return ErrorCode::Stm32CommunicationError;

    std::vector<bool> physical;
    try {
        physical = getSnapshot();
    } catch (const std::exception&) {
        return ErrorCode::Stm32CommunicationError;
    }

    return reconcile(physical);
}

bool StartupService::ping() {
    Frame cmd;
    cmd.type  = FrameType::Cmd;
    cmd.cmdId = CommandId::Ping;

    auto resp = link_.sendCommand(cmd);
    return resp.has_value() && resp->type == FrameType::Ack;
}

bool StartupService::waitReady() {
    Frame cmd;
    cmd.type  = FrameType::Cmd;
    cmd.cmdId = CommandId::GetReadyState;

    auto resp = link_.sendCommand(cmd);
    if (!resp.has_value())
        return false;

    if (resp->type == FrameType::Resp &&
        !resp->payload.empty() &&
        resp->payload[0] == 0x01)
        return true;

    std::mutex              mtx;
    std::condition_variable cv;
    bool                    ready = false;

    link_.setEventCallback([&](const Frame& evt) {
        if (evt.type == FrameType::Evt && evt.cmdId == CommandId::EvtReady) {
            std::lock_guard lock(mtx);
            ready = true;
            cv.notify_one();
        }
    });

    std::unique_lock lock(mtx);
    const bool ok = cv.wait_for(
        lock,
        std::chrono::milliseconds(config_.readyTimeoutMs),
        [&] { return ready; }
    );

    link_.setEventCallback(nullptr);
    return ok;
}

std::vector<bool> StartupService::getSnapshot() {
    Frame cmd;
    cmd.type  = FrameType::Cmd;
    cmd.cmdId = CommandId::GetSwitchSnapshot;

    auto resp = link_.sendCommand(cmd);
    if (!resp.has_value() ||
        resp->type != FrameType::Resp ||
        resp->payload.size() < 3)
    {
        throw std::runtime_error("GetSwitchSnapshot: неверный ответ");
    }

    std::vector<bool> result(config_.slotCount, false);
    for (int i = 0; i < config_.slotCount; ++i) {
        const int byteIdx = i / 8;
        const int bitIdx  = i % 8;
        result[i] = (resp->payload[byteIdx] >> bitIdx) & 0x01;
    }
    return result;
}

std::vector<Slot> StartupService::reconcile(const std::vector<bool>& physical) {
    const auto activeReels = reelRepo_.getActiveByModule(config_.moduleId);

    std::unordered_map<int, std::string> reelBySlot;
    for (const auto& reel : activeReels)
        reelBySlot[reel.slotIndex] = reel.barcode;

    std::vector<Slot> slots;
    slots.reserve(config_.slotCount);

    for (int i = 0; i < config_.slotCount; ++i) {
        const int slotIndex = i + 1;
        const int ledIndex  = (i < static_cast<int>(config_.slotToLedMap.size()))
                              ? config_.slotToLedMap[i]
                              : i * 2;

        const bool physOccupied = physical[i];
        const bool dbOccupied   = reelBySlot.contains(slotIndex);

        SlotState state = SlotState::Free;
        if      ( physOccupied &&  dbOccupied) state = SlotState::Occupied;
        else if ( physOccupied && !dbOccupied) state = SlotState::Error;
        else if (!physOccupied &&  dbOccupied) state = SlotState::Error;
        else                                   state = SlotState::Free;

        slots.push_back(Slot{
            .moduleId  = config_.moduleId,
            .slotIndex = slotIndex,
            .ledIndex  = ledIndex,
            .state     = state
        });

        moduleRepo_.updateSlotState(config_.moduleId, slotIndex, state);
    }

    return slots;
}

} // namespace smartcart::application::services
