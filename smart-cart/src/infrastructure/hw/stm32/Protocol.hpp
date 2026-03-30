// ===== src/infrastructure/hw/stm32/Protocol.hpp =====
#pragma once

#include <cstdint>
#include <vector>

namespace smartcart::infrastructure::hw::stm32 {

constexpr uint8_t    kSof1               = 0xAA;
constexpr uint8_t    kSof2               = 0x55;
constexpr uint8_t    kProtocolVersionV1  = 0x01;
constexpr uint16_t   kMaxPayloadV1       = 128;

enum class FrameType : uint8_t {
    Cmd  = 0x01,
    Resp = 0x02,
    Ack  = 0x03,
    Nack = 0x04,
    Evt  = 0x05
};

enum class CommandId : uint8_t {
    Ping                 = 0x01,
    GetReadyState        = 0x02,
    GetFwVersion         = 0x03,

    GetSwitchSnapshot    = 0x10,
    GetSwitchDelta       = 0x11,

    LedSetSlot           = 0x20,
    LedSetBulk           = 0x21,
    LedClearAll          = 0x22,
    LedApply             = 0x23,
    LedTest              = 0x24,
    LedSetMappingVersion = 0x25,

    GetDiag              = 0x30,
    ResetDiagCounters    = 0x31,

    SetPollHint          = 0x40,
    Nop                  = 0x7F,

    EvtReady             = 0xE0,
    EvtFault             = 0xE1,
    EvtSwitchChanged     = 0xE2
};

enum class StatusCode : uint8_t {
    Ok                    = 0x00,
    ErrUnknownCmd         = 0x01,
    ErrBadPayload         = 0x02,
    ErrBadLength          = 0x03,
    ErrUnsupportedVersion = 0x05,
    ErrBusy               = 0x06,
    ErrInvalidSlot        = 0x07,
    ErrInvalidParam       = 0x08,
    ErrHwFailure          = 0x09,
    ErrNotReady           = 0x0A,
    ErrRateLimit          = 0x0B,
    ErrInternal           = 0x0C
};

struct Frame {
    uint8_t              protocolVersion {kProtocolVersionV1};
    FrameType            type            {FrameType::Cmd};
    uint8_t              seq             {0};
    CommandId            cmdId           {CommandId::Nop};
    std::vector<uint8_t> payload         {};
};

} // namespace smartcart::infrastructure::hw::stm32
