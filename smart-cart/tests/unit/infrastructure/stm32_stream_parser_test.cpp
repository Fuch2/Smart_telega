#include "src/infrastructure/hw/stm32/FrameCodec.hpp"
#include "src/infrastructure/hw/stm32/Protocol.hpp"

#include <gtest/gtest.h>

using namespace smartcart::infrastructure::hw::stm32;

TEST(Stm32ParserTest, Parses_Frame_ByteByByte) {
    Frame f;
    f.protocolVersion = kProtocolVersionV1;
    f.frameType = FrameType::Cmd;
    f.seq = 0x33;
    f.commandId = static_cast<uint8_t>(CommandId::GetSwitchSnapshot);
    f.payload = {};

    const auto bytes = FrameCodec::encode(f);

    FrameCodec::StreamParser p;
    bool got = false;
    for (uint8_t b : bytes) {
        auto ev = p.feed(b);
        if (ev.type == FrameCodec::ParseEventType::FrameReady) {
            ASSERT_TRUE(ev.frame.has_value());
            EXPECT_EQ(ev.frame->seq, 0x33);
            EXPECT_EQ(ev.frame->commandId, static_cast<uint8_t>(CommandId::GetSwitchSnapshot));
            got = true;
        }
    }
    EXPECT_TRUE(got);
}

TEST(Stm32ParserTest, Detects_CrcError_And_Resyncs) {
    Frame good1;
    good1.protocolVersion = kProtocolVersionV1;
    good1.frameType = FrameType::Cmd;
    good1.seq = 0x01;
    good1.commandId = static_cast<uint8_t>(CommandId::Ping);
    good1.payload = {0xAA,0xBB,0xCC,0xDD};

    Frame good2 = good1;
    good2.seq = 0x02;

    auto bad = FrameCodec::encode(good1);
    bad.back() ^= 0xFF; // break CRC

    auto ok = FrameCodec::encode(good2);

    FrameCodec::StreamParser p;
    bool saw_crc = false;
    bool saw_second = false;

    for (uint8_t b : bad) {
        auto ev = p.feed(b);
        if (ev.type == FrameCodec::ParseEventType::CrcError) saw_crc = true;
    }
    for (uint8_t b : ok) {
        auto ev = p.feed(b);
        if (ev.type == FrameCodec::ParseEventType::FrameReady) {
            ASSERT_TRUE(ev.frame.has_value());
            EXPECT_EQ(ev.frame->seq, 0x02);
            saw_second = true;
        }
    }

    EXPECT_TRUE(saw_crc);
    EXPECT_TRUE(saw_second);
    EXPECT_GE(p.crcDropCount(), 1u);
}

TEST(Stm32ParserTest, Rejects_TooLargePayloadLength) {
    // Manually craft frame start + header with payload len > 128
    std::vector<uint8_t> raw = {
        kSof1, kSof2,
        kProtocolVersionV1,
        static_cast<uint8_t>(FrameType::Cmd),
        0x11,
        static_cast<uint8_t>(CommandId::Nop),
        0x81, 0x00 // 129
    };

    FrameCodec::StreamParser p;
    FrameCodec::ParseEvent ev{};
    for (uint8_t b : raw) ev = p.feed(b);

    EXPECT_EQ(ev.type, FrameCodec::ParseEventType::LengthError);
    EXPECT_GE(p.parseErrorCount(), 1u);
}
