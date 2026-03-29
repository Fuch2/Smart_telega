#include "src/infrastructure/hw/stm32/FrameCodec.hpp"
#include "src/infrastructure/hw/stm32/Protocol.hpp"

#include <gtest/gtest.h>

using namespace smartcart::infrastructure::hw::stm32;

TEST(FrameCodecTest, EncodeDecode_Roundtrip_GetFwVersion) {
    Frame f;
    f.protocolVersion = kProtocolVersionV1;
    f.frameType = FrameType::Cmd;
    f.seq = 0x10;
    f.commandId = static_cast<uint8_t>(CommandId::GetFwVersion);
    f.payload = {};

    const auto bytes = FrameCodec::encode(f);
    const auto decoded = FrameCodec::decode(bytes.data(), bytes.size());

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->protocolVersion, f.protocolVersion);
    EXPECT_EQ(decoded->frameType, f.frameType);
    EXPECT_EQ(decoded->seq, f.seq);
    EXPECT_EQ(decoded->commandId, f.commandId);
    EXPECT_EQ(decoded->payload, f.payload);
}

TEST(FrameCodecTest, Decode_Fails_OnCrcError) {
    Frame f;
    f.protocolVersion = kProtocolVersionV1;
    f.frameType = FrameType::Cmd;
    f.seq = 0x22;
    f.commandId = static_cast<uint8_t>(CommandId::LedSetSlot);
    f.payload = {0x05, 0xFF, 0x00, 0x00, 0x01};

    auto bytes = FrameCodec::encode(f);
    bytes[8] ^= 0x01; // damage payload

    const auto decoded = FrameCodec::decode(bytes.data(), bytes.size());
    EXPECT_FALSE(decoded.has_value());
}

TEST(FrameCodecTest, Decode_Fails_OnBadSof) {
    Frame f;
    f.protocolVersion = kProtocolVersionV1;
    f.frameType = FrameType::Cmd;
    f.seq = 1;
    f.commandId = static_cast<uint8_t>(CommandId::Ping);
    f.payload = {1,2,3,4};

    auto bytes = FrameCodec::encode(f);
    bytes[0] = 0x00;

    const auto decoded = FrameCodec::decode(bytes.data(), bytes.size());
    EXPECT_FALSE(decoded.has_value());
}
