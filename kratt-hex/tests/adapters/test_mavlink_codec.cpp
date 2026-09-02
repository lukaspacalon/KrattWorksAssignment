#include <gtest/gtest.h>

#include <array>

#include "adapters/mavlink/mavlink_codec.hpp"
#include "adapters/mavlink/mavlink_include.hpp"

namespace kratt::adapters::mavlink {
namespace {

/// MAVLink codec tests: serialization and incremental byte-wise parsing.
class MavlinkCodecTest : public ::testing::Test {
protected:
    Decoder decoder{};
};

TEST(SerializationTest, Heartbeat_serializes_to_bytes) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &message, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0,
                               MAV_STATE_ACTIVE);

    const auto bytes = serialize(message);
    EXPECT_GT(bytes.size(), 0U);
    EXPECT_LE(bytes.size(), MAVLINK_MAX_PACKET_LEN);
    // MAVLink frames always start with the protocol version magic byte
    EXPECT_EQ(bytes[0], MAVLINK_STX);
}

TEST(SerializationTest, Serialized_frame_can_be_parsed_back) {
    mavlink_message_t original{};
    mavlink_msg_heartbeat_pack(2, MAV_COMP_ID_AUTOPILOT1, &original, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 12345,
                               MAV_STATE_ACTIVE);

    const auto bytes = serialize(original);

    Decoder decoder;
    const auto decoded = decoder.parse(bytes);
    ASSERT_EQ(decoded.size(), 1U);
    EXPECT_EQ(decoded[0].msgid, MAVLINK_MSG_ID_HEARTBEAT);
    EXPECT_EQ(decoded[0].sysid, 2);
}

TEST_F(MavlinkCodecTest, Parser_handles_single_complete_frame) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &message, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0,
                               MAV_STATE_ACTIVE);

    const auto bytes = serialize(message);
    const auto parsed = decoder.parse(bytes);
    ASSERT_EQ(parsed.size(), 1U);
    EXPECT_EQ(parsed[0].msgid, MAVLINK_MSG_ID_HEARTBEAT);
}

TEST_F(MavlinkCodecTest, Parser_handles_incomplete_frames_gracefully) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &message, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0,
                               MAV_STATE_ACTIVE);

    auto bytes = serialize(message);
    bytes.resize(bytes.size() / 2);  // truncate
    auto partial = decoder.parse(bytes);
    // Incomplete frame: may or may not produce output, but must not crash
    (void)partial;
}

TEST_F(MavlinkCodecTest, Parser_handles_multiple_frames_in_one_buffer) {
    std::vector<std::uint8_t> combined;

    for (int i = 0; i < 3; ++i) {
        mavlink_message_t message{};
        mavlink_msg_heartbeat_pack(1 + i, MAV_COMP_ID_AUTOPILOT1, &message, MAV_TYPE_QUADROTOR,
                                   MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, i,
                                   MAV_STATE_ACTIVE);
        auto bytes = serialize(message);
        combined.insert(combined.end(), bytes.begin(), bytes.end());
    }

    const auto parsed = decoder.parse(combined);
    EXPECT_EQ(parsed.size(), 3U);
    for (std::size_t i = 0; i < parsed.size(); ++i) {
        EXPECT_EQ(parsed[i].sysid, 1 + static_cast<int>(i));
    }
}

TEST_F(MavlinkCodecTest, Bad_crc_is_counted) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &message, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0,
                               MAV_STATE_ACTIVE);

    auto bytes = serialize(message);
    if (bytes.size() > 2) {
        bytes[bytes.size() - 1] ^= 0xFF;  // flip the CRC byte
    }

    const auto before = decoder.stats().crc_errors;
    decoder.parse(bytes);
    const auto after = decoder.stats().crc_errors;
    EXPECT_GT(after, before);
}

TEST_F(MavlinkCodecTest, Decoder_is_stateful_across_calls) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(5, MAV_COMP_ID_AUTOPILOT1, &message, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0,
                               MAV_STATE_ACTIVE);
    auto bytes = serialize(message);

    // Feed byte by byte and accumulate
    std::vector<mavlink_message_t> all_decoded;
    for (std::uint8_t byte : bytes) {
        auto result = decoder.parse(std::span(&byte, 1));
        all_decoded.insert(all_decoded.end(), result.begin(), result.end());
    }

    ASSERT_EQ(all_decoded.size(), 1U);
    EXPECT_EQ(all_decoded[0].sysid, 5);
}

TEST_F(MavlinkCodecTest, Stats_accumulate) {
    EXPECT_EQ(decoder.stats().messages, 0U);

    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &message, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0,
                               MAV_STATE_ACTIVE);
    auto bytes = serialize(message);

    decoder.parse(bytes);
    EXPECT_EQ(decoder.stats().messages, 1U);

    decoder.parse(bytes);
    EXPECT_EQ(decoder.stats().messages, 2U);
}

}  // namespace
}  // namespace kratt::adapters::mavlink
