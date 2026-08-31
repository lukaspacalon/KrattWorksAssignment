#include "adapters/mavlink/mavlink_codec.hpp"

namespace kratt::adapters::mavlink {

std::vector<std::uint8_t> serialize(const mavlink_message_t& message) {
    std::vector<std::uint8_t> buffer(MAVLINK_MAX_PACKET_LEN);
    const std::uint16_t length = mavlink_msg_to_send_buffer(buffer.data(), &message);
    buffer.resize(length);
    return buffer;
}

std::vector<mavlink_message_t> Decoder::parse(std::span<const std::uint8_t> bytes) {
    std::vector<mavlink_message_t> messages;
    mavlink_message_t decoded{};
    mavlink_status_t frame_status{};

    for (const std::uint8_t byte : bytes) {
        switch (mavlink_frame_char_buffer(&partial_, &status_, byte, &decoded, &frame_status)) {
            case MAVLINK_FRAMING_OK:
                ++stats_.messages;
                messages.push_back(decoded);
                break;
            case MAVLINK_FRAMING_BAD_CRC:
            case MAVLINK_FRAMING_BAD_SIGNATURE:
                ++stats_.crc_errors;
                break;
            default:
                break;  // MAVLINK_FRAMING_INCOMPLETE: keep feeding bytes
        }
    }
    stats_.parse_errors += status_.parse_error;
    status_.parse_error = 0;
    return messages;
}

}  // namespace kratt::adapters::mavlink
