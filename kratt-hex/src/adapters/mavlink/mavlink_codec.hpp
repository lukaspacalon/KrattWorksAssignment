#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "adapters/mavlink/mavlink_include.hpp"

namespace kratt::adapters::mavlink {

/// Serialises a finalised message into wire bytes. Pure function, no state.
[[nodiscard]] std::vector<std::uint8_t> serialize(const mavlink_message_t& message);

/// Incremental, byte-oriented MAVLink parser.
///
/// Uses `mavlink_frame_char_buffer`, the re-entrant helper: the parser state is
/// a member instead of a global channel table. Two links, or two parallel unit
/// tests, therefore cannot corrupt each other's partial frames — which is the
/// difference between "probably fine" and "provably race-free" here.
///
/// Being byte-oriented, it handles both several frames inside one datagram and
/// a frame split across two datagrams.
class Decoder {
public:
    struct Stats {
        std::uint64_t messages{0};
        std::uint64_t crc_errors{0};
        std::uint64_t parse_errors{0};
    };

    [[nodiscard]] std::vector<mavlink_message_t> parse(std::span<const std::uint8_t> bytes);
    [[nodiscard]] const Stats& stats() const noexcept { return stats_; }

private:
    mavlink_message_t partial_{};
    mavlink_status_t status_{};
    Stats stats_{};
};

}  // namespace kratt::adapters::mavlink
