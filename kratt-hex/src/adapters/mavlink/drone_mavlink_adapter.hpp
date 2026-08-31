#pragma once

#include "adapters/mavlink/message_channel.hpp"
#include "domain/ports/flight_commands.hpp"
#include "domain/ports/telemetry_sink.hpp"

namespace kratt::adapters::mavlink {

/// MAVLink identities. Kept in the adapter: system ids are a protocol concept,
/// not a domain one.
inline constexpr std::uint8_t kDroneSystemId = 1;
inline constexpr std::uint8_t kGcsSystemId = 255;

/// SECONDARY (driven) ADAPTER — implements the domain's outbound telemetry port.
///
/// The domain calls `publish(Telemetry)`; this class turns that into HEARTBEAT
/// + LOCAL_POSITION_NED and hands the frames to the channel. The domain never
/// learns that either message exists.
class MavlinkTelemetryPublisher final : public domain::ITelemetrySink {
public:
    explicit MavlinkTelemetryPublisher(IMessageChannel& channel,
                                       std::uint8_t system_id = kDroneSystemId);

    void publish(const domain::Telemetry& telemetry, domain::Instant now) override;

    /// Sends a COMMAND_ACK. Separate from publish() so the two concerns stay
    /// independent (Interface Segregation).
    void acknowledge(std::uint16_t command, domain::CommandResult result,
                     std::uint8_t target_system);

private:
    IMessageChannel& channel_;
    std::uint8_t system_id_;
    /// Per-adapter tx state. The `*_pack_status` helpers are used precisely so
    /// this counter is a member and not a global channel table.
    mavlink_status_t tx_status_{};
};

/// PRIMARY (driving) ADAPTER — turns incoming MAVLink into domain commands.
///
/// This is the translation table between the protocol and the ubiquitous
/// language. It holds no state of its own, which is why a unit test can feed it
/// a raw frame and assert on both the resulting flight state and the ACK.
class MavlinkCommandReceiver {
public:
    MavlinkCommandReceiver(domain::IFlightCommands& commands,
                           MavlinkTelemetryPublisher& publisher,
                           std::uint8_t system_id = kDroneSystemId);

    /// Handles one message. Ignores anything addressed elsewhere or unknown.
    void handle(const mavlink_message_t& message, domain::Instant now);

    /// Handles a batch, e.g. everything returned by `IMessageChannel::poll()`.
    void handle_all(const std::vector<Inbound>& inbound, domain::Instant now);

private:
    domain::IFlightCommands& commands_;
    MavlinkTelemetryPublisher& publisher_;
    std::uint8_t system_id_;
};

}  // namespace kratt::adapters::mavlink
