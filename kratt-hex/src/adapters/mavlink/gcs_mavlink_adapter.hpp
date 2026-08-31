#pragma once

#include "adapters/mavlink/drone_mavlink_adapter.hpp"
#include "domain/gcs_service.hpp"

namespace kratt::adapters::mavlink {

/// SECONDARY ADAPTER — implements the GCS outbound command port.
///
/// Mirror image of MavlinkCommandReceiver: where the drone decodes, the ground
/// station encodes. The GCS domain calls `transmit_arm()`; only this class knows
/// that arming is a SET_MODE with the SAFETY_ARMED bit set.
class MavlinkCommandTransmitter final : public domain::ICommandTransmitter {
public:
    explicit MavlinkCommandTransmitter(IMessageChannel& channel,
                                       std::uint8_t system_id = kGcsSystemId,
                                       std::uint8_t target_system = kDroneSystemId);

    void transmit_arm() override;
    void transmit_disarm() override;
    void transmit_land() override;
    void transmit_goto(const domain::Vec3& target) override;
    void transmit_manual(const domain::Vec3& axes) override;

private:
    void send_set_mode(bool armed);

    IMessageChannel& channel_;
    std::uint8_t system_id_;
    std::uint8_t target_system_;
    mavlink_status_t tx_status_{};
};

/// PRIMARY ADAPTER — feeds decoded telemetry into the GCS domain.
///
/// It pushes into an `ITelemetrySink`, the very same port the drone publishes
/// through. That symmetry is not cosmetic: it means a recorded flight can be
/// replayed into the GCS with no adapter at all.
class MavlinkTelemetryReceiver {
public:
    MavlinkTelemetryReceiver(domain::ITelemetrySink& sink, domain::ILinkLiveness& liveness);

    /// Returns true when the message updated the telemetry snapshot.
    bool handle(const mavlink_message_t& message, domain::Instant now);
    void handle_all(const std::vector<Inbound>& inbound, domain::Instant now);

    [[nodiscard]] std::uint64_t heartbeats() const noexcept { return heartbeats_; }
    [[nodiscard]] std::uint64_t acks() const noexcept { return acks_; }

private:
    domain::ITelemetrySink& sink_;
    domain::ILinkLiveness& liveness_;
    /// Accumulates fields across HEARTBEAT and LOCAL_POSITION_NED, because the
    /// domain port takes one complete Telemetry rather than partial updates.
    domain::Telemetry snapshot_{};
    std::uint64_t heartbeats_{0};
    std::uint64_t acks_{0};
};

}  // namespace kratt::adapters::mavlink
