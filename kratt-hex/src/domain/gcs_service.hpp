#pragma once

#include "domain/connection_monitor.hpp"
#include "domain/geofence.hpp"
#include "domain/ports/event_log.hpp"
#include "domain/ports/telemetry_sink.hpp"
#include "domain/types.hpp"

namespace kratt::domain {

/// What the GCS believes about the drone. Immutable snapshot handed to the GUI.
struct GcsView {
    Telemetry telemetry{};
    bool connected{false};
    std::optional<Vec3> goto_target{};
};

/// OUTBOUND PORT of the GCS: it issues commands without knowing the wire format.
/// Symmetric to IFlightCommands on the drone side — the same vocabulary, seen
/// from the other end of the link.
class ICommandTransmitter {
public:
    virtual ~ICommandTransmitter() = default;
    ICommandTransmitter(const ICommandTransmitter&) = delete;
    ICommandTransmitter& operator=(const ICommandTransmitter&) = delete;

    virtual void transmit_arm() = 0;
    virtual void transmit_disarm() = 0;
    virtual void transmit_land() = 0;
    virtual void transmit_goto(const Vec3& target) = 0;
    virtual void transmit_manual(const Vec3& axes) = 0;

protected:
    ICommandTransmitter() = default;
};

/// Use-case layer of the ground station. Implements ITelemetrySink so that the
/// receiving adapter can push decoded telemetry straight into it.
class GcsService final : public ITelemetrySink, public ILinkLiveness {
public:
    struct Config {
        Geofence fence{100.0, 60.0};
        Instant connection_timeout{std::chrono::milliseconds{1000}};
        /// MANUAL_CONTROL repetition while a key is held. Must stay well below
        /// the drone-side 500 ms input timeout.
        Instant manual_period{std::chrono::milliseconds{50}};
    };

    GcsService(Config config, ICommandTransmitter& transmitter,
               IEventLog& event_log = NullEventLog::instance());

    // --- Inbound from the receiving adapter ---------------------------------
    /// New telemetry snapshot. Does not by itself prove the link is alive.
    void publish(const Telemetry& telemetry, Instant now) override;
    /// A heartbeat arrived: this, and only this, refreshes the liveness timer.
    void on_heartbeat(Instant now) override;

    /// Called once per GUI frame. Non-blocking.
    void update(Instant now);

    // --- Operator actions ---------------------------------------------------
    void arm();
    void disarm();
    void land();
    void send_goto(const Vec3& target);
    /// Rate-limited. A zero vector after a non-zero one sends a single explicit
    /// "sticks centred" message, then goes quiet.
    void send_manual(const Vec3& axes, Instant now);

    [[nodiscard]] const GcsView& view() const noexcept { return view_; }

private:
    Config config_;
    ICommandTransmitter& transmitter_;
    IEventLog& event_log_;
    ConnectionMonitor connection_;
    GcsView view_{};
    Instant last_manual_sent_{Instant::min()};
    bool manual_active_{false};
};

}  // namespace kratt::domain
