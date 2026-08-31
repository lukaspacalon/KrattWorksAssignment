#pragma once

#include <memory>

#include "adapters/logging/console_event_log.hpp"
#include "adapters/mavlink/message_channel.hpp"
#include "adapters/transport/loopback_transport.hpp"
#include "app/drone_composition.hpp"
#include "app/gcs_composition.hpp"

namespace kratt::testing {

/// A complete Drone + GCS system, wired over an in-memory network.
///
/// Zero threads, zero sockets, virtual clock. Every integration test in this
/// suite is therefore deterministic and runs in microseconds — that property is
/// a direct consequence of the hexagonal seams, not an accident: the production
/// object graph is instantiated verbatim, only the adapters differ.
///
/// The two `tick` calls are interleaved to mimic the real order of events
/// (drone simulates and publishes, GCS consumes), so a bug in the message
/// ordering shows up here rather than only on real hardware.
class World {
public:
    static constexpr std::uint16_t kDronePort = 14551;
    static constexpr std::uint16_t kGcsPort = 14550;

    explicit World(domain::FlightConfig flight = {}) {
        auto drone_transport = network_.create_endpoint(kDronePort);
        auto gcs_transport = network_.create_endpoint(kGcsPort);

        adapters::mavlink::SynchronousChannel::Config drone_channel_config;
        drone_channel_config.peer = adapters::Endpoint{adapters::LoopbackNetwork::kAddress,
                                                       kGcsPort};
        drone_channel_config.learn_peer_from_traffic = false;
        drone_channel_ = std::make_unique<adapters::mavlink::SynchronousChannel>(
            std::move(drone_transport), drone_channel_config);

        adapters::mavlink::SynchronousChannel::Config gcs_channel_config;
        gcs_channel_config.learn_peer_from_traffic = true;  // discovers the drone
        gcs_channel_ = std::make_unique<adapters::mavlink::SynchronousChannel>(
            std::move(gcs_transport), gcs_channel_config);

        app::DroneComposition::Config drone_config;
        drone_config.service.flight = flight;
        drone_ = std::make_unique<app::DroneComposition>(*drone_channel_, drone_config,
                                                        drone_events_);

        app::GcsComposition::Config gcs_config;
        gcs_config.service.fence = flight.fence;
        gcs_ = std::make_unique<app::GcsComposition>(*gcs_channel_, gcs_config, gcs_events_);
    }

    /// Advances the virtual clock by `dt` and runs one tick on each side.
    void advance(std::chrono::milliseconds dt) {
        now_ += std::chrono::duration_cast<domain::Instant>(dt);
        const domain::Seconds seconds = domain::to_seconds(
            std::chrono::duration_cast<domain::Instant>(dt));
        drone_->tick(seconds, now_);
        gcs_->tick(now_);
    }

    /// Advances in `step` increments until `predicate` holds or `budget` elapses.
    /// Returns false on timeout, so a test failure reads as an assertion rather
    /// than as a hang.
    template <typename Predicate>
    bool advance_until(Predicate predicate, std::chrono::milliseconds budget,
                       std::chrono::milliseconds step = std::chrono::milliseconds{10}) {
        if (predicate()) {
            return true;
        }
        for (std::chrono::milliseconds elapsed{0}; elapsed < budget; elapsed += step) {
            advance(step);
            if (predicate()) {
                return true;
            }
        }
        return false;
    }

    /// Advances until the GCS has heard the drone and can therefore address it.
    ///
    /// This mirrors reality: the ground station is the UDP server, it has no
    /// peer address until the first telemetry frame arrives, so a command issued
    /// before that is dropped. Tests that exercise commands must establish the
    /// link first — making that explicit here rather than hiding it in the
    /// constructor keeps the connection tests honest.
    bool establish_link(std::chrono::milliseconds budget = std::chrono::milliseconds{2000}) {
        return advance_until([this] { return gcs_->view().connected; }, budget);
    }

    [[nodiscard]] domain::Instant now() const noexcept { return now_; }
    [[nodiscard]] app::DroneComposition& drone() noexcept { return *drone_; }
    [[nodiscard]] app::GcsComposition& gcs() noexcept { return *gcs_; }
    [[nodiscard]] const domain::Telemetry& drone_state() const noexcept {
        return drone_->telemetry();
    }
    [[nodiscard]] const domain::GcsView& gcs_view() const noexcept { return gcs_->view(); }
    [[nodiscard]] adapters::LoopbackNetwork& network() noexcept { return network_; }
    [[nodiscard]] adapters::RecordingEventLog& drone_events() noexcept { return drone_events_; }
    [[nodiscard]] adapters::RecordingEventLog& gcs_events() noexcept { return gcs_events_; }

private:
    // Declaration order matters: the network outlives the channels, which
    // outlive the compositions that reference them.
    adapters::LoopbackNetwork network_;
    adapters::RecordingEventLog drone_events_;
    adapters::RecordingEventLog gcs_events_;
    std::unique_ptr<adapters::mavlink::SynchronousChannel> drone_channel_;
    std::unique_ptr<adapters::mavlink::SynchronousChannel> gcs_channel_;
    std::unique_ptr<app::DroneComposition> drone_;
    std::unique_ptr<app::GcsComposition> gcs_;
    domain::Instant now_{};
};

}  // namespace kratt::testing
