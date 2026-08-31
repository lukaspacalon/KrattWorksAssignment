#pragma once

#include <memory>

#include "adapters/mavlink/drone_mavlink_adapter.hpp"
#include "domain/drone_service.hpp"

namespace kratt::app {

/// Composition root of the drone.
///
/// The only place in the code base that knows about *both* the domain and the
/// adapters. It owns the object graph and the lifetime; everything else
/// receives its collaborators by reference and owns nothing.
///
/// `tick()` is the single entry point of one simulation step, and it is
/// deliberately independent of any clock: the real-time loop lives in
/// `run_realtime()`, the tests drive `tick()` directly with a virtual clock.
/// That is why the same production object graph is exercised by the integration
/// suite, with zero threads.
class DroneComposition {
public:
    struct Config {
        domain::DroneService::Config service{};
    };

    DroneComposition(adapters::mavlink::IMessageChannel& channel, Config config,
                     domain::IEventLog& event_log = domain::NullEventLog::instance());

    /// One tick: drain commands, advance physics, publish telemetry.
    /// Never blocks, never allocates in steady state.
    void tick(domain::Seconds dt, domain::Instant now);

    [[nodiscard]] domain::DroneService& service() noexcept { return service_; }
    [[nodiscard]] const domain::Telemetry& telemetry() const noexcept {
        return service_.telemetry();
    }

private:
    adapters::mavlink::IMessageChannel& channel_;
    // Declaration order is the construction order: the publisher must outlive
    // the receiver that references it.
    adapters::mavlink::MavlinkTelemetryPublisher publisher_;
    domain::DroneService service_;
    adapters::mavlink::MavlinkCommandReceiver receiver_;
};

}  // namespace kratt::app
