#pragma once

#include "adapters/mavlink/gcs_mavlink_adapter.hpp"
#include "domain/gcs_service.hpp"

namespace kratt::app {

/// Composition root of the ground station. Mirrors DroneComposition.
class GcsComposition {
public:
    struct Config {
        domain::GcsService::Config service{};
    };

    GcsComposition(adapters::mavlink::IMessageChannel& channel, Config config,
                   domain::IEventLog& event_log = domain::NullEventLog::instance());

    /// One GUI frame: drain the link, refresh the connection status.
    void tick(domain::Instant now);

    [[nodiscard]] domain::GcsService& service() noexcept { return service_; }
    [[nodiscard]] const domain::GcsView& view() const noexcept { return service_.view(); }

private:
    adapters::mavlink::IMessageChannel& channel_;
    adapters::mavlink::MavlinkCommandTransmitter transmitter_;
    domain::GcsService service_;
    adapters::mavlink::MavlinkTelemetryReceiver receiver_;
};

}  // namespace kratt::app
