#include "domain/gcs_service.hpp"

namespace kratt::domain {

GcsService::GcsService(Config config, ICommandTransmitter& transmitter, IEventLog& event_log)
    : config_{config},
      transmitter_{transmitter},
      event_log_{event_log},
      connection_{config.connection_timeout} {}

void GcsService::publish(const Telemetry& telemetry, Instant) {
    view_.telemetry = telemetry;
}

void GcsService::on_heartbeat(Instant now) { connection_.on_heartbeat(now); }

void GcsService::update(Instant now) {
    if (connection_.update(now)) {
        event_log_.record(Severity::Info,
                          connection_.connected() ? "drone connected" : "drone disconnected", now);
        if (!connection_.connected()) {
            // Stale data must not be presented as live.
            view_.goto_target.reset();
        }
    }
    view_.connected = connection_.connected();
}

void GcsService::arm() {
    transmitter_.transmit_arm();
    event_log_.record(Severity::Info, "ARM requested", Instant{});
}

void GcsService::disarm() {
    transmitter_.transmit_disarm();
    event_log_.record(Severity::Info, "DISARM requested", Instant{});
}

void GcsService::land() {
    transmitter_.transmit_land();
    event_log_.record(Severity::Info, "LAND requested", Instant{});
}

void GcsService::send_goto(const Vec3& target) {
    const Vec3 clamped = config_.fence.clamp(target);
    transmitter_.transmit_goto(clamped);
    view_.goto_target = clamped;
    event_log_.record(Severity::Info, "GOTO requested", Instant{});
}

void GcsService::send_manual(const Vec3& axes, Instant now) {
    const bool active = axes.norm() > 1e-9;
    if (!active && !manual_active_) {
        return;  // nothing held, nothing to release: stay silent on the wire
    }
    if (active && last_manual_sent_ != Instant::min() &&
        now - last_manual_sent_ < config_.manual_period) {
        return;
    }
    transmitter_.transmit_manual(axes);
    last_manual_sent_ = now;
    if (active) {
        view_.goto_target.reset();  // manual input overrides a pending GOTO
        manual_active_ = true;
    } else {
        manual_active_ = false;
    }
}

}  // namespace kratt::domain
