#include "domain/drone_service.hpp"

namespace kratt::domain {

DroneService::DroneService(Config config, ITelemetrySink& telemetry_sink, IEventLog& event_log)
    : config_{config},
      controller_{config.flight},
      telemetry_sink_{telemetry_sink},
      event_log_{event_log} {}

void DroneService::record(Severity severity, std::string_view event, Instant now) {
    event_log_.record(severity, event, now);
}

CommandResult DroneService::arm(Instant now) {
    const CommandResult result = controller_.arm();
    record(Severity::Info, "ARM", now);
    return result;
}

CommandResult DroneService::disarm(Instant now) {
    const CommandResult result = controller_.disarm();
    if (result == CommandResult::AcceptedAsLand) {
        record(Severity::Warn, "DISARM above safety altitude, entering LAND", now);
    } else {
        record(Severity::Info, "DISARM", now);
    }
    return result;
}

CommandResult DroneService::goto_target(const Vec3& target, Instant now) {
    const CommandResult result = controller_.goto_target(target);
    record(result == CommandResult::Accepted ? Severity::Info : Severity::Warn, "GOTO", now);
    return result;
}

CommandResult DroneService::land(Instant now) {
    const CommandResult result = controller_.land();
    record(Severity::Info, "LAND", now);
    return result;
}

CommandResult DroneService::manual_input(const Vec3& axes, Instant now) {
    return controller_.manual_input(axes, now);  // too frequent to log at Info
}

void DroneService::step(Seconds dt, Instant now) {
    controller_.step(dt, now);

    if (const auto transition = controller_.consume_transition()) {
        record(Severity::Info, to_string(*transition), now);
    }

    // First call arms the schedule on the caller's timeline, so the service has
    // no notion of a start-up instant and stays independent of any clock.
    if (next_publication_ == Instant::min()) {
        next_publication_ = now;
    }
    if (now < next_publication_) {
        return;
    }
    // Absolute scheduling keeps the average rate at exactly 10 Hz; a single
    // overrun is absorbed rather than turned into a catch-up burst.
    next_publication_ += config_.telemetry_period;
    if (next_publication_ <= now) {
        next_publication_ = now + config_.telemetry_period;
    }

    ++publications_;
    telemetry_sink_.publish(controller_.telemetry(), now);
}

}  // namespace kratt::domain
