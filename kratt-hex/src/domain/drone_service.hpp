#pragma once

#include "domain/flight_controller.hpp"
#include "domain/ports/event_log.hpp"
#include "domain/ports/flight_commands.hpp"
#include "domain/ports/telemetry_sink.hpp"

namespace kratt::domain {

/// Use-case layer of the drone.
///
/// It is the only class that touches both the inbound and the outbound ports:
///   * it *implements* IFlightCommands, so any primary adapter can drive it;
///   * it *depends on* ITelemetrySink and IEventLog, injected by reference at
///     construction, so it never knows what is on the other side.
///
/// The controller stays a pure state machine; the publication cadence and the
/// event emission live here. That separation is what keeps `FlightController`
/// testable with no collaborator at all.
class DroneService final : public IFlightCommands {
public:
    struct Config {
        FlightConfig flight{};
        /// Specification: telemetry at 10 Hz.
        Instant telemetry_period{std::chrono::milliseconds{100}};
    };

    DroneService(Config config, ITelemetrySink& telemetry_sink,
                 IEventLog& event_log = NullEventLog::instance());

    // --- IFlightCommands (inbound port) ------------------------------------
    CommandResult arm(Instant now) override;
    CommandResult disarm(Instant now) override;
    CommandResult goto_target(const Vec3& target, Instant now) override;
    CommandResult land(Instant now) override;
    CommandResult manual_input(const Vec3& axes, Instant now) override;

    /// One simulation tick. Advances the physics, emits mode-change events and
    /// publishes telemetry when the period has elapsed.
    /// Never blocks, never allocates in steady state.
    void step(Seconds dt, Instant now);

    [[nodiscard]] const Telemetry& telemetry() const noexcept { return controller_.telemetry(); }
    [[nodiscard]] const FlightController& controller() const noexcept { return controller_; }
    [[nodiscard]] std::uint64_t publications() const noexcept { return publications_; }

private:
    void record(Severity severity, std::string_view event, Instant now);

    Config config_;
    FlightController controller_;
    ITelemetrySink& telemetry_sink_;
    IEventLog& event_log_;

    Instant next_publication_{Instant::min()};
    std::uint64_t publications_{0};
};

}  // namespace kratt::domain
