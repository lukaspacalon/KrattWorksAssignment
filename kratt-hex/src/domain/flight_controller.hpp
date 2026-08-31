#pragma once

#include "domain/flight_dynamics.hpp"
#include "domain/flight_state_machine.hpp"

namespace kratt::domain {

/// Thin stateful shell over the functional core.
///
/// It owns exactly one mutable thing — the current `FlightState` — and does no
/// computation of its own. Commands become state-machine events; `step` calls
/// the pure dynamics and then feeds back the events the new state implies.
///
/// The split is deliberate and is the whole point of the design:
///   * `fsm::kTransitions` declares *what may happen* (declarative);
///   * `dynamics::*` computes *how the aircraft moves* (pure functions);
///   * this class only sequences the two (imperative, and trivially small).
///
/// Consequences: the trajectory is reproducible bit for bit, the class holds no
/// thread, no clock and no port, and a full flight can be replayed in a test in
/// microseconds.
class FlightController {
public:
    explicit FlightController(FlightConfig config = {}) noexcept : config_{config} {}

    // --- Commands. Each is one line: build the event, apply the table. ------
    CommandResult arm() noexcept { return dispatch(fsm::Event::Arm, {}); }
    CommandResult disarm() noexcept { return dispatch(fsm::Event::Disarm, {}); }
    CommandResult land() noexcept { return dispatch(fsm::Event::Land, {}); }

    CommandResult goto_target(const Vec3& target) noexcept {
        return dispatch(fsm::Event::Goto, fsm::EventPayload{target, {}});
    }

    CommandResult manual_input(const Vec3& axes, Instant now) noexcept {
        const Vec3 clamped = kinematics::clamp_to_unit_ball(
            Vec3{clamp(axes.x, -1.0, 1.0), clamp(axes.y, -1.0, 1.0), clamp(axes.z, -1.0, 1.0)});
        return dispatch(fsm::manual_event(clamped), fsm::EventPayload{clamped, now});
    }

    /// One simulation tick. `now` is injected, never read from a clock.
    void step(Seconds dt, Instant now) noexcept;

    [[nodiscard]] const Telemetry& telemetry() const noexcept { return state_.telemetry; }
    [[nodiscard]] const FlightState& state() const noexcept { return state_; }
    [[nodiscard]] const FlightConfig& config() const noexcept { return config_; }
    [[nodiscard]] std::optional<Vec3> target() const noexcept { return state_.target; }
    [[nodiscard]] bool fence_engaged() const noexcept { return fence_engaged_; }

    /// Mode transition that occurred since the last call, if any. Lets the
    /// service emit an event without the controller owning a log port.
    [[nodiscard]] std::optional<FlightMode> consume_transition() noexcept {
        const auto transition = transition_;
        transition_.reset();
        return transition;
    }

private:
    CommandResult dispatch(fsm::Event event, const fsm::EventPayload& payload) noexcept {
        const fsm::Outcome outcome = fsm::apply(state_, config_, event, payload);
        state_ = outcome.state;
        if (outcome.mode_changed) {
            transition_ = state_.mode();
        }
        return outcome.result;
    }

    FlightConfig config_;
    FlightState state_{};
    bool fence_engaged_{false};
    std::optional<FlightMode> transition_{};
};

}  // namespace kratt::domain
