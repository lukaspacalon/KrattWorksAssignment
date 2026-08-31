#pragma once

#include <optional>

#include "domain/geofence.hpp"
#include "domain/types.hpp"

namespace kratt::domain {

/// Tuning constants of the simulated airframe.
/// A value object injected at construction: no global, no #define, and every
/// test can build its own airframe (e.g. a 15 m ceiling to test the take-off
/// edge case) without touching production defaults.
struct FlightConfig {
    double horizontal_speed_mps{10.0};  ///< specification: fixed 10 m/s
    double climb_speed_mps{10.0};
    double land_speed_mps{1.0};         ///< "gently descend"
    double takeoff_altitude_m{20.0};    ///< specification
    double disarm_altitude_m{0.5};      ///< specification: disarm only below 0.5 m
    double arrival_tolerance_m{0.25};
    Instant manual_timeout{std::chrono::milliseconds{500}};
    Geofence fence{100.0, 60.0};
};

/// The flight state machine and the 3-DOF kinematic simulation.
///
/// Deliberately holds no port and no collaborator: it is a pure state machine
/// driven by `step(dt, now)`. Given the same initial state and the same
/// sequence of (command, dt, now), it produces exactly the same trajectory —
/// which is what lets a full 10-minute flight be tested in microseconds.
///
/// State transitions:
///   Disarmed --arm--> Takeoff --altitude reached--> Hold
///   Hold --goto--> Goto --target reached--> Hold
///   Hold --manual--> Manual --input timeout--> Hold
///   any(armed) --land--> Land --touchdown--> Disarmed
///   Hold --disarm, alt < 0.5--> Disarmed
///   any(armed) --disarm, alt >= 0.5--> Land   (specification)
class FlightController {
public:
    explicit FlightController(FlightConfig config = {}) noexcept;

    // --- Commands. Pure decisions: they mutate state but perform no I/O. ---
    CommandResult arm() noexcept;
    CommandResult disarm() noexcept;
    CommandResult goto_target(const Vec3& target) noexcept;
    CommandResult land() noexcept;
    CommandResult manual_input(const Vec3& normalised_axes, Instant now) noexcept;

    /// Advances the simulation. `now` is used only for the manual-input
    /// timeout; nothing here reads a clock.
    void step(Seconds dt, Instant now) noexcept;

    [[nodiscard]] const Telemetry& telemetry() const noexcept { return telemetry_; }
    [[nodiscard]] const FlightConfig& config() const noexcept { return config_; }
    [[nodiscard]] std::optional<Vec3> target() const noexcept { return target_; }

    /// True when the last step had its position clamped by the fence.
    /// Exposed so the GUI and the tests can observe the constraint firing.
    [[nodiscard]] bool fence_engaged() const noexcept { return fence_engaged_; }

    /// Mode transition that occurred during the last `step`, if any.
    /// Lets the service emit an event without the controller owning a log port.
    [[nodiscard]] std::optional<FlightMode> consume_transition() noexcept;

private:
    [[nodiscard]] Vec3 desired_velocity(Instant now) const noexcept;
    void enter(FlightMode mode) noexcept;

    FlightConfig config_;
    Telemetry telemetry_{};
    std::optional<Vec3> target_{};
    Vec3 manual_axes_{};
    Instant last_manual_input_{};
    bool fence_engaged_{false};
    std::optional<FlightMode> transition_{};
};

}  // namespace kratt::domain
