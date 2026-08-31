#pragma once

#include <optional>

#include "domain/geofence.hpp"
#include "domain/types.hpp"

namespace kratt::domain {

/// Tuning constants of the simulated airframe. Immutable value object injected
/// at construction: no global, no #define, and a test can build its own
/// airframe without touching production defaults.
struct FlightConfig {
    double horizontal_speed_mps{10.0};  ///< specification: fixed 10 m/s
    double climb_speed_mps{10.0};
    double land_speed_mps{1.0};         ///< "gently descend"
    double takeoff_altitude_m{20.0};    ///< specification
    double disarm_altitude_m{0.5};      ///< specification: disarm only below 0.5 m
    double arrival_tolerance_m{0.25};
    Instant manual_timeout{std::chrono::milliseconds{500}};
    Geofence fence{100.0, 60.0};

    /// The take-off ceiling actually reachable: the fence always wins.
    [[nodiscard]] constexpr double effective_takeoff_altitude() const noexcept {
        return takeoff_altitude_m < fence.ceiling() ? takeoff_altitude_m : fence.ceiling();
    }
};

/// The complete flight state, as a plain value.
///
/// A struct rather than a class with invariants, on purpose: every function that
/// transforms it is pure and takes it by value, so there is no encapsulation to
/// protect. `telemetry` is nested rather than duplicated, so the published
/// projection and the internal state can never drift apart.
struct FlightState {
    Telemetry telemetry{};
    std::optional<Vec3> target{};   ///< active GOTO target, if any
    Vec3 manual_axes{};             ///< last stick input, normalised
    Instant last_manual_input{};

    [[nodiscard]] constexpr FlightMode mode() const noexcept { return telemetry.mode; }
    [[nodiscard]] constexpr bool armed() const noexcept { return telemetry.armed; }
    [[nodiscard]] constexpr double altitude() const noexcept { return telemetry.position.z; }
};

}  // namespace kratt::domain
