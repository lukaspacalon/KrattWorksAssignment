#pragma once

#include "domain/flight_state.hpp"
#include "domain/kinematics.hpp"

/// Pure flight dynamics.
///
/// Every function is referentially transparent: it takes a state and returns a
/// new one, touching no member, no clock and no allocation. This is the layer
/// the "functional core" of the design refers to — it can be exhaustively
/// property-tested, and it cannot race with anything because it owns nothing.
namespace kratt::domain::dynamics {

/// The velocity the current mode asks for. Depends only on its arguments.
[[nodiscard]] inline Vec3 desired_velocity(const FlightState& state, const FlightConfig& config,
                                           Instant now) noexcept {
    switch (state.mode()) {
        case FlightMode::Takeoff:
            return state.altitude() < config.effective_takeoff_altitude()
                       ? Vec3{0.0, 0.0, config.climb_speed_mps}
                       : Vec3{};
        case FlightMode::Goto:
            return state.target ? kinematics::velocity_towards(state.telemetry.position,
                                                               *state.target,
                                                               config.horizontal_speed_mps,
                                                               config.arrival_tolerance_m)
                                : Vec3{};
        case FlightMode::Manual:
            // The timeout is also a state-machine event; checking it here keeps
            // the function total, so it is safe to call in any order.
            if (now - state.last_manual_input > config.manual_timeout) {
                return {};
            }
            return Vec3{state.manual_axes.x * config.horizontal_speed_mps,
                        state.manual_axes.y * config.horizontal_speed_mps,
                        state.manual_axes.z * config.climb_speed_mps};
        case FlightMode::Land:
            return Vec3{0.0, 0.0, -config.land_speed_mps};
        case FlightMode::Hold:
        case FlightMode::Disarmed:
            break;
    }
    return {};
}

struct StepResult {
    FlightState state;
    bool fence_engaged{false};
};

/// Advances the state by `dt`. Pure: same inputs, same output, always.
///
/// The geofence is applied as a projection of the integrated position, and the
/// reported velocity is then *derived* from the movement that actually
/// happened — so a drone pressed against the fence reports 0 m/s on that axis
/// instead of the 10 m/s it was commanded.
[[nodiscard]] inline StepResult integrate(FlightState state, const FlightConfig& config,
                                          Seconds dt, Instant now) noexcept {
    if (dt <= 0.0 || !state.armed()) {
        state.telemetry.velocity = {};
        return StepResult{state, false};
    }
    const Vec3 previous = state.telemetry.position;
    const Vec3 candidate =
        kinematics::integrate(previous, desired_velocity(state, config, now), dt);
    const Vec3 clamped = config.fence.clamp(candidate);

    state.telemetry.position = clamped;
    state.telemetry.velocity = kinematics::derive(clamped, previous, dt);
    return StepResult{state, !(clamped == candidate)};
}

}  // namespace kratt::domain::dynamics
