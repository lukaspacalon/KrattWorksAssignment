#include "domain/flight_controller.hpp"

#include <algorithm>

#include "domain/kinematics.hpp"

namespace kratt::domain {

FlightController::FlightController(FlightConfig config) noexcept : config_{config} {}

void FlightController::enter(FlightMode mode) noexcept {
    if (telemetry_.mode == mode) {
        return;
    }
    telemetry_.mode = mode;
    transition_ = mode;
}

std::optional<FlightMode> FlightController::consume_transition() noexcept {
    const auto transition = transition_;
    transition_.reset();
    return transition;
}

CommandResult FlightController::arm() noexcept {
    if (telemetry_.armed) {
        return CommandResult::Accepted;  // idempotent
    }
    telemetry_.armed = true;
    target_.reset();
    manual_axes_ = {};
    enter(FlightMode::Takeoff);
    return CommandResult::Accepted;
}

CommandResult FlightController::disarm() noexcept {
    if (!telemetry_.armed) {
        return CommandResult::Accepted;
    }
    // Specification: disarming is only allowed below 0.5 m. Above it, the
    // request becomes a landing rather than a refusal — refusing outright would
    // leave an operator with no way to bring the aircraft down.
    if (telemetry_.position.z >= config_.disarm_altitude_m) {
        land();
        return CommandResult::AcceptedAsLand;
    }
    telemetry_.armed = false;
    telemetry_.velocity = {};
    target_.reset();
    manual_axes_ = {};
    enter(FlightMode::Disarmed);
    return CommandResult::Accepted;
}

CommandResult FlightController::goto_target(const Vec3& target) noexcept {
    if (!telemetry_.armed || telemetry_.mode == FlightMode::Land ||
        telemetry_.mode == FlightMode::Takeoff) {
        return CommandResult::Rejected;
    }
    // Clamped rather than rejected: the invariant "position always inside the
    // fence" is preserved, and the drone flies as close as the fence allows,
    // which is the least surprising behaviour for an operator.
    target_ = config_.fence.clamp(target);
    manual_axes_ = {};
    enter(FlightMode::Goto);
    return CommandResult::Accepted;
}

CommandResult FlightController::land() noexcept {
    if (!telemetry_.armed) {
        return CommandResult::Rejected;
    }
    target_.reset();
    manual_axes_ = {};
    enter(FlightMode::Land);
    return CommandResult::Accepted;
}

CommandResult FlightController::manual_input(const Vec3& axes, Instant now) noexcept {
    // Manual input must not fight the automatic phases.
    if (!telemetry_.armed || telemetry_.mode == FlightMode::Land ||
        telemetry_.mode == FlightMode::Takeoff) {
        return CommandResult::Rejected;
    }
    manual_axes_ = kinematics::clamp_to_unit_ball(
        Vec3{clamp(axes.x, -1.0, 1.0), clamp(axes.y, -1.0, 1.0), clamp(axes.z, -1.0, 1.0)});
    last_manual_input_ = now;

    if (manual_axes_.norm() < 1e-9) {
        // Explicit "sticks centred": hold at once instead of waiting for the
        // timeout. The timeout remains the backstop if this message is lost.
        enter(FlightMode::Hold);
        return CommandResult::Accepted;
    }
    target_.reset();
    enter(FlightMode::Manual);
    return CommandResult::Accepted;
}

Vec3 FlightController::desired_velocity(Instant now) const noexcept {
    switch (telemetry_.mode) {
        case FlightMode::Takeoff: {
            const double ceiling = std::min(config_.takeoff_altitude_m, config_.fence.ceiling());
            return telemetry_.position.z < ceiling ? Vec3{0.0, 0.0, config_.climb_speed_mps}
                                                   : Vec3{};
        }
        case FlightMode::Goto:
            return target_ ? kinematics::velocity_towards(telemetry_.position, *target_,
                                                          config_.horizontal_speed_mps,
                                                          config_.arrival_tolerance_m)
                           : Vec3{};
        case FlightMode::Manual:
            if (now - last_manual_input_ > config_.manual_timeout) {
                return {};
            }
            return Vec3{manual_axes_.x * config_.horizontal_speed_mps,
                        manual_axes_.y * config_.horizontal_speed_mps,
                        manual_axes_.z * config_.climb_speed_mps};
        case FlightMode::Land:
            return Vec3{0.0, 0.0, -config_.land_speed_mps};
        case FlightMode::Hold:
        case FlightMode::Disarmed:
            break;
    }
    return {};
}

void FlightController::step(Seconds dt, Instant now) noexcept {
    if (dt <= 0.0) {
        return;
    }
    if (!telemetry_.armed) {
        telemetry_.velocity = {};
        fence_engaged_ = false;
        return;
    }

    // Time-driven transitions are evaluated first, so the velocity used for
    // integration always matches the mode that will be reported.
    if (telemetry_.mode == FlightMode::Manual &&
        now - last_manual_input_ > config_.manual_timeout) {
        manual_axes_ = {};
        enter(FlightMode::Hold);
    }

    const Vec3 previous = telemetry_.position;
    const Vec3 candidate = kinematics::integrate(previous, desired_velocity(now), dt);
    const Vec3 clamped = config_.fence.clamp(candidate);

    fence_engaged_ = !(clamped == candidate);
    telemetry_.position = clamped;
    // Report the velocity actually achieved. A drone held against the fence
    // must not report 10 m/s on an axis it is not moving along.
    telemetry_.velocity = kinematics::derive(clamped, previous, dt);

    switch (telemetry_.mode) {
        case FlightMode::Takeoff: {
            const double ceiling = std::min(config_.takeoff_altitude_m, config_.fence.ceiling());
            if (telemetry_.position.z >= ceiling - 1e-9) {
                telemetry_.position.z = ceiling;
                telemetry_.velocity = {};
                enter(FlightMode::Hold);
            }
            break;
        }
        case FlightMode::Goto:
            if (target_ && (*target_ - telemetry_.position).norm() <= config_.arrival_tolerance_m) {
                telemetry_.position = *target_;
                telemetry_.velocity = {};
                target_.reset();
                enter(FlightMode::Hold);
            }
            break;
        case FlightMode::Land:
            if (telemetry_.position.z < config_.disarm_altitude_m) {
                telemetry_.position.z = 0.0;
                telemetry_.velocity = {};
                telemetry_.armed = false;
                enter(FlightMode::Disarmed);
            }
            break;
        case FlightMode::Hold:
        case FlightMode::Manual:
        case FlightMode::Disarmed:
            break;
    }
}

}  // namespace kratt::domain
