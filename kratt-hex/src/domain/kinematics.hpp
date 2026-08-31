#pragma once

#include "domain/types.hpp"

/// Pure kinematics.
///
/// Every function here is referentially transparent: same arguments, same
/// result, no hidden state, no clock, no allocation. They are `constexpr` so
/// the compiler can evaluate them at build time and so the unit tests can
/// assert on them with `static_assert`.
namespace kratt::domain::kinematics {

/// Forward Euler integration: p(t+dt) = p(t) + v * dt.
[[nodiscard]] constexpr Vec3 integrate(const Vec3& position, const Vec3& velocity,
                                       Seconds dt) noexcept {
    return position + velocity * dt;
}

/// Backward finite difference: v ≈ (p_now - p_prev) / dt.
///
/// Used to report the velocity actually achieved after the geofence clamp, so a
/// drone pressed against the fence reports 0 m/s on that axis instead of the
/// commanded 10 m/s. Also the building block for an acceleration-plausibility
/// check, should one be added later — same function, applied to velocities.
[[nodiscard]] constexpr Vec3 derive(const Vec3& current, const Vec3& previous,
                                    Seconds dt) noexcept {
    return dt > 0.0 ? (current - previous) * (1.0 / dt) : Vec3{};
}

/// Velocity vector of magnitude `speed` pointing from `from` to `to`.
/// Returns the null vector when already within `tolerance` of the target.
[[nodiscard]] inline Vec3 velocity_towards(const Vec3& from, const Vec3& to, double speed,
                                           double tolerance) noexcept {
    const Vec3 delta = to - from;
    return delta.norm() <= tolerance ? Vec3{} : delta.normalized() * speed;
}

/// Clamps a normalised stick input to the unit ball, so that a diagonal never
/// exceeds the speed envelope.
[[nodiscard]] inline Vec3 clamp_to_unit_ball(const Vec3& axes) noexcept {
    const double n = axes.norm();
    return n > 1.0 ? axes * (1.0 / n) : axes;
}

}  // namespace kratt::domain::kinematics
