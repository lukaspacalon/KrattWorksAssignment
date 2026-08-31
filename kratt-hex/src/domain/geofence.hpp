#pragma once

#include "domain/types.hpp"

namespace kratt::domain {

/// Axis-aligned box geofence centred on the take-off point.
///
/// An immutable value object: it is injected into the flight controller by the
/// composition root, never mutated, and safe to copy. A box rather than a
/// cylinder because the clamp is exact and branch-free, and because "keep it
/// simple and basic" is an explicit requirement.
class Geofence {
public:
    constexpr Geofence(double half_extent_m, double ceiling_m) noexcept
        : half_extent_{half_extent_m}, ceiling_{ceiling_m} {}

    [[nodiscard]] constexpr double half_extent() const noexcept { return half_extent_; }
    [[nodiscard]] constexpr double ceiling() const noexcept { return ceiling_; }

    [[nodiscard]] constexpr bool contains(const Vec3& p) const noexcept {
        return p.x >= -half_extent_ && p.x <= half_extent_ &&
               p.y >= -half_extent_ && p.y <= half_extent_ &&
               p.z >= 0.0 && p.z <= ceiling_;
    }

    /// Projects a point onto the closest point inside the fence.
    /// Post-condition: `contains(clamp(p))` is always true.
    [[nodiscard]] constexpr Vec3 clamp(const Vec3& p) const noexcept {
        return Vec3{domain::clamp(p.x, -half_extent_, half_extent_),
                    domain::clamp(p.y, -half_extent_, half_extent_),
                    domain::clamp(p.z, 0.0, ceiling_)};
    }

private:
    double half_extent_;
    double ceiling_;
};

}  // namespace kratt::domain
