#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string_view>

/// Domain layer.
///
/// Nothing in this namespace may include a socket header, a MAVLink header, a
/// GUI header, or start a thread. The rule is enforced mechanically by the
/// `domain_purity` CTest target, not by discipline alone.
namespace kratt::domain {

/// Physical duration, in seconds. A plain double because the domain integrates
/// it; `std::chrono::duration` would force conversions on every multiplication.
using Seconds = double;

/// A monotonic instant with an arbitrary origin.
///
/// The domain never *reads* a clock: instants are always passed in as
/// arguments. Choosing `nanoseconds` rather than `steady_clock::time_point`
/// makes that explicit — there is no `now()` to call on this type, so the
/// compiler helps keep the domain deterministic.
using Instant = std::chrono::nanoseconds;

[[nodiscard]] constexpr Seconds to_seconds(Instant d) noexcept {
    return static_cast<Seconds>(d.count()) * 1e-9;
}

// ---------------------------------------------------------------------------

/// 3D vector. Immutable value object: every operation returns a new value.
///
/// Frame convention used throughout the domain:
///   x = north (m), y = east (m), z = altitude above take-off (m, up positive).
/// The NED sign flip required by MAVLink lives in the adapter, not here.
struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    [[nodiscard]] constexpr Vec3 operator+(const Vec3& o) const noexcept {
        return {x + o.x, y + o.y, z + o.z};
    }
    [[nodiscard]] constexpr Vec3 operator-(const Vec3& o) const noexcept {
        return {x - o.x, y - o.y, z - o.z};
    }
    [[nodiscard]] constexpr Vec3 operator*(double s) const noexcept {
        return {x * s, y * s, z * s};
    }

    [[nodiscard]] double norm() const noexcept { return std::sqrt(x * x + y * y + z * z); }

    [[nodiscard]] Vec3 normalized() const noexcept {
        const double n = norm();
        return n > 1e-9 ? Vec3{x / n, y / n, z / n} : Vec3{};
    }

    friend constexpr bool operator==(const Vec3&, const Vec3&) = default;
};

[[nodiscard]] constexpr double clamp(double v, double lo, double hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

// ---------------------------------------------------------------------------

/// Internal flight mode.
///
/// The specification only requires the two MAV_MODE values; those are derived
/// from `armed` in the adapter. This enum is the finer internal state, and it
/// is a domain concept — it exists whether or not MAVLink does.
enum class FlightMode : std::uint8_t {
    Disarmed,
    Takeoff,
    Hold,
    Goto,
    Manual,
    Land,
};

[[nodiscard]] constexpr std::string_view to_string(FlightMode mode) noexcept {
    switch (mode) {
        case FlightMode::Disarmed: return "DISARMED";
        case FlightMode::Takeoff:  return "TAKEOFF";
        case FlightMode::Hold:     return "HOLD";
        case FlightMode::Goto:     return "GOTO";
        case FlightMode::Manual:   return "MANUAL";
        case FlightMode::Land:     return "LAND";
    }
    return "UNKNOWN";
}

/// Immutable snapshot of the drone state. This is the DTO that crosses the
/// outbound port; adapters may not see anything richer.
struct Telemetry {
    Vec3 position{};
    Vec3 velocity{};
    FlightMode mode{FlightMode::Disarmed};
    bool armed{false};

    friend constexpr bool operator==(const Telemetry&, const Telemetry&) = default;
};

/// Outcome of a command. `AcceptedAsLand` exists so the "disarm while flying"
/// rule is reported honestly instead of being flattened into a plain success.
enum class CommandResult : std::uint8_t {
    Accepted,
    AcceptedAsLand,
    Rejected,
};

[[nodiscard]] constexpr std::string_view to_string(CommandResult r) noexcept {
    switch (r) {
        case CommandResult::Accepted:       return "ACCEPTED";
        case CommandResult::AcceptedAsLand: return "ACCEPTED_AS_LAND";
        case CommandResult::Rejected:       return "REJECTED";
    }
    return "UNKNOWN";
}

}  // namespace kratt::domain
