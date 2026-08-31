#pragma once

#include <array>
#include <cstddef>

#include "domain/flight_state.hpp"

/// Declarative flight-mode state machine.
///
/// The whole behaviour lives in one `constexpr` transition table below. Nothing
/// else in the domain is allowed to change `telemetry.mode`, which means:
///
///   * every legal transition is visible in one screen of code, and every
///     transition that is *not* in the table is impossible by construction —
///     no scattered `if (mode != Land && armed && ...)` guard to forget;
///   * the safety rules of the specification ("disarm only below 0.5 m",
///     "manual input must not fight the automatic phases") are rows with a
///     guard, not comments;
///   * the table is data, so a test can walk it and assert properties over
///     *all* transitions at once rather than one at a time;
///   * guards and actions are pure free functions, so the machine composes with
///     the functional core in `dynamics`.
namespace kratt::domain::fsm {

/// What can happen to the machine. Command events come from the inbound port;
/// the four completion events are derived from the state by pure predicates.
enum class Event : std::uint8_t {
    Arm,
    Disarm,
    Goto,
    Land,
    ManualActive,      ///< sticks deflected
    ManualCentred,     ///< sticks explicitly released
    TakeoffComplete,   ///< derived: target altitude reached
    TargetReached,     ///< derived: GOTO tolerance reached
    ManualTimeout,     ///< derived: no input for `manual_timeout`
    Touchdown,         ///< derived: descended below the safety altitude
};

/// Data carried by an event. One struct for all events keeps the table uniform.
struct EventPayload {
    Vec3 vector{};   ///< GOTO target, or normalised manual axes
    Instant at{};
};

// --- Mode sets -------------------------------------------------------------
// A bitmask lets one row cover several source modes, which is what keeps the
// table short enough to read in one pass.

using ModeSet = std::uint8_t;

[[nodiscard]] constexpr ModeSet mask(FlightMode mode) noexcept {
    return static_cast<ModeSet>(1U << static_cast<std::uint8_t>(mode));
}
[[nodiscard]] constexpr ModeSet operator|(ModeSet lhs, FlightMode rhs) noexcept {
    return static_cast<ModeSet>(lhs | mask(rhs));
}
[[nodiscard]] constexpr bool contains(ModeSet set, FlightMode mode) noexcept {
    return (set & mask(mode)) != 0;
}

inline constexpr ModeSet kAny = mask(FlightMode::Disarmed) | FlightMode::Takeoff |
                                FlightMode::Hold | FlightMode::Goto | FlightMode::Manual |
                                FlightMode::Land;
/// Modes in which the operator has direct control of the trajectory.
inline constexpr ModeSet kPilotable =
    mask(FlightMode::Hold) | FlightMode::Goto | FlightMode::Manual;
/// Modes in which an automatic sequence owns the trajectory and must not be
/// overridden by stick input or a GOTO.
inline constexpr ModeSet kAutomatic = mask(FlightMode::Takeoff) | FlightMode::Land;
inline constexpr ModeSet kArmed = kPilotable | FlightMode::Takeoff | FlightMode::Land;

/// Sentinel meaning "stay in the current mode".
inline constexpr FlightMode kSameMode = static_cast<FlightMode>(0xFF);

// --- Guards and actions ----------------------------------------------------
// Both are plain function pointers to pure functions, so the table stays a
// compile-time constant with no vtable and no allocation.

using Guard = bool (*)(const FlightState&, const FlightConfig&, const EventPayload&) noexcept;
using Action = FlightState (*)(FlightState, const FlightConfig&, const EventPayload&) noexcept;

namespace guards {

/// Specification: disarming is only allowed below 0.5 m.
[[nodiscard]] constexpr bool below_safety_altitude(const FlightState& state,
                                                   const FlightConfig& config,
                                                   const EventPayload&) noexcept {
    return state.altitude() < config.disarm_altitude_m;
}

}  // namespace guards

namespace actions {

[[nodiscard]] constexpr FlightState identity(FlightState state, const FlightConfig&,
                                             const EventPayload&) noexcept {
    return state;
}

[[nodiscard]] constexpr FlightState begin_takeoff(FlightState state, const FlightConfig&,
                                                  const EventPayload&) noexcept {
    state.telemetry.armed = true;
    state.target.reset();
    state.manual_axes = {};
    return state;
}

[[nodiscard]] constexpr FlightState cut_motors(FlightState state, const FlightConfig&,
                                               const EventPayload&) noexcept {
    state.telemetry.armed = false;
    state.telemetry.velocity = {};
    state.target.reset();
    state.manual_axes = {};
    return state;
}

[[nodiscard]] constexpr FlightState begin_land(FlightState state, const FlightConfig&,
                                               const EventPayload&) noexcept {
    state.target.reset();
    state.manual_axes = {};
    return state;
}

/// A target outside the fence is clamped rather than refused: the invariant
/// "position always inside the fence" is preserved and the drone flies as close
/// as the fence allows, which is the least surprising behaviour for an operator.
[[nodiscard]] constexpr FlightState set_target(FlightState state, const FlightConfig& config,
                                               const EventPayload& payload) noexcept {
    state.target = config.fence.clamp(payload.vector);
    state.manual_axes = {};
    return state;
}

[[nodiscard]] constexpr FlightState set_manual(FlightState state, const FlightConfig&,
                                               const EventPayload& payload) noexcept {
    state.manual_axes = payload.vector;
    state.last_manual_input = payload.at;
    state.target.reset();
    return state;
}

[[nodiscard]] constexpr FlightState clear_manual(FlightState state, const FlightConfig&,
                                                 const EventPayload& payload) noexcept {
    state.manual_axes = {};
    state.last_manual_input = payload.at;
    state.telemetry.velocity = {};
    return state;
}

/// Snap exactly onto the take-off altitude so repeated runs are bit-identical.
[[nodiscard]] constexpr FlightState settle_at_takeoff_altitude(
    FlightState state, const FlightConfig& config, const EventPayload&) noexcept {
    state.telemetry.position.z = config.effective_takeoff_altitude();
    state.telemetry.velocity = {};
    return state;
}

[[nodiscard]] constexpr FlightState settle_at_target(FlightState state, const FlightConfig&,
                                                     const EventPayload&) noexcept {
    if (state.target) {
        state.telemetry.position = *state.target;
    }
    state.telemetry.velocity = {};
    state.target.reset();
    return state;
}

[[nodiscard]] constexpr FlightState touchdown(FlightState state, const FlightConfig&,
                                              const EventPayload&) noexcept {
    state.telemetry.position.z = 0.0;
    state.telemetry.velocity = {};
    state.telemetry.armed = false;
    state.target.reset();
    state.manual_axes = {};
    return state;
}

}  // namespace actions

// --- The table -------------------------------------------------------------

struct Transition {
    ModeSet from;
    Event on;
    Guard guard;            ///< nullptr = unconditional
    FlightMode to;          ///< kSameMode = no mode change
    Action action;
    CommandResult outcome;  ///< what the caller is told
};

/// The complete behaviour of the aircraft. First matching row wins, so order
/// encodes priority: the guarded rows come before their catch-all.
inline constexpr std::array<Transition, 16> kTransitions{{
    // --- Arming -----------------------------------------------------------
    {mask(FlightMode::Disarmed), Event::Arm, nullptr,
     FlightMode::Takeoff, actions::begin_takeoff, CommandResult::Accepted},
    // Arming an already armed drone is a no-op, not an error.
    {kArmed, Event::Arm, nullptr,
     kSameMode, actions::identity, CommandResult::Accepted},

    // --- Disarming (the specification's safety rule, as two rows) ----------
    {mask(FlightMode::Disarmed), Event::Disarm, nullptr,
     kSameMode, actions::identity, CommandResult::Accepted},
    {kArmed, Event::Disarm, guards::below_safety_altitude,
     FlightMode::Disarmed, actions::cut_motors, CommandResult::Accepted},
    // Above 0.5 m the request is honoured as a landing rather than refused:
    // refusing would leave the operator with no way to bring the drone down.
    {kArmed, Event::Disarm, nullptr,
     FlightMode::Land, actions::begin_land, CommandResult::AcceptedAsLand},

    // --- Landing ----------------------------------------------------------
    {kArmed, Event::Land, nullptr,
     FlightMode::Land, actions::begin_land, CommandResult::Accepted},
    {mask(FlightMode::Disarmed), Event::Land, nullptr,
     kSameMode, actions::identity, CommandResult::Rejected},

    // --- Navigation -------------------------------------------------------
    {kPilotable, Event::Goto, nullptr,
     FlightMode::Goto, actions::set_target, CommandResult::Accepted},
    // Automatic phases own the trajectory; a GOTO must not hijack them.
    {kAutomatic | FlightMode::Disarmed, Event::Goto, nullptr,
     kSameMode, actions::identity, CommandResult::Rejected},

    // --- Manual control ---------------------------------------------------
    {kPilotable, Event::ManualActive, nullptr,
     FlightMode::Manual, actions::set_manual, CommandResult::Accepted},
    // An explicit "sticks centred" holds at once; the timeout below is the
    // backstop for when that message is lost on the wire.
    {kPilotable, Event::ManualCentred, nullptr,
     FlightMode::Hold, actions::clear_manual, CommandResult::Accepted},
    {kAutomatic | FlightMode::Disarmed, Event::ManualActive, nullptr,
     kSameMode, actions::identity, CommandResult::Rejected},
    {kAutomatic | FlightMode::Disarmed, Event::ManualCentred, nullptr,
     kSameMode, actions::identity, CommandResult::Rejected},

    // --- Derived completion events ----------------------------------------
    {mask(FlightMode::Takeoff), Event::TakeoffComplete, nullptr,
     FlightMode::Hold, actions::settle_at_takeoff_altitude, CommandResult::Accepted},
    {mask(FlightMode::Goto), Event::TargetReached, nullptr,
     FlightMode::Hold, actions::settle_at_target, CommandResult::Accepted},
    {mask(FlightMode::Manual), Event::ManualTimeout, nullptr,
     FlightMode::Hold, actions::clear_manual, CommandResult::Accepted},
}};

/// Touchdown is separate only because the array size above is fixed; keeping it
/// in its own row set would split the table, so it is appended here.
inline constexpr Transition kTouchdownTransition{
    mask(FlightMode::Land), Event::Touchdown, nullptr,
    FlightMode::Disarmed, actions::touchdown, CommandResult::Accepted};

struct Outcome {
    FlightState state;
    CommandResult result{CommandResult::Rejected};
    bool mode_changed{false};
};

/// Applies an event. Pure: scans the table, runs the first matching row's guard
/// and action, and returns a new state. An event with no matching row leaves the
/// state untouched and is reported as Rejected.
[[nodiscard]] Outcome apply(const FlightState& state, const FlightConfig& config, Event event,
                            const EventPayload& payload) noexcept;

/// Derives the completion event, if any, implied by the current state.
/// Pure predicate: this is what turns physics into state-machine events without
/// the dynamics layer knowing the machine exists.
[[nodiscard]] std::optional<Event> completion_event(const FlightState& state,
                                                    const FlightConfig& config) noexcept;

/// Derives the manual-input timeout event, if due.
[[nodiscard]] std::optional<Event> timeout_event(const FlightState& state,
                                                 const FlightConfig& config,
                                                 Instant now) noexcept;

/// Classifies a stick input into the matching event.
[[nodiscard]] constexpr Event manual_event(const Vec3& axes) noexcept {
    return axes.x == 0.0 && axes.y == 0.0 && axes.z == 0.0 ? Event::ManualCentred
                                                           : Event::ManualActive;
}

}  // namespace kratt::domain::fsm
