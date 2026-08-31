#include "domain/flight_state_machine.hpp"

namespace kratt::domain::fsm {
namespace {

/// Table scan shared by the array and the appended touchdown row.
[[nodiscard]] const Transition* find(const FlightState& state, const FlightConfig& config,
                                     Event event, const EventPayload& payload) noexcept {
    for (const auto& transition : kTransitions) {
        if (transition.on != event || !contains(transition.from, state.mode())) {
            continue;
        }
        if (transition.guard != nullptr && !transition.guard(state, config, payload)) {
            continue;
        }
        return &transition;
    }
    if (kTouchdownTransition.on == event && contains(kTouchdownTransition.from, state.mode())) {
        return &kTouchdownTransition;
    }
    return nullptr;
}

}  // namespace

Outcome apply(const FlightState& state, const FlightConfig& config, Event event,
              const EventPayload& payload) noexcept {
    const Transition* transition = find(state, config, event, payload);
    if (transition == nullptr) {
        // No row matched: the event is not legal here. The state is returned
        // untouched, so an unhandled event can never corrupt the aircraft.
        return Outcome{state, CommandResult::Rejected, false};
    }

    FlightState next = transition->action(state, config, payload);
    bool mode_changed = false;
    if (transition->to != kSameMode && transition->to != state.mode()) {
        next.telemetry.mode = transition->to;
        mode_changed = true;
    }
    return Outcome{next, transition->outcome, mode_changed};
}

std::optional<Event> completion_event(const FlightState& state,
                                      const FlightConfig& config) noexcept {
    switch (state.mode()) {
        case FlightMode::Takeoff:
            return state.altitude() >= config.effective_takeoff_altitude() - 1e-9
                       ? std::optional{Event::TakeoffComplete}
                       : std::nullopt;
        case FlightMode::Goto:
            return state.target && (*state.target - state.telemetry.position).norm() <=
                                       config.arrival_tolerance_m
                       ? std::optional{Event::TargetReached}
                       : std::nullopt;
        case FlightMode::Land:
            return state.altitude() < config.disarm_altitude_m ? std::optional{Event::Touchdown}
                                                               : std::nullopt;
        default:
            return std::nullopt;
    }
}

std::optional<Event> timeout_event(const FlightState& state, const FlightConfig& config,
                                   Instant now) noexcept {
    if (state.mode() == FlightMode::Manual &&
        now - state.last_manual_input > config.manual_timeout) {
        return Event::ManualTimeout;
    }
    return std::nullopt;
}

}  // namespace kratt::domain::fsm
