#include "domain/flight_controller.hpp"

namespace kratt::domain {

void FlightController::step(Seconds dt, Instant now) noexcept {
    // 1. Time-driven event first, so the velocity used for integration always
    //    matches the mode that will be reported.
    if (const auto timeout = fsm::timeout_event(state_, config_, now)) {
        (void)dispatch(*timeout, fsm::EventPayload{{}, now});
    }

    // 2. Pure physics. The controller does no arithmetic itself.
    const auto result = dynamics::integrate(state_, config_, dt, now);
    state_ = result.state;
    fence_engaged_ = result.fence_engaged;

    // 3. Feed the new state back into the machine: reaching an altitude or a
    //    target is an event like any other, handled by the same table.
    if (const auto completion = fsm::completion_event(state_, config_)) {
        (void)dispatch(*completion, fsm::EventPayload{{}, now});
    }
}

}  // namespace kratt::domain
