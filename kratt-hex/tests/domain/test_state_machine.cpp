#include <gtest/gtest.h>

#include <set>
#include <vector>

#include "domain/flight_dynamics.hpp"
#include "domain/flight_state_machine.hpp"

namespace kratt::domain {
namespace {

using fsm::Event;
using fsm::EventPayload;

constexpr std::array kAllModes{FlightMode::Disarmed, FlightMode::Takeoff, FlightMode::Hold,
                               FlightMode::Goto,     FlightMode::Manual,  FlightMode::Land};

constexpr std::array kAllEvents{Event::Arm,             Event::Disarm,       Event::Goto,
                                Event::Land,            Event::ManualActive, Event::ManualCentred,
                                Event::TakeoffComplete, Event::TargetReached, Event::ManualTimeout,
                                Event::Touchdown};

/// Builds a plausible state for a given mode, so the property sweeps below
/// exercise realistic inputs rather than impossible ones.
FlightState state_in(FlightMode mode, double altitude = 20.0) {
    FlightState state;
    state.telemetry.mode = mode;
    state.telemetry.armed = mode != FlightMode::Disarmed;
    state.telemetry.position = Vec3{0.0, 0.0, mode == FlightMode::Disarmed ? 0.0 : altitude};
    if (mode == FlightMode::Goto) {
        state.target = Vec3{50.0, 0.0, altitude};
    }
    if (mode == FlightMode::Manual) {
        state.manual_axes = Vec3{1.0, 0.0, 0.0};
    }
    return state;
}

// ---------------------------------------------------------------------------
// Properties of the table itself. These are only possible because the machine
// is data: they assert over *every* transition at once, not one case at a time.
// ---------------------------------------------------------------------------

TEST(TransitionTableTest, EveryRowHasAnAction) {
    for (const auto& transition : fsm::kTransitions) {
        EXPECT_NE(transition.action, nullptr);
    }
    EXPECT_NE(fsm::kTouchdownTransition.action, nullptr);
}

TEST(TransitionTableTest, NoRowIsUnreachable) {
    // A row whose source set is empty could never fire and would be dead
    // behaviour hiding in the table.
    for (const auto& transition : fsm::kTransitions) {
        EXPECT_NE(transition.from, 0U);
    }
}

TEST(TransitionTableTest, GuardedRowsPrecedeTheirCatchAll) {
    // First match wins, so a guarded row placed after its unguarded twin would
    // be dead. This walks the table and fails if that ever happens.
    std::set<std::pair<int, int>> unguarded_seen;
    for (const auto& transition : fsm::kTransitions) {
        const auto key = std::pair{static_cast<int>(transition.on),
                                   static_cast<int>(transition.from)};
        if (transition.guard == nullptr) {
            unguarded_seen.insert(key);
        } else {
            EXPECT_FALSE(unguarded_seen.contains(key))
                << "a guarded row is shadowed by an earlier unguarded one";
        }
    }
}

TEST(TransitionTableTest, NoEventCanEverArmTheDroneExceptArm) {
    // The safety property that matters most: nothing but an explicit ARM may
    // start the motors. Swept over the whole cross product of modes and events.
    const FlightConfig config;
    for (const auto mode : kAllModes) {
        for (const auto event : kAllEvents) {
            if (event == Event::Arm) {
                continue;
            }
            const FlightState before = state_in(mode, 0.0);
            const auto outcome = fsm::apply(before, config, event, EventPayload{});
            if (!before.armed()) {
                EXPECT_FALSE(outcome.state.armed())
                    << "event " << static_cast<int>(event) << " armed a disarmed drone in mode "
                    << to_string(mode);
            }
        }
    }
}

TEST(TransitionTableTest, DisarmedStateIsAlwaysConsistentWithArmedFlag) {
    // Invariant: mode == Disarmed if and only if armed == false.
    const FlightConfig config;
    for (const auto mode : kAllModes) {
        for (const auto event : kAllEvents) {
            for (const double altitude : {0.0, 0.4, 0.6, 20.0}) {
                const auto outcome =
                    fsm::apply(state_in(mode, altitude), config, event, EventPayload{});
                EXPECT_EQ(outcome.state.mode() == FlightMode::Disarmed,
                          !outcome.state.armed())
                    << "inconsistent after event " << static_cast<int>(event) << " in "
                    << to_string(mode) << " at " << altitude << " m";
            }
        }
    }
}

TEST(TransitionTableTest, UnmatchedEventLeavesTheStateUntouched) {
    const FlightConfig config;
    // TakeoffComplete is only legal in Takeoff; anywhere else it must be inert.
    for (const auto mode : kAllModes) {
        if (mode == FlightMode::Takeoff) {
            continue;
        }
        const FlightState before = state_in(mode);
        const auto outcome = fsm::apply(before, config, Event::TakeoffComplete, EventPayload{});
        EXPECT_EQ(outcome.result, CommandResult::Rejected);
        EXPECT_EQ(outcome.state.telemetry, before.telemetry);
        EXPECT_FALSE(outcome.mode_changed);
    }
}

// ---------------------------------------------------------------------------
// The specification's safety rule, expressed directly against the table.
// ---------------------------------------------------------------------------

TEST(DisarmRuleTest, DisarmBelowSafetyAltitudeCutsTheMotors) {
    const FlightConfig config;
    const auto outcome = fsm::apply(state_in(FlightMode::Hold, 0.4), config, Event::Disarm,
                                    EventPayload{});
    EXPECT_EQ(outcome.result, CommandResult::Accepted);
    EXPECT_EQ(outcome.state.mode(), FlightMode::Disarmed);
    EXPECT_FALSE(outcome.state.armed());
}

TEST(DisarmRuleTest, DisarmAtOrAboveSafetyAltitudeBecomesALanding) {
    const FlightConfig config;
    // Boundary is documented as strictly-less-than, so exactly 0.5 m must land.
    for (const double altitude : {0.5, 0.51, 20.0}) {
        const auto outcome = fsm::apply(state_in(FlightMode::Hold, altitude), config,
                                        Event::Disarm, EventPayload{});
        EXPECT_EQ(outcome.result, CommandResult::AcceptedAsLand) << "at " << altitude << " m";
        EXPECT_EQ(outcome.state.mode(), FlightMode::Land);
        EXPECT_TRUE(outcome.state.armed()) << "the motors must not be cut in flight";
    }
}

TEST(DisarmRuleTest, NoModeCanBeDisarmedInFlightInOneStep) {
    // Swept over every airborne mode: none may go straight to Disarmed.
    const FlightConfig config;
    for (const auto mode : kAllModes) {
        if (mode == FlightMode::Disarmed) {
            continue;
        }
        const auto outcome =
            fsm::apply(state_in(mode, 10.0), config, Event::Disarm, EventPayload{});
        EXPECT_NE(outcome.state.mode(), FlightMode::Disarmed) << "from " << to_string(mode);
    }
}

TEST(ManualRuleTest, AutomaticPhasesRejectStickInput) {
    const FlightConfig config;
    for (const auto mode : {FlightMode::Takeoff, FlightMode::Land, FlightMode::Disarmed}) {
        const FlightState before = state_in(mode);
        const auto outcome = fsm::apply(before, config, Event::ManualActive,
                                        EventPayload{Vec3{1, 0, 0}, Instant{}});
        EXPECT_EQ(outcome.result, CommandResult::Rejected) << "in " << to_string(mode);
        EXPECT_EQ(outcome.state.mode(), before.mode());
    }
}

TEST(GotoRuleTest, TargetOutsideTheFenceIsClampedNotRejected) {
    const FlightConfig config;
    const auto outcome = fsm::apply(state_in(FlightMode::Hold), config, Event::Goto,
                                    EventPayload{Vec3{9999.0, -9999.0, 9999.0}, Instant{}});
    ASSERT_EQ(outcome.result, CommandResult::Accepted);
    ASSERT_TRUE(outcome.state.target.has_value());
    EXPECT_TRUE(config.fence.contains(*outcome.state.target));
}

// ---------------------------------------------------------------------------
// Purity of the dynamics layer.
// ---------------------------------------------------------------------------

TEST(DynamicsTest, IntegrateIsReferentiallyTransparent) {
    const FlightConfig config;
    const FlightState state = state_in(FlightMode::Goto);
    const auto first = dynamics::integrate(state, config, 0.01, Instant{});
    const auto second = dynamics::integrate(state, config, 0.01, Instant{});
    EXPECT_EQ(first.state.telemetry, second.state.telemetry);
    EXPECT_EQ(first.fence_engaged, second.fence_engaged);
}

TEST(DynamicsTest, IntegrateDoesNotMutateItsArgument) {
    const FlightConfig config;
    FlightState state = state_in(FlightMode::Goto);
    const Telemetry before = state.telemetry;
    (void)dynamics::integrate(state, config, 0.5, Instant{});
    EXPECT_EQ(state.telemetry, before) << "the pure core must take its state by value";
}

TEST(DynamicsTest, SpeedNeverExceedsTheEnvelopeInAnyMode) {
    const FlightConfig config;
    const double envelope =
        std::max(config.horizontal_speed_mps, config.climb_speed_mps) * std::sqrt(3.0) + 1e-6;
    for (const auto mode : kAllModes) {
        FlightState state = state_in(mode);
        state.manual_axes = Vec3{1.0, 1.0, 1.0};  // worst case, already unit-clamped upstream
        const Vec3 velocity = dynamics::desired_velocity(state, config, Instant{});
        EXPECT_LE(velocity.norm(), envelope) << "in " << to_string(mode);
    }
}

TEST(DynamicsTest, FenceEngagedIsReportedWhenTheClampFires) {
    FlightConfig config;
    FlightState state = state_in(FlightMode::Manual);
    state.telemetry.position = Vec3{99.9, 0.0, 20.0};
    state.manual_axes = Vec3{1.0, 0.0, 0.0};
    const auto result = dynamics::integrate(state, config, 1.0, Instant{});
    EXPECT_TRUE(result.fence_engaged);
    EXPECT_NEAR(result.state.telemetry.position.x, 100.0, 1e-9);
    EXPECT_NEAR(result.state.telemetry.velocity.x, 0.1, 1e-6)
        << "reported velocity must be the one actually achieved";
}

}  // namespace
}  // namespace kratt::domain
