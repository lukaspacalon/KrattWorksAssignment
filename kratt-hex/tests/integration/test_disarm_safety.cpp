#include <gtest/gtest.h>

#include "integration/world.hpp"

namespace kratt::testing {
namespace {

using namespace std::chrono_literals;

/// Full-stack tests of the specification's safety rule:
///   "Disarming should only be allowed if altitude is < 0.5m. If trying to
///    disarm while flying, automatically enter Land mode."
///
/// These go through the real MAVLink encoding and the real UDP-shaped transport
/// (loopback), so they cover the GCS, the wire format and the drone at once —
/// while still running with no thread and no wall-clock wait.
class DisarmSafetyIT : public ::testing::Test {
protected:
    World world{};

    void SetUp() override {
        ASSERT_TRUE(world.establish_link()) << "the GCS never heard the drone";
    }

    /// Brings the drone to a stable hover at the take-off altitude.
    void arm_and_reach_hover() {
        world.gcs().service().arm();
        ASSERT_TRUE(world.advance_until(
            [&] { return world.drone_state().mode == domain::FlightMode::Hold; }, 10s))
            << "the drone never reached hover";
        ASSERT_NEAR(world.drone_state().position.z, 20.0, 0.01);
    }
};

TEST_F(DisarmSafetyIT, Disarming_allowed_if_altitude_is_below_0_5m) {
    world.gcs().service().arm();
    world.advance(10ms);  // armed, but still essentially on the ground
    ASSERT_TRUE(world.drone_state().armed);
    ASSERT_LT(world.drone_state().position.z, 0.5);

    world.gcs().service().disarm();
    ASSERT_TRUE(world.advance_until([&] { return !world.drone_state().armed; }, 1s));
    EXPECT_EQ(world.drone_state().mode, domain::FlightMode::Disarmed);
}

TEST_F(DisarmSafetyIT, Disarming_while_flying_enters_land_mode_instead) {
    arm_and_reach_hover();

    world.gcs().service().disarm();
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Land; }, 1s))
        << "disarm above the safety altitude must convert into a landing";

    EXPECT_TRUE(world.drone_state().armed) << "the drone must not cut its motors in flight";
    EXPECT_LT(world.drone_state().velocity.z, 0.0) << "the descent must have started";
    EXPECT_TRUE(world.drone_events().contains("LAND"));
}

TEST_F(DisarmSafetyIT, Disarm_while_flying_eventually_disarms_after_landing) {
    arm_and_reach_hover();
    world.gcs().service().disarm();

    // 20 m at 1 m/s, plus margin.
    ASSERT_TRUE(world.advance_until([&] { return !world.drone_state().armed; }, 60s));
    EXPECT_EQ(world.drone_state().mode, domain::FlightMode::Disarmed);
    EXPECT_NEAR(world.drone_state().position.z, 0.0, 1e-9);
}

TEST_F(DisarmSafetyIT, Descent_is_gentle_compared_to_the_climb_rate) {
    arm_and_reach_hover();
    world.gcs().service().disarm();
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Land; }, 1s));
    world.advance(100ms);

    const double descent = -world.drone_state().velocity.z;
    EXPECT_GT(descent, 0.0);
    EXPECT_LT(descent, world.drone().service().controller().config().climb_speed_mps)
        << "landing must be gentler than the automatic climb";
}

TEST_F(DisarmSafetyIT, Repeated_disarm_requests_during_landing_do_not_abort_it) {
    arm_and_reach_hover();
    world.gcs().service().disarm();
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Land; }, 1s));

    // An impatient operator hammering the button must not make things worse.
    for (int i = 0; i < 10; ++i) {
        world.gcs().service().disarm();
        world.advance(50ms);
        EXPECT_EQ(world.drone_state().mode, domain::FlightMode::Land);
    }
    ASSERT_TRUE(world.advance_until([&] { return !world.drone_state().armed; }, 60s));
}

TEST_F(DisarmSafetyIT, Gcs_observes_the_disarm_through_heartbeat_telemetry) {
    arm_and_reach_hover();
    ASSERT_TRUE(world.advance_until([&] { return world.gcs_view().telemetry.armed; }, 1s));

    world.gcs().service().disarm();
    ASSERT_TRUE(world.advance_until([&] { return !world.drone_state().armed; }, 60s));
    // The ground station must converge on the same view within a few frames.
    ASSERT_TRUE(world.advance_until([&] { return !world.gcs_view().telemetry.armed; }, 1s));
    EXPECT_EQ(world.gcs_view().telemetry.mode, domain::FlightMode::Disarmed);
}

}  // namespace
}  // namespace kratt::testing
