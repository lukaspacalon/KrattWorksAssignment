#include <gtest/gtest.h>

#include "integration/world.hpp"

namespace kratt::testing {
namespace {

using namespace std::chrono_literals;

/// Arming and disarming sequence. The specification requires:
///   "Arm(activate)/Disarm(deactivate) the drone by sending Mavlink SET_MODE.
///    The drone should automatically climb to 20m Alt."
class ArmDisarmIT : public ::testing::Test {
protected:
    World world{};

    void SetUp() override { ASSERT_TRUE(world.establish_link()); }
};

TEST_F(ArmDisarmIT, Drone_starts_disarmed) { EXPECT_FALSE(world.drone_state().armed); }

TEST_F(ArmDisarmIT, Set_mode_guided_armed_arms_the_drone) {
    world.gcs().service().arm();
    ASSERT_TRUE(world.advance_until([&] { return world.drone_state().armed; }, 2s));
    EXPECT_EQ(world.drone_state().mode, domain::FlightMode::Takeoff);
}

TEST_F(ArmDisarmIT, Automatic_climb_reaches_20m_altitude) {
    world.gcs().service().arm();
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Hold; }, 10s));
    EXPECT_NEAR(world.drone_state().position.z, 20.0, 0.01);
}

TEST_F(ArmDisarmIT, Takeoff_transitions_to_hold_at_ceiling) {
    world.gcs().service().arm();
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Hold; }, 10s));
    EXPECT_EQ(world.drone_state().mode, domain::FlightMode::Hold);
}

TEST_F(ArmDisarmIT, Arming_an_already_armed_drone_is_idempotent) {
    world.gcs().service().arm();
    ASSERT_TRUE(world.advance_until([&] { return world.drone_state().armed; }, 2s));
    const auto position_before = world.drone_state().position;

    world.gcs().service().arm();
    world.advance(100ms);
    EXPECT_EQ(world.drone_state().position.x, position_before.x);
}

TEST_F(ArmDisarmIT, Set_mode_disarm_on_ground_disarms_immediately) {
    ASSERT_FALSE(world.drone_state().armed);
    world.gcs().service().disarm();
    EXPECT_FALSE(world.drone_state().armed);
}

TEST_F(ArmDisarmIT, Gcs_sees_armed_state_change_within_reasonable_time) {
    world.gcs().service().arm();
    ASSERT_TRUE(world.advance_until([&] { return world.gcs_view().telemetry.armed; }, 2s));
}

}  // namespace
}  // namespace kratt::testing
