#include <gtest/gtest.h>

#include "integration/world.hpp"

namespace kratt::testing {
namespace {

using namespace std::chrono_literals;

/// Manual stick control. The specification requires:
///   "Add MANUAL_CONTROL message for moving the drone. When user input stops,
///    the drone should hold position."
class ManualControlIT : public ::testing::Test {
protected:
    World world{};

    void SetUp() override { ASSERT_TRUE(world.establish_link()); }

    void arm_and_hover() {
        world.gcs().service().arm();
        ASSERT_TRUE(world.advance_until(
            [&] { return world.drone_state().mode == domain::FlightMode::Hold; }, 10s));
    }
};

TEST_F(ManualControlIT, Manual_control_moves_drone_north) {
    arm_and_hover();
    world.gcs().service().send_manual({1.0, 0.0, 0.0}, world.now());
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Manual; }, 1s));

    for (int i = 0; i < 50; ++i) {
        world.gcs().service().send_manual({1.0, 0.0, 0.0}, world.now());
        world.advance(50ms);
    }
    EXPECT_GT(world.drone_state().position.x, 5.0);
}

TEST_F(ManualControlIT, Manual_control_moves_drone_east) {
    arm_and_hover();
    for (int i = 0; i < 50; ++i) {
        world.gcs().service().send_manual({0.0, 1.0, 0.0}, world.now());
        world.advance(50ms);
    }
    EXPECT_GT(world.drone_state().position.y, 5.0);
}

TEST_F(ManualControlIT, Manual_control_changes_altitude) {
    arm_and_hover();
    for (int i = 0; i < 50; ++i) {
        world.gcs().service().send_manual({0.0, 0.0, -1.0}, world.now());
        world.advance(50ms);
    }
    EXPECT_LT(world.drone_state().position.z, 15.0);
}

TEST_F(ManualControlIT, Drone_holds_position_when_input_stops) {
    arm_and_hover();
    for (int i = 0; i < 30; ++i) {
        world.gcs().service().send_manual({1.0, 0.0, 0.0}, world.now());
        world.advance(50ms);
    }
    const auto pos_before = world.drone_state().position;

    // Release the sticks
    world.gcs().service().send_manual({0.0, 0.0, 0.0}, world.now());
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Hold; }, 2s));

    world.advance(500ms);
    EXPECT_NEAR(world.drone_state().position.x, pos_before.x, 1e-5);
}

TEST_F(ManualControlIT, Held_position_does_not_drift_over_time) {
    arm_and_hover();
    const auto pos_start = world.drone_state().position;
    for (int i = 0; i < 500; ++i) {
        world.advance(10ms);
    }
    EXPECT_NEAR(world.drone_state().position.x, pos_start.x, 1e-6);
    EXPECT_NEAR(world.drone_state().position.y, pos_start.y, 1e-6);
    EXPECT_NEAR(world.drone_state().position.z, pos_start.z, 1e-6);
}

TEST_F(ManualControlIT, Manual_input_cancels_active_goto_target) {
    arm_and_hover();
    world.gcs().service().send_goto({100.0, 0.0, 20.0});
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Goto; }, 1s));

    world.gcs().service().send_manual({1.0, 0.0, 0.0}, world.now());
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Manual; }, 1s));
    EXPECT_FALSE(world.drone().service().controller().target().has_value());
}

TEST_F(ManualControlIT, Manual_control_is_clamped_to_10mps_envelope) {
    arm_and_hover();
    for (int i = 0; i < 20; ++i) {
        world.gcs().service().send_manual({1.0, 1.0, 1.0}, world.now());
        world.advance(50ms);
        const double speed = world.drone_state().velocity.norm();
        EXPECT_LE(speed, 10.01) << "at step " << i;
    }
}



}  // namespace
}  // namespace kratt::testing
