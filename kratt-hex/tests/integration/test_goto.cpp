#include <gtest/gtest.h>

#include "integration/world.hpp"

namespace kratt::testing {
namespace {

using namespace std::chrono_literals;

/// Goto navigation. The specification requires:
///   "Implement MAV_CMD_OVERRIDE_GOTO for setting current GoTo target."
class GotoIT : public ::testing::Test {
protected:
    World world{};

    void SetUp() override { ASSERT_TRUE(world.establish_link()); }

    void arm_and_hover() {
        world.gcs().service().arm();
        ASSERT_TRUE(world.advance_until(
            [&] { return world.drone_state().mode == domain::FlightMode::Hold; }, 10s));
    }
};

TEST_F(GotoIT, Override_goto_moves_drone_towards_target) {
    arm_and_hover();
    world.gcs().service().send_goto({50.0, 0.0, 20.0});
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Goto; }, 1s));

    for (int i = 0; i < 50; ++i) {
        world.advance(100ms);
    }
    EXPECT_GT(world.drone_state().position.x, 10.0) << "drone should have moved north";
}

TEST_F(GotoIT, Horizontal_speed_never_exceeds_10mps_during_transit) {
    arm_and_hover();
    world.gcs().service().send_goto({50.0, 0.0, 20.0});
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Goto; }, 1s));

    for (int i = 0; i < 300; ++i) {
        world.advance(10ms);
        const double horizontal_speed =
            std::sqrt(world.drone_state().velocity.x * world.drone_state().velocity.x +
                      world.drone_state().velocity.y * world.drone_state().velocity.y);
        EXPECT_LE(horizontal_speed, 10.01) << "at step " << i;
    }
}

TEST_F(GotoIT, Goto_clamped_to_fence_when_outside) {
    arm_and_hover();
    world.gcs().service().send_goto({9999.0, -9999.0, 9999.0});
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Goto; }, 1s));
    EXPECT_TRUE(world.drone().service().controller().config().fence.contains(
        *world.drone().service().controller().target()));
}

TEST_F(GotoIT, Goto_while_disarmed_is_rejected) {
    EXPECT_EQ(world.gcs().service().view().telemetry.mode, domain::FlightMode::Disarmed);
    world.gcs().service().send_goto({50.0, 0.0, 20.0});
    EXPECT_EQ(world.drone_state().mode, domain::FlightMode::Disarmed);
}

}  // namespace
}  // namespace kratt::testing
