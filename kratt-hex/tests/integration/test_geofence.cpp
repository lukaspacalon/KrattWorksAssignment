#include <gtest/gtest.h>

#include "integration/world.hpp"

namespace kratt::testing {
namespace {

using namespace std::chrono_literals;

/// The geofence must hold under real commands arriving over the wire, not only
/// as a unit-tested clamp.
class GeofenceIT : public ::testing::Test {
protected:
    World world{};

    void SetUp() override { ASSERT_TRUE(world.establish_link()); }

    void arm_and_hover() {
        world.gcs().service().arm();
        ASSERT_TRUE(world.advance_until(
            [&] { return world.drone_state().mode == domain::FlightMode::Hold; }, 10s));
    }
};

TEST_F(GeofenceIT, Manual_input_cannot_push_the_drone_past_the_north_boundary) {
    arm_and_hover();
    // Full north stick for far longer than it takes to cross 100 m at 10 m/s.
    for (int i = 0; i < 400; ++i) {
        world.gcs().service().send_manual({1.0, 0.0, 0.0}, world.now());
        world.advance(50ms);
    }
    EXPECT_LE(world.drone_state().position.x, 100.0 + 1e-9);
    EXPECT_NEAR(world.drone_state().position.x, 100.0, 0.5);
}

TEST_F(GeofenceIT, Position_stays_inside_the_fence_under_sustained_corner_pressure) {
    arm_and_hover();
    const auto& fence = world.drone().service().controller().config().fence;
    for (int i = 0; i < 400; ++i) {
        world.gcs().service().send_manual({1.0, 1.0, 1.0}, world.now());
        world.advance(50ms);
        ASSERT_TRUE(fence.contains(world.drone_state().position))
            << "escaped at (" << world.drone_state().position.x << ", "
            << world.drone_state().position.y << ", " << world.drone_state().position.z << ")";
    }
}

TEST_F(GeofenceIT, Reported_velocity_is_zero_on_the_clamped_axis) {
    arm_and_hover();
    for (int i = 0; i < 400; ++i) {
        world.gcs().service().send_manual({1.0, 0.0, 0.0}, world.now());
        world.advance(50ms);
    }
    // Pressed against the fence: the drone must report what it actually does,
    // not the 10 m/s it was commanded.
    EXPECT_NEAR(world.drone_state().velocity.x, 0.0, 1e-6);
}

TEST_F(GeofenceIT, Drone_still_slides_along_the_boundary_on_the_free_axis) {
    arm_and_hover();
    for (int i = 0; i < 400; ++i) {
        world.gcs().service().send_manual({1.0, 0.0, 0.0}, world.now());
        world.advance(50ms);
    }
    ASSERT_NEAR(world.drone_state().position.x, 100.0, 0.5);

    const double y_before = world.drone_state().position.y;
    for (int i = 0; i < 20; ++i) {
        world.gcs().service().send_manual({1.0, 1.0, 0.0}, world.now());
        world.advance(50ms);
    }
    EXPECT_GT(world.drone_state().position.y, y_before + 1.0)
        << "clamping one axis must not freeze the others";
}

TEST_F(GeofenceIT, Goto_outside_the_fence_is_clamped_not_ignored) {
    arm_and_hover();
    world.gcs().service().send_goto({5000.0, 0.0, 20.0});
    // Wait for the command to take effect first: the drone is already in Hold,
    // so polling for Hold would succeed before the GOTO ever arrived.
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Goto; }, 1s));
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Hold; }, 60s));
    EXPECT_NEAR(world.drone_state().position.x, 100.0, 0.5)
        << "the drone should fly as close as the fence allows";
}

TEST_F(GeofenceIT, Takeoff_stops_at_the_ceiling_when_it_is_below_20m) {
    // Edge case: a fence lower than the specified take-off altitude.
    domain::FlightConfig config;
    config.fence = domain::Geofence{100.0, 15.0};
    World low_ceiling{config};
    ASSERT_TRUE(low_ceiling.establish_link());

    low_ceiling.gcs().service().arm();
    ASSERT_TRUE(low_ceiling.advance_until(
        [&] { return low_ceiling.drone_state().mode == domain::FlightMode::Hold; }, 10s));
    EXPECT_NEAR(low_ceiling.drone_state().position.z, 15.0, 0.01);
}

}  // namespace
}  // namespace kratt::testing
