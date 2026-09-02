#include <gtest/gtest.h>

#include "integration/world.hpp"

namespace kratt::testing {
namespace {

using namespace std::chrono_literals;

/// Telemetry stream and rate tests. The specification demands:
///   "Telemetry stream: sends all current drone status at 10Hz using appropriate
///    MAVLink messages: HEARTBEAT, LOCAL_POSITION_NED [X,Y,Alt]."
class TelemetryRateIT : public ::testing::Test {
protected:
    World world{};

    void SetUp() override { ASSERT_TRUE(world.establish_link()); }
};

TEST_F(TelemetryRateIT, Telemetry_publishes_at_exactly_10Hz) {
    world.gcs().service().arm();
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Hold; }, 10s));

    const auto before = world.drone().service().publications();
    for (int i = 0; i < 1000; ++i) {
        world.advance(10ms);  // 10 seconds simulated
    }
    const auto published = world.drone().service().publications() - before;
    // 10 Hz for 10 s = 100 frames, with ±10% tolerance
    EXPECT_GE(published, 99U);
    EXPECT_LE(published, 101U) << "telemetry rate drift detected";
}

TEST_F(TelemetryRateIT, Gcs_receives_position_x_y_altitude_in_telemetry) {
    world.gcs().service().arm();
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Hold; }, 10s));

    world.gcs().service().send_goto({30.0, -40.0, 20.0});
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Goto; }, 1s));
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Hold; }, 60s));
    world.advance(200ms);

    // Round-trip through NED wire format: x, y, z sign-flipped
    EXPECT_NEAR(world.gcs_view().telemetry.position.x, 30.0, 0.1);
    EXPECT_NEAR(world.gcs_view().telemetry.position.y, -40.0, 0.1);
    EXPECT_NEAR(world.gcs_view().telemetry.position.z, 20.0, 0.1);
}

TEST_F(TelemetryRateIT, Heartbeat_base_mode_reflects_armed_state) {
    // Disarmed
    EXPECT_FALSE(world.gcs_view().telemetry.armed);

    world.gcs().service().arm();
    ASSERT_TRUE(world.advance_until([&] { return world.gcs_view().telemetry.armed; }, 2s));
    EXPECT_TRUE(world.gcs_view().telemetry.armed);

    world.gcs().service().disarm();
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().position.z < 0.5; }, 60s));
    ASSERT_TRUE(world.advance_until([&] { return !world.gcs_view().telemetry.armed; }, 1s));
    EXPECT_FALSE(world.gcs_view().telemetry.armed);
}

TEST_F(TelemetryRateIT, Heartbeat_custom_mode_field_carries_internal_flight_mode) {
    world.gcs().service().arm();
    ASSERT_TRUE(world.advance_until(
        [&] { return world.gcs_view().telemetry.mode == domain::FlightMode::Hold; }, 10s));

    world.gcs().service().send_goto({50.0, 0.0, 20.0});
    ASSERT_TRUE(world.advance_until(
        [&] { return world.gcs_view().telemetry.mode == domain::FlightMode::Goto; }, 2s));

    world.gcs().service().send_manual({1.0, 0.0, 0.0}, world.now());
    ASSERT_TRUE(world.advance_until(
        [&] { return world.gcs_view().telemetry.mode == domain::FlightMode::Manual; }, 2s));
}

TEST_F(TelemetryRateIT, Velocity_is_zero_when_holding_position) {
    world.gcs().service().arm();
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Hold; }, 10s));

    world.advance(500ms);
    EXPECT_NEAR(world.gcs_view().telemetry.velocity.x, 0.0, 1e-6);
    EXPECT_NEAR(world.gcs_view().telemetry.velocity.y, 0.0, 1e-6);
    EXPECT_NEAR(world.gcs_view().telemetry.velocity.z, 0.0, 1e-6);
}

TEST_F(TelemetryRateIT, Telemetry_is_unaffected_by_command_flood) {
    world.gcs().service().arm();
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Hold; }, 10s));

    const auto before = world.drone().service().publications();
    // Flood with 1000 MANUAL_CONTROL messages while counting publications
    for (int i = 0; i < 100; ++i) {
        for (int j = 0; j < 10; ++j) {
            world.gcs().service().send_manual({0.5, 0.0, 0.0}, world.now());
        }
        world.advance(10ms);
    }
    const auto published = world.drone().service().publications() - before;
    // 1 second at 10 Hz = 10 frames, ±20% tolerance for the load
    EXPECT_GE(published, 8U) << "telemetry rate collapsed under command load";
    EXPECT_LE(published, 12U);
}

}  // namespace
}  // namespace kratt::testing
