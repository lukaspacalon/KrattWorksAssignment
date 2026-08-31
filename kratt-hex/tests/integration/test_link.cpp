#include <gtest/gtest.h>

#include "integration/world.hpp"

namespace kratt::testing {
namespace {

using namespace std::chrono_literals;

TEST(ConnectionIT, Gcs_is_disconnected_before_any_heartbeat) {
    World world{};
    EXPECT_FALSE(world.gcs_view().connected);
}

TEST(ConnectionIT, Gcs_becomes_connected_after_the_first_heartbeat) {
    World world{};
    ASSERT_TRUE(world.advance_until([&] { return world.gcs_view().connected; }, 1s));
    EXPECT_TRUE(world.gcs_events().contains("drone connected"));
}

TEST(ConnectionIT, Gcs_goes_disconnected_after_a_link_partition) {
    World world{};
    ASSERT_TRUE(world.advance_until([&] { return world.gcs_view().connected; }, 1s));

    world.network().set_partitioned(true);
    ASSERT_TRUE(world.advance_until([&] { return !world.gcs_view().connected; }, 3s));
    EXPECT_TRUE(world.gcs_events().contains("drone disconnected"));
}

TEST(ConnectionIT, Gcs_reconnects_when_the_partition_is_healed) {
    World world{};
    ASSERT_TRUE(world.advance_until([&] { return world.gcs_view().connected; }, 1s));
    world.network().set_partitioned(true);
    ASSERT_TRUE(world.advance_until([&] { return !world.gcs_view().connected; }, 3s));

    world.network().set_partitioned(false);
    EXPECT_TRUE(world.advance_until([&] { return world.gcs_view().connected; }, 2s));
}

TEST(ConnectionIT, Connection_survives_twenty_percent_packet_loss) {
    World world{};
    world.network().set_packet_loss(0.2);
    ASSERT_TRUE(world.advance_until([&] { return world.gcs_view().connected; }, 2s));

    // Stay up continuously for five seconds despite one datagram in five going
    // missing: the 1 s heartbeat timeout must tolerate that.
    for (int i = 0; i < 500; ++i) {
        world.advance(10ms);
        ASSERT_TRUE(world.gcs_view().connected) << "dropped out at iteration " << i;
    }
    EXPECT_GT(world.network().dropped(), 0U) << "the loss injection did not actually fire";
}

TEST(TelemetryIT, Gcs_sees_the_position_the_drone_actually_holds) {
    World world{};
    ASSERT_TRUE(world.establish_link());
    world.gcs().service().arm();
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Hold; }, 10s));

    world.gcs().service().send_goto({30.0, -40.0, 20.0});
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Goto; }, 1s));
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Hold; }, 60s));
    world.advance(200ms);  // let one telemetry period elapse

    // Round-trip through LOCAL_POSITION_NED, including the up/down sign flip.
    EXPECT_NEAR(world.gcs_view().telemetry.position.x, 30.0, 0.1);
    EXPECT_NEAR(world.gcs_view().telemetry.position.y, -40.0, 0.1);
    EXPECT_NEAR(world.gcs_view().telemetry.position.z, 20.0, 0.1);
}

TEST(TelemetryIT, Publication_rate_is_10Hz) {
    World world{};
    const auto before = world.drone().service().publications();
    for (int i = 0; i < 1000; ++i) {
        world.advance(10ms);  // 10 simulated seconds
    }
    const auto published = world.drone().service().publications() - before;
    EXPECT_GE(published, 99U);
    EXPECT_LE(published, 101U) << "telemetry must stream at 10 Hz, not at the simulation rate";
}

}  // namespace
}  // namespace kratt::testing
