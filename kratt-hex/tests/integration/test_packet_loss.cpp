#include <gtest/gtest.h>

#include "integration/world.hpp"

namespace kratt::testing {
namespace {

using namespace std::chrono_literals;

/// Network resilience under packet loss. UDP is lossy; the protocol must survive.
class PacketLossIT : public ::testing::Test {
protected:
    World world{};

    void SetUp() override { ASSERT_TRUE(world.establish_link()); }
};

TEST_F(PacketLossIT, Connection_survives_5_percent_packet_loss) {
    world.network().set_packet_loss(0.05);
    world.gcs().service().arm();
    ASSERT_TRUE(world.advance_until(
        [&] { return world.gcs_view().connected; }, 2s));

    for (int i = 0; i < 500; ++i) {
        world.advance(10ms);
        EXPECT_TRUE(world.gcs_view().connected);
    }
}

TEST_F(PacketLossIT, Connection_survives_10_percent_packet_loss) {
    world.network().set_packet_loss(0.10);
    world.gcs().service().arm();
    ASSERT_TRUE(world.advance_until(
        [&] { return world.gcs_view().connected; }, 2s));

    for (int i = 0; i < 500; ++i) {
        world.advance(10ms);
        EXPECT_TRUE(world.gcs_view().connected);
    }
}

TEST_F(PacketLossIT, Connection_survives_20_percent_packet_loss) {
    world.network().set_packet_loss(0.20);
    world.gcs().service().arm();
    ASSERT_TRUE(world.advance_until(
        [&] { return world.gcs_view().connected; }, 2s));

    for (int i = 0; i < 500; ++i) {
        world.advance(10ms);
        EXPECT_TRUE(world.gcs_view().connected);
    }
}

TEST_F(PacketLossIT, Goto_command_survives_packet_loss) {
    world.network().set_packet_loss(0.15);
    world.gcs().service().arm();
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Hold; }, 10s));

    world.gcs().service().send_goto({30.0, 0.0, 20.0});
    ASSERT_TRUE(world.advance_until(
        [&] { return world.drone_state().mode == domain::FlightMode::Goto; }, 5s))
        << "GOTO never took effect despite retries";
}

TEST_F(PacketLossIT, Packet_loss_is_measured_and_reported) {
    world.network().set_packet_loss(0.50);
    for (int i = 0; i < 100; ++i) {
        world.advance(10ms);
    }
    EXPECT_GT(world.network().dropped(), 0U) << "loss injection should have fired";
}

}  // namespace
}  // namespace kratt::testing
