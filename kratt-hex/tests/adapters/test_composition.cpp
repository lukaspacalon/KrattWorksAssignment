#include <gtest/gtest.h>

#include "adapters/logging/console_event_log.hpp"
#include "adapters/mavlink/message_channel.hpp"
#include "adapters/transport/loopback_transport.hpp"
#include "app/drone_composition.hpp"
#include "app/gcs_composition.hpp"

namespace kratt::app {
namespace {

using kratt::adapters::LoopbackNetwork;
using kratt::adapters::mavlink::SynchronousChannel;
using kratt::adapters::Endpoint;
using kratt::adapters::RecordingEventLog;
using kratt::domain::FlightMode;
using kratt::domain::Instant;

/// Composition tests: the wiring of domain + adapters + ports.
TEST(CompositionTest, DroneComposition_can_be_instantiated) {
    LoopbackNetwork network;
    auto transport = network.create_endpoint(10001);

    SynchronousChannel::Config cfg;
    cfg.peer = Endpoint{LoopbackNetwork::kAddress, 11001};
    cfg.learn_peer_from_traffic = false;
    SynchronousChannel channel{std::move(transport), cfg};

    RecordingEventLog event_log;
    DroneComposition::Config config;
    DroneComposition drone{channel, config, event_log};

    EXPECT_FALSE(drone.telemetry().armed);
    EXPECT_EQ(drone.telemetry().mode, FlightMode::Disarmed);
}

TEST(CompositionTest, DroneComposition_tick_processes_messages) {
    LoopbackNetwork network;
    auto transport = network.create_endpoint(10002);

    SynchronousChannel::Config cfg;
    cfg.peer = Endpoint{LoopbackNetwork::kAddress, 11002};
    cfg.learn_peer_from_traffic = false;
    SynchronousChannel channel{std::move(transport), cfg};

    RecordingEventLog event_log;
    DroneComposition::Config config;
    DroneComposition drone{channel, config, event_log};

    // A tick with zero dt should be a no-op
    const auto pos_before = drone.telemetry().position;
    drone.tick(0.0, Instant{});
    EXPECT_EQ(drone.telemetry().position, pos_before);
}

TEST(CompositionTest, DroneComposition_exposes_service) {
    LoopbackNetwork network;
    auto transport = network.create_endpoint(10003);

    SynchronousChannel::Config cfg;
    cfg.peer = Endpoint{LoopbackNetwork::kAddress, 11003};
    cfg.learn_peer_from_traffic = false;
    SynchronousChannel channel{std::move(transport), cfg};

    RecordingEventLog event_log;
    DroneComposition::Config config;
    DroneComposition drone{channel, config, event_log};

    auto& service = drone.service();
    EXPECT_FALSE(service.telemetry().armed);
    EXPECT_EQ(service.telemetry().mode, FlightMode::Disarmed);
}

TEST(CompositionTest, GcsComposition_can_be_instantiated) {
    LoopbackNetwork network;
    auto transport = network.create_endpoint(20001);

    SynchronousChannel::Config cfg;
    cfg.learn_peer_from_traffic = true;
    SynchronousChannel channel{std::move(transport), cfg};

    RecordingEventLog event_log;
    GcsComposition::Config config;
    GcsComposition gcs{channel, config, event_log};

    EXPECT_FALSE(gcs.view().connected);
}

TEST(CompositionTest, GcsComposition_tick_updates_link_status) {
    LoopbackNetwork network;
    auto transport = network.create_endpoint(20002);

    SynchronousChannel::Config cfg;
    cfg.learn_peer_from_traffic = true;
    SynchronousChannel channel{std::move(transport), cfg};

    RecordingEventLog event_log;
    GcsComposition::Config config;
    GcsComposition gcs{channel, config, event_log};

    // A tick with no messages should keep it disconnected
    gcs.tick(Instant{});
    EXPECT_FALSE(gcs.view().connected);
}

TEST(CompositionTest, GcsComposition_exposes_view) {
    LoopbackNetwork network;
    auto transport = network.create_endpoint(20003);

    SynchronousChannel::Config cfg;
    cfg.learn_peer_from_traffic = true;
    SynchronousChannel channel{std::move(transport), cfg};

    RecordingEventLog event_log;
    GcsComposition::Config config;
    GcsComposition gcs{channel, config, event_log};

    const auto& view = gcs.view();
    EXPECT_FALSE(view.connected);
    EXPECT_FALSE(view.telemetry.armed);
}

}  // namespace
}  // namespace kratt::app
