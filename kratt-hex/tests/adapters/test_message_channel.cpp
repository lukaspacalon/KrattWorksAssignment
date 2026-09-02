#include <gtest/gtest.h>

#include "adapters/mavlink/message_channel.hpp"
#include "adapters/transport/loopback_transport.hpp"

namespace kratt::adapters::mavlink {
namespace {

/// Message channel tests: synchronous channel vs threaded channel.
class SynchronousChannelTest : public ::testing::Test {
protected:
    LoopbackNetwork network{};

    mavlink_message_t make_heartbeat(std::uint8_t sysid) {
        mavlink_message_t msg{};
        mavlink_msg_heartbeat_pack(sysid, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR,
                                   MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0,
                                   MAV_STATE_ACTIVE);
        return msg;
    }
};

TEST_F(SynchronousChannelTest, Send_encodes_message_and_transmits) {
    auto tx_transport = network.create_endpoint(1001);
    auto rx_transport = network.create_endpoint(1002);

    SynchronousChannel::Config cfg;
    cfg.peer = Endpoint{"loopback", 1002};
    cfg.learn_peer_from_traffic = false;
    SynchronousChannel channel{std::move(tx_transport), cfg};

    const auto msg = make_heartbeat(1);
    channel.send(msg);

    auto received = rx_transport->receive(std::chrono::milliseconds{100});
    ASSERT_TRUE(received.has_value());
    EXPECT_GT(received->payload.size(), 0U);
}

TEST_F(SynchronousChannelTest, Poll_returns_empty_when_no_data) {
    auto transport = network.create_endpoint(2001);
    SynchronousChannel::Config cfg;
    SynchronousChannel channel{std::move(transport), cfg};

    auto messages = channel.poll();
    EXPECT_EQ(messages.size(), 0U);
}

TEST_F(SynchronousChannelTest, Poll_decodes_received_messages) {
    auto sender = network.create_endpoint(3001);
    auto receiver_transport = network.create_endpoint(3002);

    SynchronousChannel::Config cfg;
    cfg.learn_peer_from_traffic = false;
    cfg.peer = Endpoint{"loopback", 3002};
    SynchronousChannel receiver{std::move(receiver_transport), cfg};

    // Send a heartbeat via the network
    const auto msg = make_heartbeat(42);
    SynchronousChannel tx{std::move(sender), cfg};
    tx.send(msg);

    // Receive and decode
    auto received = receiver.poll();
    ASSERT_EQ(received.size(), 1U);
    EXPECT_EQ(received[0].message.msgid, MAVLINK_MSG_ID_HEARTBEAT);
    EXPECT_EQ(received[0].message.sysid, 42);
}

TEST_F(SynchronousChannelTest, Learn_peer_from_first_datagram) {
    auto sender = network.create_endpoint(4001);
    auto receiver_transport = network.create_endpoint(4002);

    SynchronousChannel::Config cfg;
    cfg.learn_peer_from_traffic = true;  // starts without a peer
    cfg.peer.reset();
    SynchronousChannel receiver{std::move(receiver_transport), cfg};

    EXPECT_FALSE(receiver.peer().has_value());

    const auto msg = make_heartbeat(1);
    SynchronousChannel tx{std::move(sender), SynchronousChannel::Config{
                                                  .peer = Endpoint{"loopback", 4002},
                                                  .learn_peer_from_traffic = false}};
    tx.send(msg);

    receiver.poll();
    EXPECT_TRUE(receiver.peer().has_value());
    EXPECT_EQ(receiver.peer()->port, 4001);
}

TEST_F(SynchronousChannelTest, Send_fails_gracefully_with_no_peer) {
    auto transport = network.create_endpoint(5001);
    SynchronousChannel::Config cfg;
    cfg.peer.reset();
    SynchronousChannel channel{std::move(transport), cfg};

    const auto msg = make_heartbeat(1);
    channel.send(msg);  // should not crash

    EXPECT_GT(channel.dropped_no_peer(), 0U);
}

TEST_F(SynchronousChannelTest, Decoder_stats_are_accessible) {
    auto sender = network.create_endpoint(6001);
    auto receiver_transport = network.create_endpoint(6002);

    SynchronousChannel::Config cfg;
    cfg.peer = Endpoint{"loopback", 6002};
    cfg.learn_peer_from_traffic = false;
    SynchronousChannel receiver{std::move(receiver_transport), cfg};

    SynchronousChannel tx{std::move(sender), cfg};
    const auto msg = make_heartbeat(1);
    tx.send(msg);

    const auto before = receiver.decoder_stats().messages;
    receiver.poll();
    const auto after = receiver.decoder_stats().messages;

    EXPECT_GT(after, before);
}

}  // namespace
}  // namespace kratt::adapters::mavlink
