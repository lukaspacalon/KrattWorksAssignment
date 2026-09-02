#include <gtest/gtest.h>

#include <array>

#include "adapters/transport/loopback_transport.hpp"
#include "adapters/transport/udp_socket.hpp"

namespace kratt::adapters {
namespace {

/// Transport layer tests. Both UdpSocket and LoopbackNetwork must satisfy the
/// same ITransport contract, but only LoopbackNetwork is tested here (UdpSocket
/// would require real network setup). In production, they are swapped via
/// dependency injection.
class LoopbackNetworkTest : public ::testing::Test {
protected:
    LoopbackNetwork network{};
};

TEST_F(LoopbackNetworkTest, Endpoints_can_send_and_receive) {
    auto ep1 = network.create_endpoint(1000);
    auto ep2 = network.create_endpoint(2000);

    const std::array<std::uint8_t, 5> payload{0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
    ASSERT_TRUE(ep1->send(payload, Endpoint{"loopback", 2000}));

    auto received = ep2->receive(std::chrono::milliseconds{100});
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->payload.size(), 5);
    EXPECT_EQ(received->payload[0], 0xAA);
}

TEST_F(LoopbackNetworkTest, Receive_timeout_returns_nullopt) {
    auto ep = network.create_endpoint(3000);
    auto result = ep->receive(std::chrono::milliseconds{10});
    EXPECT_FALSE(result.has_value());
}

TEST_F(LoopbackNetworkTest, Sender_endpoint_is_preserved) {
    auto ep1 = network.create_endpoint(4000);
    auto ep2 = network.create_endpoint(5000);

    const std::array<std::uint8_t, 1> data{42};
    ep1->send(data, Endpoint{"loopback", 5000});

    auto received = ep2->receive(std::chrono::milliseconds{100});
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->from.address, "loopback");
    EXPECT_EQ(received->from.port, 4000);
}

TEST_F(LoopbackNetworkTest, Packet_loss_injection_drops_datagrams) {
    network.set_packet_loss(1.0);  // 100% loss
    auto ep1 = network.create_endpoint(6000);
    auto ep2 = network.create_endpoint(7000);

    const std::array<std::uint8_t, 1> data{0xFF};
    for (int i = 0; i < 10; ++i) {
        ep1->send(data, Endpoint{"loopback", 7000});
    }

    // All dropped: nothing to receive
    for (int i = 0; i < 10; ++i) {
        auto result = ep2->receive(std::chrono::milliseconds{10});
        EXPECT_FALSE(result.has_value());
    }
    EXPECT_EQ(network.dropped(), 10U);
}

TEST_F(LoopbackNetworkTest, Packet_loss_zero_loses_nothing) {
    network.set_packet_loss(0.0);
    auto ep1 = network.create_endpoint(8000);
    auto ep2 = network.create_endpoint(9000);

    const std::array<std::uint8_t, 1> data{0x55};
    for (int i = 0; i < 5; ++i) {
        ASSERT_TRUE(ep1->send(data, Endpoint{"loopback", 9000}));
    }

    for (int i = 0; i < 5; ++i) {
        auto result = ep2->receive(std::chrono::milliseconds{10});
        ASSERT_TRUE(result.has_value());
    }
    EXPECT_EQ(network.delivered(), 5U);
    EXPECT_EQ(network.dropped(), 0U);
}

TEST_F(LoopbackNetworkTest, Multiple_endpoints_isolate_traffic) {
    auto ep1 = network.create_endpoint(10000);
    auto ep2 = network.create_endpoint(11000);
    auto ep3 = network.create_endpoint(12000);

    const std::array<std::uint8_t, 1> data{99};
    ep1->send(data, Endpoint{"loopback", 11000});  // ep1 -> ep2

    auto received = ep2->receive(std::chrono::milliseconds{100});
    ASSERT_TRUE(received.has_value());

    // ep3 should have nothing
    auto nothing = ep3->receive(std::chrono::milliseconds{10});
    EXPECT_FALSE(nothing.has_value());
}

TEST_F(LoopbackNetworkTest, Partition_mode_blocks_all_traffic) {
    network.set_partitioned(true);
    auto ep1 = network.create_endpoint(13000);
    auto ep2 = network.create_endpoint(14000);

    const std::array<std::uint8_t, 1> data{0x77};
    ASSERT_TRUE(ep1->send(data, Endpoint{"loopback", 14000}));  // returns true (UDP semantics)

    auto result = ep2->receive(std::chrono::milliseconds{100});
    EXPECT_FALSE(result.has_value());
    EXPECT_GT(network.dropped(), 0U);
}

TEST_F(LoopbackNetworkTest, Large_payloads_are_preserved) {
    auto ep1 = network.create_endpoint(15000);
    auto ep2 = network.create_endpoint(16000);

    std::vector<std::uint8_t> large(1000);
    for (int i = 0; i < 1000; ++i) {
        large[i] = static_cast<std::uint8_t>(i % 256);
    }

    ep1->send(large, Endpoint{"loopback", 16000});

    auto received = ep2->receive(std::chrono::milliseconds{100});
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(received->payload.size(), large.size());
    for (std::size_t i = 0; i < large.size(); ++i) {
        EXPECT_EQ(received->payload[i], large[i]);
    }
}

TEST_F(LoopbackNetworkTest, Shutdown_empties_inbox) {
    auto ep = network.create_endpoint(17000);
    auto sender = network.create_endpoint(18000);

    const std::array<std::uint8_t, 1> data{44};
    sender->send(data, Endpoint{"loopback", 17000});

    ep->shutdown();
    auto result = ep->receive(std::chrono::milliseconds{10});
    EXPECT_FALSE(result.has_value());
}

}  // namespace
}  // namespace kratt::adapters
