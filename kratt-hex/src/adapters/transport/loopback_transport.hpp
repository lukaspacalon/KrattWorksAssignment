#pragma once

#include <deque>
#include <map>
#include <memory>
#include <random>

#include "adapters/transport/transport.hpp"

namespace kratt::adapters {

/// In-process virtual network. Zero threads, zero sockets, fully synchronous.
///
/// This is what makes the integration suite deterministic: `receive()` returns
/// immediately from an in-memory inbox instead of blocking on poll(), so a full
/// Drone <-> GCS exchange runs inside a single test thread in microseconds.
/// Packet loss is injectable with a fixed seed, so "survives 20% loss" is a
/// reproducible test rather than a hopeful one.
class LoopbackNetwork {
public:
    [[nodiscard]] std::unique_ptr<ITransport> create_endpoint(std::uint16_t port);

    /// Fraction of datagrams dropped on send, in [0, 1]. Deterministic sequence.
    void set_packet_loss(double ratio) noexcept;
    /// Drops every datagram until restored: simulates a total link outage.
    void set_partitioned(bool partitioned) noexcept { partitioned_ = partitioned; }

    [[nodiscard]] std::uint64_t delivered() const noexcept { return delivered_; }
    [[nodiscard]] std::uint64_t dropped() const noexcept { return dropped_; }

    static constexpr const char* kAddress = "loopback";

private:
    class Port;
    friend class Port;

    bool deliver(std::vector<std::uint8_t> payload, const Endpoint& from, const Endpoint& to);
    void unregister(std::uint16_t port) noexcept;

    std::map<std::uint16_t, Port*> ports_;
    double packet_loss_{0.0};
    bool partitioned_{false};
    std::mt19937 rng_{12345};  // fixed seed: loss patterns are reproducible
    std::uint64_t delivered_{0};
    std::uint64_t dropped_{0};
};

}  // namespace kratt::adapters
