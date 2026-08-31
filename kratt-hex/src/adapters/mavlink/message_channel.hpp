#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "adapters/mavlink/mavlink_codec.hpp"
#include "adapters/transport/transport.hpp"

namespace kratt::adapters::mavlink {

struct Inbound {
    mavlink_message_t message{};
    Endpoint from{};
};

/// Adapter-level port: a bidirectional stream of MAVLink messages.
///
/// This is the seam where threading becomes a *choice*. Production wires a
/// `ThreadedChannel`; the integration tests wire a `SynchronousChannel` and run
/// the entire Drone <-> GCS exchange in one thread. Neither the domain nor the
/// MAVLink translation layer can tell the difference.
class IMessageChannel {
public:
    virtual ~IMessageChannel() = default;
    IMessageChannel(const IMessageChannel&) = delete;
    IMessageChannel& operator=(const IMessageChannel&) = delete;

    /// Queues or sends a message. Must never block the caller.
    virtual void send(const mavlink_message_t& message) = 0;
    /// Returns every message received since the last call. Must never block.
    [[nodiscard]] virtual std::vector<Inbound> poll() = 0;

    /// Convenience alias: `poll()` with no waiting. Present so callers read
    /// naturally at the call site.
    [[nodiscard]] std::vector<Inbound> poll_now() { return poll(); }

protected:
    IMessageChannel() = default;
};

/// Synchronous channel: sends straight through the transport, polls it inline.
/// No thread, no queue, no lock. This is the whole reason the test suite is
/// deterministic.
class SynchronousChannel final : public IMessageChannel {
public:
    struct Config {
        /// Peer to send to. Empty on the GCS, which learns it from the first
        /// datagram received (the drone is the UDP client).
        std::optional<Endpoint> peer{};
        bool learn_peer_from_traffic{true};
    };

    SynchronousChannel(std::unique_ptr<ITransport> transport, Config config);

    void send(const mavlink_message_t& message) override;

    /// Non-blocking drain, as required by IMessageChannel.
    [[nodiscard]] std::vector<Inbound> poll() override;

    /// Blocking-with-timeout variant used by ThreadedChannel's worker: it waits
    /// up to `timeout` for the first datagram, then drains whatever else is
    /// already queued. Keeping the wait here means the worker never has to know
    /// about the transport or the decoder.
    [[nodiscard]] std::vector<Inbound> poll(std::chrono::milliseconds timeout);

    void set_peer(const Endpoint& peer) { config_.peer = peer; }
    [[nodiscard]] const std::optional<Endpoint>& peer() const noexcept { return config_.peer; }
    [[nodiscard]] const Decoder::Stats& decoder_stats() const noexcept {
        return decoder_.stats();
    }
    [[nodiscard]] std::uint64_t dropped_no_peer() const noexcept { return dropped_no_peer_; }
    [[nodiscard]] ITransport& transport() noexcept { return *transport_; }

private:
    std::unique_ptr<ITransport> transport_;
    Config config_;
    Decoder decoder_;
    std::uint64_t dropped_no_peer_{0};
};

}  // namespace kratt::adapters::mavlink
