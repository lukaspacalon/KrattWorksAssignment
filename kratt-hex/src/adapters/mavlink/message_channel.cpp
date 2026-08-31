#include "adapters/mavlink/message_channel.hpp"

#include <utility>

namespace kratt::adapters::mavlink {

SynchronousChannel::SynchronousChannel(std::unique_ptr<ITransport> transport, Config config)
    : transport_{std::move(transport)}, config_{std::move(config)} {}

void SynchronousChannel::send(const mavlink_message_t& message) {
    if (!config_.peer) {
        ++dropped_no_peer_;  // counted, never silent
        return;
    }
    const auto bytes = serialize(message);
    transport_->send(bytes, *config_.peer);
}

std::vector<Inbound> SynchronousChannel::poll() {
    return poll(std::chrono::milliseconds{0});
}

std::vector<Inbound> SynchronousChannel::poll(std::chrono::milliseconds timeout) {
    std::vector<Inbound> inbound;
    // First read may wait; every subsequent one drains without waiting, so a
    // burst of datagrams is consumed in a single pass.
    auto remaining = timeout;
    while (auto datagram = transport_->receive(remaining)) {
        remaining = std::chrono::milliseconds{0};
        const auto messages = decoder_.parse(datagram->payload);
        if (!messages.empty() && config_.learn_peer_from_traffic) {
            config_.peer = datagram->from;
        }
        for (const auto& message : messages) {
            inbound.push_back(Inbound{message, datagram->from});
        }
    }
    return inbound;
}

}  // namespace kratt::adapters::mavlink
