#include "adapters/transport/loopback_transport.hpp"

#include <stdexcept>

namespace kratt::adapters {

class LoopbackNetwork::Port final : public ITransport {
public:
    Port(LoopbackNetwork& network, std::uint16_t port)
        : network_{network}, local_{Endpoint{kAddress, port}} {}

    ~Port() override { network_.unregister(local_.port); }

    bool send(std::span<const std::uint8_t> payload, const Endpoint& to) override {
        return network_.deliver({payload.begin(), payload.end()}, local_, to);
    }

    std::optional<Datagram> receive(std::chrono::milliseconds) override {
        // Single-threaded by design: there is nothing to wait for, so the
        // timeout is ignored and the call returns at once. Blocking here would
        // deadlock the test thread that also drives the sender.
        if (inbox_.empty()) {
            return std::nullopt;
        }
        Datagram datagram = std::move(inbox_.front());
        inbox_.pop_front();
        return datagram;
    }

    [[nodiscard]] Endpoint local_endpoint() const override { return local_; }
    void shutdown() override { inbox_.clear(); }
    void enqueue(Datagram datagram) { inbox_.push_back(std::move(datagram)); }

private:
    LoopbackNetwork& network_;
    Endpoint local_;
    std::deque<Datagram> inbox_;
};

std::unique_ptr<ITransport> LoopbackNetwork::create_endpoint(std::uint16_t port) {
    if (ports_.contains(port)) {
        throw std::runtime_error{"loopback port already in use: " + std::to_string(port)};
    }
    auto endpoint = std::make_unique<Port>(*this, port);
    ports_.emplace(port, endpoint.get());
    return endpoint;
}

void LoopbackNetwork::set_packet_loss(double ratio) noexcept {
    packet_loss_ = ratio < 0.0 ? 0.0 : (ratio > 1.0 ? 1.0 : ratio);
}

bool LoopbackNetwork::deliver(std::vector<std::uint8_t> payload, const Endpoint& from,
                              const Endpoint& to) {
    if (partitioned_) {
        ++dropped_;
        return true;  // UDP: a lost datagram is not a send error
    }
    if (packet_loss_ > 0.0) {
        std::uniform_real_distribution<double> dist{0.0, 1.0};
        if (dist(rng_) < packet_loss_) {
            ++dropped_;
            return true;
        }
    }
    const auto it = ports_.find(to.port);
    if (it == ports_.end()) {
        ++dropped_;
        return true;  // nothing listening, same as a real UDP send into the void
    }
    ++delivered_;
    it->second->enqueue(Datagram{std::move(payload), from});
    return true;
}

void LoopbackNetwork::unregister(std::uint16_t port) noexcept { ports_.erase(port); }

}  // namespace kratt::adapters
