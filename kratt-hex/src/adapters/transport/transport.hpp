#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace kratt::adapters {

struct Endpoint {
    std::string address{"127.0.0.1"};
    std::uint16_t port{0};

    friend bool operator==(const Endpoint&, const Endpoint&) = default;
    [[nodiscard]] std::string to_string() const { return address + ":" + std::to_string(port); }
};

struct Datagram {
    std::vector<std::uint8_t> payload;
    Endpoint from;
};

/// Datagram transport abstraction.
///
/// Note this interface lives in the adapter layer, not the domain: the domain
/// has no concept of a datagram at all. It exists so that the *MAVLink adapter*
/// can be tested against an in-memory network, and so that swapping UDP for
/// anything else is a one-class change.
class ITransport {
public:
    virtual ~ITransport() = default;
    ITransport(const ITransport&) = delete;
    ITransport& operator=(const ITransport&) = delete;

    virtual bool send(std::span<const std::uint8_t> payload, const Endpoint& to) = 0;
    /// Waits up to `timeout`. Returns nullopt on timeout.
    virtual std::optional<Datagram> receive(std::chrono::milliseconds timeout) = 0;
    [[nodiscard]] virtual Endpoint local_endpoint() const = 0;
    /// Unblocks a pending receive() so a worker thread can exit promptly.
    virtual void shutdown() = 0;

protected:
    ITransport() = default;
};

}  // namespace kratt::adapters
