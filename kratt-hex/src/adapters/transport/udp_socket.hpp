#pragma once

#include <array>
#include <atomic>

#include "adapters/transport/transport.hpp"

namespace kratt::adapters {

/// Blocking-with-timeout UDP socket, POSIX and Winsock, wrapped in RAII.
///
/// Own wrapper rather than SimpleUDP: it removes a submodule, behaves
/// identically on both platforms, and lets us own the shutdown semantics —
/// `shutdown()` must reliably unblock a pending `receive()`, which is what the
/// clean thread teardown depends on.
class UdpSocket final : public ITransport {
public:
    /// Port 0 lets the OS pick; the effective port is then readable through
    /// `local_endpoint()`. Throws std::runtime_error on failure.
    explicit UdpSocket(const Endpoint& bind_address);
    ~UdpSocket() override;

    bool send(std::span<const std::uint8_t> payload, const Endpoint& to) override;
    std::optional<Datagram> receive(std::chrono::milliseconds timeout) override;
    [[nodiscard]] Endpoint local_endpoint() const override { return local_; }
    void shutdown() override;

private:
    /// MAVLink2 frames are at most 280 bytes; 2 KiB leaves headroom.
    static constexpr std::size_t kBufferSize = 2048;

    std::intptr_t fd_{-1};
    Endpoint local_{};
    std::atomic<bool> shutting_down_{false};
    std::array<std::uint8_t, kBufferSize> buffer_{};
};

}  // namespace kratt::adapters
