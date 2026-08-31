#pragma once

#include <optional>

#include "domain/types.hpp"

namespace kratt::domain {

/// Derives Connected / Disconnected from the heartbeat stream.
///
/// Pure timing logic, no clock of its own: instants are injected. The GCS side
/// of the domain, and the reason the connection tests need no sleep.
class ConnectionMonitor {
public:
    /// The drone beats at 10 Hz. A 1 s timeout tolerates about ten consecutive
    /// losses before declaring the link down — long enough not to flap on a
    /// lossy link, short enough for an operator to notice.
    explicit constexpr ConnectionMonitor(
        Instant timeout = std::chrono::milliseconds{1000}) noexcept
        : timeout_{timeout} {}

    constexpr void on_heartbeat(Instant now) noexcept { last_heartbeat_ = now; }

    /// Re-evaluates the status. Returns true when it changed on this call, so
    /// the caller can emit exactly one event per transition.
    [[nodiscard]] constexpr bool update(Instant now) noexcept {
        const bool alive = last_heartbeat_.has_value() && (now - *last_heartbeat_) < timeout_;
        if (alive == connected_) {
            return false;
        }
        connected_ = alive;
        return true;
    }

    [[nodiscard]] constexpr bool connected() const noexcept { return connected_; }
    [[nodiscard]] constexpr Instant timeout() const noexcept { return timeout_; }

private:
    Instant timeout_;
    std::optional<Instant> last_heartbeat_{};
    bool connected_{false};
};

}  // namespace kratt::domain
