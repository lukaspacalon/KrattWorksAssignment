#pragma once

#include "domain/types.hpp"

namespace kratt::domain {

/// INBOUND PORT (driving side).
///
/// The vocabulary in which the outside world talks to the drone. Note what is
/// absent: no mavlink_message_t, no byte buffer, no socket. A MAVLink adapter,
/// a CLI adapter, a replay adapter and a test driver all speak this same
/// interface, which is why the domain can be exercised with zero I/O.
///
/// `now` is a parameter, never read from a clock, so every call is replayable.
class IFlightCommands {
public:
    virtual ~IFlightCommands() = default;

    IFlightCommands(const IFlightCommands&) = delete;
    IFlightCommands& operator=(const IFlightCommands&) = delete;

    /// Arm and begin the automatic climb to the take-off altitude.
    virtual CommandResult arm(Instant now) = 0;

    /// Disarm. Below the safety altitude this cuts the motors; above it, the
    /// request is converted into a landing and `AcceptedAsLand` is returned.
    virtual CommandResult disarm(Instant now) = 0;

    /// Fly to an absolute position. A target outside the fence is clamped.
    virtual CommandResult goto_target(const Vec3& target, Instant now) = 0;

    /// Begin a gentle descent, then disarm on touchdown.
    virtual CommandResult land(Instant now) = 0;

    /// Normalised stick input in [-1, 1] per axis. A zero vector means
    /// "sticks centred": hold position immediately.
    virtual CommandResult manual_input(const Vec3& axes, Instant now) = 0;

protected:
    IFlightCommands() = default;
};

}  // namespace kratt::domain
