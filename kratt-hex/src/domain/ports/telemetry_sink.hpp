#pragma once

#include "domain/types.hpp"

namespace kratt::domain {

/// OUTBOUND PORT (driven side).
///
/// The domain publishes state; it does not know whether the implementation
/// encodes MAVLink onto a UDP socket, appends to a flight log, or records into
/// a vector for a test assertion.
class ITelemetrySink {
public:
    virtual ~ITelemetrySink() = default;

    ITelemetrySink(const ITelemetrySink&) = delete;
    ITelemetrySink& operator=(const ITelemetrySink&) = delete;

    /// Called at the publication rate decided by the domain service (10 Hz).
    /// Implementations must not block: the caller is the simulation loop.
    virtual void publish(const Telemetry& telemetry, Instant now) = 0;

protected:
    ITelemetrySink() = default;
};

/// OUTBOUND PORT: link liveness.
///
/// Deliberately separate from ITelemetrySink. Only a HEARTBEAT proves the peer
/// is alive; a position message may legitimately be absent or stale. Folding
/// the two into one port would let stale data keep a dead link looking healthy.
class ILinkLiveness {
public:
    virtual ~ILinkLiveness() = default;
    ILinkLiveness(const ILinkLiveness&) = delete;
    ILinkLiveness& operator=(const ILinkLiveness&) = delete;

    virtual void on_heartbeat(Instant now) = 0;

protected:
    ILinkLiveness() = default;
};

/// OUTBOUND PORT: acknowledgement of a command that requires one.
///
/// Separate from ITelemetrySink because the two have different lifetimes and
/// different consumers — Interface Segregation. An adapter is free to implement
/// both.
class ICommandAcknowledger {
public:
    virtual ~ICommandAcknowledger() = default;

    ICommandAcknowledger(const ICommandAcknowledger&) = delete;
    ICommandAcknowledger& operator=(const ICommandAcknowledger&) = delete;

    virtual void acknowledge(CommandResult result, Instant now) = 0;

protected:
    ICommandAcknowledger() = default;
};

}  // namespace kratt::domain
