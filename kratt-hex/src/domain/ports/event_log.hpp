#pragma once

#include <string_view>

#include "domain/types.hpp"

namespace kratt::domain {

enum class Severity : std::uint8_t { Debug, Info, Warn, Error };

/// OUTBOUND PORT: significant domain events.
///
/// The domain never calls std::cout, std::format or a logging library. It emits
/// a pre-composed, allocation-free view of what happened; the adapter decides
/// whether that becomes a console line, a file, or a GUI event list.
///
/// A null implementation (`NullEventLog`) is the default, so the domain has no
/// mandatory collaborator.
class IEventLog {
public:
    virtual ~IEventLog() = default;

    IEventLog(const IEventLog&) = delete;
    IEventLog& operator=(const IEventLog&) = delete;

    virtual void record(Severity severity, std::string_view event, Instant now) = 0;

protected:
    IEventLog() = default;
};

/// Null Object: lets the domain be constructed with no logging collaborator at
/// all, which keeps the unit tests free of setup noise.
class NullEventLog final : public IEventLog {
public:
    void record(Severity, std::string_view, Instant) override {}

    static NullEventLog& instance() {
        static NullEventLog log;
        return log;
    }
};

}  // namespace kratt::domain
