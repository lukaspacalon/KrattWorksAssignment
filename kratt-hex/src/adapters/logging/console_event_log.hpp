#pragma once

#include <cstdio>
#include <mutex>
#include <string>

#include "domain/ports/event_log.hpp"

namespace kratt::adapters {

/// SECONDARY ADAPTER — writes domain events to stderr.
///
/// The domain emits a severity, a short event name and an instant; this class
/// decides on the timestamp format, the colours and the destination. Swap it for
/// a file writer or a GUI list and not one line of the domain changes.
///
/// Serialised by a mutex so lines from the simulation thread and the link thread
/// never interleave.
class ConsoleEventLog final : public domain::IEventLog {
public:
    explicit ConsoleEventLog(std::string tag, domain::Severity minimum = domain::Severity::Info)
        : tag_{std::move(tag)}, minimum_{minimum} {}

    void record(domain::Severity severity, std::string_view event, domain::Instant now) override {
        if (severity < minimum_) {
            return;
        }
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
        const std::lock_guard lock{mutex_};
        std::fprintf(stderr, "[%8lld ms] [%s] [%-8.8s] %.*s\n", static_cast<long long>(ms),
                     to_string(severity).data(), tag_.c_str(), static_cast<int>(event.size()),
                     event.data());
    }

private:
    static constexpr std::string_view to_string(domain::Severity severity) noexcept {
        switch (severity) {
            case domain::Severity::Debug: return "DEBUG";
            case domain::Severity::Info:  return "INFO ";
            case domain::Severity::Warn:  return "WARN ";
            case domain::Severity::Error: return "ERROR";
        }
        return "?????";
    }

    std::string tag_;
    domain::Severity minimum_;
    std::mutex mutex_;
};

/// SECONDARY ADAPTER for tests: keeps events in memory so assertions can read
/// them, instead of scraping stderr.
class RecordingEventLog final : public domain::IEventLog {
public:
    struct Entry {
        domain::Severity severity;
        std::string event;
        domain::Instant at;
    };

    void record(domain::Severity severity, std::string_view event, domain::Instant now) override {
        entries_.push_back(Entry{severity, std::string{event}, now});
    }

    [[nodiscard]] const std::vector<Entry>& entries() const noexcept { return entries_; }
    [[nodiscard]] bool contains(std::string_view event) const {
        for (const auto& entry : entries_) {
            if (entry.event == event) {
                return true;
            }
        }
        return false;
    }
    void clear() { entries_.clear(); }

private:
    std::vector<Entry> entries_;
};

}  // namespace kratt::adapters
