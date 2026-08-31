#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

#include "adapters/mavlink/message_channel.hpp"

namespace kratt::adapters::mavlink {

/// Bounded thread-safe queue. The single synchronisation primitive in the whole
/// project.
///
/// Bounded on purpose: if a consumer stalls, dropping the oldest message is far
/// better than growing without limit. `dropped()` makes that visible in the
/// logs rather than silent, and it is what keeps memory usage provably flat
/// under a command flood.
template <typename T>
class BoundedQueue {
public:
    explicit BoundedQueue(std::size_t capacity = 512) : capacity_{capacity} {}

    bool push(T value) {
        bool ok = true;
        {
            const std::lock_guard lock{mutex_};
            if (items_.size() >= capacity_) {
                items_.pop_front();
                ++dropped_;
                ok = false;
            }
            items_.push_back(std::move(value));
        }
        cv_.notify_one();
        return ok;
    }

    [[nodiscard]] std::vector<T> drain() {
        const std::lock_guard lock{mutex_};
        std::vector<T> out;
        out.reserve(items_.size());
        for (auto& item : items_) {
            out.push_back(std::move(item));
        }
        items_.clear();
        return out;
    }

    void close() {
        {
            const std::lock_guard lock{mutex_};
            closed_ = true;
        }
        cv_.notify_all();
    }

    [[nodiscard]] std::size_t size() const {
        const std::lock_guard lock{mutex_};
        return items_.size();
    }
    [[nodiscard]] std::size_t dropped() const {
        const std::lock_guard lock{mutex_};
        return dropped_;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<T> items_;
    std::size_t capacity_;
    std::size_t dropped_{0};
    bool closed_{false};
};

/// Decorator that moves a SynchronousChannel onto its own worker thread.
///
/// The worker owns the socket and the decoder, so neither needs a lock. The
/// application thread only ever touches the two queues. Consequences:
///   * `send()` and `poll()` never block on I/O, so the 100 Hz simulation loop
///     and the GUI loop keep their timing — the explicit requirement of the
///     specification;
///   * teardown is deterministic: shutdown() unblocks the poll, the destructor
///     joins, and no callback can outlive the object.
///
/// One worker rather than separate rx and tx threads bounds outgoing latency by
/// the poll interval (5 ms), an order of magnitude below the 10 Hz telemetry
/// period, while halving the thread count and removing a race.
class ThreadedChannel final : public IMessageChannel {
public:
    ThreadedChannel(std::unique_ptr<SynchronousChannel> inner,
                    std::chrono::milliseconds poll_interval = std::chrono::milliseconds{5});
    ~ThreadedChannel() override;

    void start();
    void stop();

    void send(const mavlink_message_t& message) override;
    [[nodiscard]] std::vector<Inbound> poll() override;

    [[nodiscard]] std::size_t rx_dropped() const { return rx_.dropped(); }
    [[nodiscard]] std::size_t tx_dropped() const { return tx_.dropped(); }

private:
    void run();

    std::unique_ptr<SynchronousChannel> inner_;  ///< worker thread only, after start()
    std::chrono::milliseconds poll_interval_;
    BoundedQueue<Inbound> rx_;
    BoundedQueue<mavlink_message_t> tx_;
    std::atomic<bool> running_{false};
    std::thread worker_;
};

}  // namespace kratt::adapters::mavlink
