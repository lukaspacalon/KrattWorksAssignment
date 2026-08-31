#include "adapters/mavlink/threaded_channel.hpp"

#include <utility>

namespace kratt::adapters::mavlink {

ThreadedChannel::ThreadedChannel(std::unique_ptr<SynchronousChannel> inner,
                                 std::chrono::milliseconds poll_interval)
    : inner_{std::move(inner)}, poll_interval_{poll_interval} {}

ThreadedChannel::~ThreadedChannel() { stop(); }

void ThreadedChannel::start() {
    if (running_.exchange(true)) {
        return;
    }
    worker_ = std::thread{[this] { run(); }};
}

void ThreadedChannel::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    inner_->transport().shutdown();  // unblocks the pending receive()
    rx_.close();
    tx_.close();
    if (worker_.joinable()) {
        worker_.join();
    }
}

void ThreadedChannel::send(const mavlink_message_t& message) { tx_.push(message); }

std::vector<Inbound> ThreadedChannel::poll() { return rx_.drain(); }

void ThreadedChannel::run() {
    while (running_.load(std::memory_order_acquire)) {
        // One blocking-with-timeout drain, so the loop reacts to stop() within
        // one poll interval instead of spinning at 100% CPU.
        for (auto& message : inner_->poll(poll_interval_)) {
            rx_.push(std::move(message));
        }
        for (const auto& message : tx_.drain()) {
            inner_->send(message);
        }
    }
    for (const auto& message : tx_.drain()) {
        inner_->send(message);  // best effort: let a final ACK go out
    }
}

}  // namespace kratt::adapters::mavlink
