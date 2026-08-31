#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "adapters/logging/console_event_log.hpp"
#include "adapters/mavlink/threaded_channel.hpp"
#include "adapters/transport/udp_socket.hpp"
#include "app/drone_composition.hpp"

namespace {

std::atomic<bool> g_running{true};

extern "C" void handle_signal(int) {
    // Async-signal-safe: flip a flag, let the loop exit and every destructor run.
    g_running.store(false, std::memory_order_release);
}

struct Options {
    kratt::adapters::Endpoint bind{"0.0.0.0", 14551};
    kratt::adapters::Endpoint gcs{"127.0.0.1", 14550};
    double fence_half_extent{100.0};
    double fence_ceiling{60.0};
    int simulation_hz{100};
};

void print_usage(const char* program) {
    std::cout << "Usage: " << program << " [options]\n"
              << "  --bind-port <p>   local UDP port          (default 14551)\n"
              << "  --gcs <ip:port>   ground station address  (default 127.0.0.1:14550)\n"
              << "  --fence <m>       geofence half-extent    (default 100)\n"
              << "  --ceiling <m>     geofence ceiling        (default 60)\n"
              << "  --rate <hz>       simulation rate         (default 100)\n"
              << "  --help\n";
}

bool parse(int argc, char** argv, Options& options, int& exit_code) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg{argv[i]};
        const auto next = [&] { return (i + 1 < argc) ? std::string{argv[++i]} : std::string{}; };
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            exit_code = EXIT_SUCCESS;
            return false;
        }
        if (arg == "--bind-port") {
            options.bind.port = static_cast<std::uint16_t>(std::stoi(next()));
        } else if (arg == "--gcs") {
            const std::string value = next();
            const auto colon = value.rfind(':');
            if (colon == std::string::npos) {
                std::cerr << "invalid --gcs, expected ip:port\n";
                exit_code = EXIT_FAILURE;
                return false;
            }
            options.gcs.address = value.substr(0, colon);
            options.gcs.port = static_cast<std::uint16_t>(std::stoi(value.substr(colon + 1)));
        } else if (arg == "--fence") {
            options.fence_half_extent = std::stod(next());
        } else if (arg == "--ceiling") {
            options.fence_ceiling = std::stod(next());
        } else if (arg == "--rate") {
            options.simulation_hz = std::stoi(next());
        } else {
            std::cerr << "unknown option: " << arg << "\n";
            print_usage(argv[0]);
            exit_code = EXIT_FAILURE;
            return false;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    using namespace kratt;
    Options options;
    int exit_code = EXIT_SUCCESS;
    if (!parse(argc, argv, options, exit_code)) {
        return exit_code;
    }

    adapters::ConsoleEventLog event_log{"drone"};

    try {
        // --- Composition: choose the adapters, wire them, then get out of the way.
        auto socket = std::make_unique<adapters::UdpSocket>(options.bind);

        adapters::mavlink::SynchronousChannel::Config channel_config;
        channel_config.peer = options.gcs;
        // The drone is the UDP client: it knows where the GCS is and keeps that
        // address, so a stray datagram cannot hijack the telemetry stream.
        channel_config.learn_peer_from_traffic = false;

        auto inner = std::make_unique<adapters::mavlink::SynchronousChannel>(std::move(socket),
                                                                            channel_config);
        adapters::mavlink::ThreadedChannel channel{std::move(inner)};

        app::DroneComposition::Config config;
        config.service.flight.fence =
            domain::Geofence{options.fence_half_extent, options.fence_ceiling};

        app::DroneComposition drone{channel, config, event_log};

        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);

        std::cout << "drone listening on " << options.bind.to_string() << ", streaming to "
                  << options.gcs.to_string() << "\n"
                  << "geofence +/-" << options.fence_half_extent << " m, ceiling "
                  << options.fence_ceiling << " m\n";

        channel.start();  // the only thread in the process, and it lives here

        // --- Simulation loop: the main startup thread, as the specification requires.
        const auto period = std::chrono::nanoseconds{std::chrono::seconds{1}} /
                            options.simulation_hz;
        const auto started = std::chrono::steady_clock::now();
        auto next_tick = started;
        auto previous = started;

        while (g_running.load(std::memory_order_acquire)) {
            const auto now = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration<double>(now - previous).count();
            previous = now;

            // The wall clock is converted to a domain Instant here, at the edge.
            // Inside the hexagon, time is just a number that was handed in.
            drone.tick(dt, std::chrono::duration_cast<domain::Instant>(now - started));

            // Absolute scheduling avoids the drift a sleep(period) loop
            // accumulates, and sleep_until keeps the process near 0% CPU.
            next_tick += period;
            if (next_tick < now) {
                next_tick = now + period;  // recover from an overrun, never spin
            }
            std::this_thread::sleep_until(next_tick);
        }

        channel.stop();
        std::cout << "drone stopped cleanly\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "fatal: " << error.what() << "\n";
        return EXIT_FAILURE;
    }
}
