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
#include "app/gcs_composition.hpp"

#if defined(KRATT_WITH_GUI)
namespace kratt::bin {
/// Implemented in gcs_gui.cpp, compiled only when ImGui and GLFW are available.
/// Declared here so this file never includes a GUI header.
int run_gui(app::GcsComposition& gcs, const domain::Geofence& fence);
}  // namespace kratt::bin
#endif

namespace {

std::atomic<bool> g_running{true};
extern "C" void handle_signal(int) { g_running.store(false, std::memory_order_release); }

struct Options {
    kratt::adapters::Endpoint bind{"0.0.0.0", 14550};
    double fence_half_extent{100.0};
    double fence_ceiling{60.0};
};

}  // namespace

int main(int argc, char** argv) {
    using namespace kratt;
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg{argv[i]};
        const auto next = [&] { return (i + 1 < argc) ? std::string{argv[++i]} : std::string{}; };
        if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0]
                      << " [--bind-port <p>] [--fence <m>] [--ceiling <m>]\n";
            return EXIT_SUCCESS;
        }
        if (arg == "--bind-port") {
            options.bind.port = static_cast<std::uint16_t>(std::stoi(next()));
        } else if (arg == "--fence") {
            options.fence_half_extent = std::stod(next());
        } else if (arg == "--ceiling") {
            options.fence_ceiling = std::stod(next());
        }
    }

    adapters::ConsoleEventLog event_log{"gcs"};

    try {
        auto socket = std::make_unique<adapters::UdpSocket>(options.bind);

        adapters::mavlink::SynchronousChannel::Config channel_config;
        // The GCS is the server: no configured peer, it discovers the drone's
        // address from the first frame it receives.
        channel_config.learn_peer_from_traffic = true;

        auto inner = std::make_unique<adapters::mavlink::SynchronousChannel>(std::move(socket),
                                                                            channel_config);
        adapters::mavlink::ThreadedChannel channel{std::move(inner)};

        app::GcsComposition::Config config;
        const domain::Geofence fence{options.fence_half_extent, options.fence_ceiling};
        config.service.fence = fence;

        app::GcsComposition gcs{channel, config, event_log};

        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);
        std::cout << "GCS listening on " << options.bind.to_string() << "\n";

        channel.start();
        int result = EXIT_SUCCESS;
#if defined(KRATT_WITH_GUI)
        // The GUI owns the main startup thread, as the specification requires.
        result = bin::run_gui(gcs, fence);
#else
        std::cout << "built without the GUI (libs/imgui or libs/glfw missing) - headless\n";
        const auto started = std::chrono::steady_clock::now();
        while (g_running.load(std::memory_order_acquire)) {
            const auto now = std::chrono::steady_clock::now();
            gcs.tick(std::chrono::duration_cast<domain::Instant>(now - started));
            std::this_thread::sleep_for(std::chrono::milliseconds{20});
        }
#endif
        channel.stop();
        return result;
    } catch (const std::exception& error) {
        std::cerr << "fatal: " << error.what() << "\n";
        return EXIT_FAILURE;
    }
}
