#include <gtest/gtest.h>

#include "domain/flight_dynamics.hpp"

namespace kratt::domain::dynamics {
namespace {

TEST(DynamicsTest, Integrate_is_referentially_transparent) {
    const FlightConfig cfg;
    const FlightState state;
    const auto first = integrate(state, cfg, 0.1, Instant{});
    const auto second = integrate(state, cfg, 0.1, Instant{});
    EXPECT_EQ(first.state.telemetry.position, second.state.telemetry.position);
}

TEST(DynamicsTest, Integrate_does_not_mutate_input_state) {
    const FlightConfig cfg;
    FlightState state;
    state.telemetry.mode = FlightMode::Hold;
    state.telemetry.armed = true;
    const Telemetry before = state.telemetry;

    (void)integrate(state, cfg, 1.0, Instant{});
    EXPECT_EQ(state.telemetry, before) << "state must be passed by value";
}

TEST(DynamicsTest, Desired_velocity_respects_speed_envelope) {
    const FlightConfig cfg;
    FlightState state;
    state.telemetry.mode = FlightMode::Manual;
    state.manual_axes = Vec3{1.0, 1.0, 1.0};  // worst case, unit-clamped
    const auto v = desired_velocity(state, cfg, Instant{});
    EXPECT_LE(v.norm(), cfg.horizontal_speed_mps * std::sqrt(3.0) + 1e-6);
}

TEST(DynamicsTest, Desired_velocity_is_zero_in_hold_mode) {
    const FlightConfig cfg;
    FlightState state;
    state.telemetry.mode = FlightMode::Hold;
    const auto v = desired_velocity(state, cfg, Instant{});
    EXPECT_EQ(v, Vec3{});
}

TEST(DynamicsTest, Desired_velocity_is_zero_when_disarmed) {
    const FlightConfig cfg;
    FlightState state;
    state.telemetry.armed = false;
    const auto v = desired_velocity(state, cfg, Instant{});
    EXPECT_EQ(v, Vec3{});
}

TEST(DynamicsTest, Manual_timeout_stops_velocity) {
    const FlightConfig cfg;
    FlightState state;
    state.telemetry.mode = FlightMode::Manual;
    state.manual_axes = Vec3{1.0, 0.0, 0.0};
    state.last_manual_input = Instant{std::chrono::milliseconds{0}};

    // 1000 seconds later, the 500ms timeout has definitely expired
    const auto v = desired_velocity(state, cfg, Instant{std::chrono::seconds{1000}});
    EXPECT_EQ(v, Vec3{});
}

}  // namespace
}  // namespace kratt::domain::dynamics
