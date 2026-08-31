#include <gtest/gtest.h>

#include "domain/kinematics.hpp"

namespace kratt::domain::kinematics {
namespace {

static_assert(integrate(Vec3{0, 0, 0}, Vec3{10, 0, 0}, 1.0) == Vec3{10, 0, 0});
static_assert(derive(Vec3{10, 0, 0}, Vec3{0, 0, 0}, 1.0) == Vec3{10, 0, 0});
static_assert(derive(Vec3{1, 0, 0}, Vec3{0, 0, 0}, 0.0) == Vec3{});  // no division by zero

TEST(KinematicsTest, IntegrateIsExactOverManySmallSteps) {
    Vec3 position{};
    constexpr Vec3 velocity{10.0, 0.0, 0.0};
    constexpr Seconds dt = 0.01;
    for (int i = 0; i < 100; ++i) {
        position = integrate(position, velocity, dt);
    }
    EXPECT_NEAR(position.x, 10.0, 1e-9);
}

TEST(KinematicsTest, DeriveRecoversTheVelocityThatWasIntegrated) {
    constexpr Vec3 start{1.0, 2.0, 3.0};
    constexpr Vec3 velocity{10.0, -5.0, 2.0};
    constexpr Seconds dt = 0.1;
    const Vec3 moved = integrate(start, velocity, dt);
    const Vec3 recovered = derive(moved, start, dt);
    EXPECT_NEAR(recovered.x, velocity.x, 1e-9);
    EXPECT_NEAR(recovered.y, velocity.y, 1e-9);
    EXPECT_NEAR(recovered.z, velocity.z, 1e-9);
}

TEST(KinematicsTest, VelocityTowardsHasTheCommandedMagnitude) {
    const Vec3 v = velocity_towards(Vec3{0, 0, 0}, Vec3{30, 40, 0}, 10.0, 0.25);
    EXPECT_NEAR(v.norm(), 10.0, 1e-9);
    EXPECT_NEAR(v.x, 6.0, 1e-9);  // 10 * 30/50
    EXPECT_NEAR(v.y, 8.0, 1e-9);  // 10 * 40/50
}

TEST(KinematicsTest, VelocityTowardsIsNullWithinTolerance) {
    EXPECT_EQ(velocity_towards(Vec3{0, 0, 0}, Vec3{0.1, 0, 0}, 10.0, 0.25), Vec3{});
}

TEST(KinematicsTest, DiagonalStickInputNeverExceedsTheEnvelope) {
    // Three axes at full deflection must not produce sqrt(3) times the speed.
    const Vec3 clamped = clamp_to_unit_ball({1.0, 1.0, 1.0});
    EXPECT_NEAR(clamped.norm(), 1.0, 1e-9);
}

}  // namespace
}  // namespace kratt::domain::kinematics
