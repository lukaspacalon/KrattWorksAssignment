#include <gtest/gtest.h>

#include "domain/geofence.hpp"

namespace kratt::domain {
namespace {

// The geofence is constexpr, so its contract can be checked at compile time.
// A failure here is a build error, not a test failure.
static_assert(Geofence{100.0, 60.0}.contains(Vec3{0, 0, 0}));
static_assert(Geofence{100.0, 60.0}.contains(Vec3{100.0, -100.0, 60.0}));
static_assert(!Geofence{100.0, 60.0}.contains(Vec3{100.1, 0, 10.0}));
static_assert(Geofence{100.0, 60.0}.clamp(Vec3{250, -400, 999}) == Vec3{100.0, -100.0, 60.0});

TEST(GeofenceTest, ContainsIncludesTheBoundary) {
    constexpr Geofence fence{100.0, 60.0};
    EXPECT_TRUE(fence.contains({100.0, 100.0, 60.0}));
    EXPECT_TRUE(fence.contains({-100.0, -100.0, 0.0}));
}

TEST(GeofenceTest, ContainsRejectsOutsidePoints) {
    constexpr Geofence fence{100.0, 60.0};
    EXPECT_FALSE(fence.contains({100.01, 0.0, 10.0}));
    EXPECT_FALSE(fence.contains({0.0, 0.0, -0.01}));  // below ground
    EXPECT_FALSE(fence.contains({0.0, 0.0, 60.01}));  // above the ceiling
}

TEST(GeofenceTest, ClampIsIdentityInside) {
    constexpr Geofence fence{100.0, 60.0};
    constexpr Vec3 inside{12.0, -34.0, 5.0};
    EXPECT_EQ(fence.clamp(inside), inside);
}

TEST(GeofenceTest, ClampAlwaysProducesAContainedPoint) {
    constexpr Geofence fence{100.0, 60.0};
    // Post-condition sweep: the invariant the whole simulation relies on.
    for (double x = -500.0; x <= 500.0; x += 37.0) {
        for (double y = -500.0; y <= 500.0; y += 41.0) {
            for (double z = -50.0; z <= 200.0; z += 23.0) {
                EXPECT_TRUE(fence.contains(fence.clamp({x, y, z})))
                    << "clamp failed for (" << x << ", " << y << ", " << z << ")";
            }
        }
    }
}

}  // namespace
}  // namespace kratt::domain
