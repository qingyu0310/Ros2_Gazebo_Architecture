#include <cmath>

#include <gtest/gtest.h>

#include "leg_height.hpp"

namespace
{

constexpr double kTolerance = 1.0e-10;

LegHeightKinematics make_kinematics()
{
    return LegHeightKinematics({
        0.056,
        0.6981317008,
        -0.1618,
        0.5109,
        -1.0345,
        0.3363,
    });
}

TEST(LegHeightKinematicsTest, NominalHeightMapsToZeroJointAngles)
{
    const auto kinematics = make_kinematics();
    const auto reference = kinematics.from_height(kinematics.nominal_height());

    EXPECT_NEAR(kinematics.nominal_height(), 0.0719922123, 1.0e-9);
    EXPECT_NEAR(reference.hip, 0.0, kTolerance);
    EXPECT_NEAR(reference.knee, 0.0, kTolerance);
}

TEST(LegHeightKinematicsTest, HeightRoundTripKeepsSymmetricLegGeometry)
{
    const auto kinematics = make_kinematics();
    const double requested = 0.085;
    const auto reference = kinematics.from_height(requested);

    EXPECT_NEAR(reference.height, requested, kTolerance);
    EXPECT_NEAR(reference.knee, -2.0 * reference.hip, kTolerance);
    EXPECT_NEAR(kinematics.height_from_hip(reference.hip), requested, kTolerance);
}

TEST(LegHeightKinematicsTest, RequestedHeightIsClampedToSoftLimitRange)
{
    const auto kinematics = make_kinematics();
    const auto low = kinematics.from_height(0.0);
    const auto high = kinematics.from_height(1.0);

    EXPECT_NEAR(low.height, kinematics.minimum_height(), kTolerance);
    EXPECT_NEAR(high.height, kinematics.maximum_height(), kTolerance);
    EXPECT_GE(low.hip, -0.1618 - kTolerance);
    EXPECT_LE(high.hip, 0.5109 + kTolerance);
    EXPECT_LE(low.knee, 0.3363 + kTolerance);
    EXPECT_GE(high.knee, -1.0345 - kTolerance);
}

TEST(LegHeightKinematicsTest, HeightReferenceObeysRateLimit)
{
    EXPECT_NEAR(LegHeightKinematics::move_towards(0.070, 0.090, 0.01, 0.001),
                0.07001, kTolerance);
    EXPECT_NEAR(LegHeightKinematics::move_towards(0.090, 0.070, 0.01, 0.001),
                0.08999, kTolerance);
    EXPECT_NEAR(LegHeightKinematics::move_towards(0.070, 0.070005, 0.01, 0.001),
                0.070005, kTolerance);
}

}  // namespace
