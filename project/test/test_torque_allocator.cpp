#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "torque_allocator.hpp"

namespace
{
TorqueAllocator::Config test_config()
{
    TorqueAllocator::Config config;
    config.effort_limit.fill(10.0);
    config.effort_rate_limit.fill(1.0e9);
    return config;
}

TorqueAllocator::Feedback zero_feedback()
{
    return TorqueAllocator::Feedback{};
}
}

TEST(TorqueAllocatorTest, CommonDifferenceRecoveryIsExact)
{
    TorqueAllocator allocator(test_config());
    TorqueAllocator::Input input;
    input.wheel_common = 0.4;
    input.wheel_diff = -0.1;
    input.hip_common = -0.2;
    input.hip_diff = 0.3;
    input.knee_common = -0.5;
    input.knee_diff = -0.4;

    const auto result = allocator.allocate(input, zero_feedback());
    EXPECT_DOUBLE_EQ(result.effort[TorqueAllocator::kWheelLeft], 0.3);
    EXPECT_DOUBLE_EQ(result.effort[TorqueAllocator::kWheelRight], 0.5);
    EXPECT_DOUBLE_EQ(result.effort[TorqueAllocator::kHipLeft], 0.1);
    EXPECT_DOUBLE_EQ(result.effort[TorqueAllocator::kHipRight], -0.5);
    EXPECT_DOUBLE_EQ(result.effort[TorqueAllocator::kKneeLeft], -0.9);
    EXPECT_DOUBLE_EQ(result.effort[TorqueAllocator::kKneeRight], -0.1);
}

TEST(TorqueAllocatorTest, LeftRightMirrorOnlyFlipsDifference)
{
    TorqueAllocator allocator_a(test_config());
    TorqueAllocator allocator_b(test_config());
    TorqueAllocator::Input input;
    input.hip_common = 0.25;
    input.hip_diff = 0.6;

    const auto original = allocator_a.allocate(input, zero_feedback());
    input.hip_diff = -input.hip_diff;
    const auto mirrored = allocator_b.allocate(input, zero_feedback());

    EXPECT_DOUBLE_EQ(original.effort[TorqueAllocator::kHipLeft], mirrored.effort[TorqueAllocator::kHipRight]);
    EXPECT_DOUBLE_EQ(original.effort[TorqueAllocator::kHipRight], mirrored.effort[TorqueAllocator::kHipLeft]);
}

TEST(TorqueAllocatorTest, CommonModeHasPriorityOverOversizedDifference)
{
    auto config = test_config();
    config.effort_limit[TorqueAllocator::kKneeLeft] = 1.5;
    config.effort_limit[TorqueAllocator::kKneeRight] = 1.5;
    TorqueAllocator allocator(config);
    TorqueAllocator::Input input;
    input.knee_common = 1.0;
    input.knee_diff = 20.0;

    const auto result = allocator.allocate(input, zero_feedback());
    EXPECT_DOUBLE_EQ(result.effort[TorqueAllocator::kKneeLeft], 1.5);
    EXPECT_DOUBLE_EQ(result.effort[TorqueAllocator::kKneeRight], 0.5);
    EXPECT_TRUE(result.diagnostics.priority_limited);
}

TEST(TorqueAllocatorTest, SoftLimitAddsRestoringEffort)
{
    TorqueAllocator allocator(test_config());
    auto feedback = zero_feedback();
    feedback.position[0] = -0.20;

    const auto result = allocator.allocate(TorqueAllocator::Input{}, feedback);
    EXPECT_GT(result.effort[TorqueAllocator::kHipLeft], 0.0);
    EXPECT_DOUBLE_EQ(result.effort[TorqueAllocator::kHipRight], 0.0);
    EXPECT_TRUE(result.diagnostics.soft_limit_active);
}

TEST(TorqueAllocatorTest, PairRateLimitScalesTheWholeChangeVector)
{
    auto config = test_config();
    config.dt = 0.1;
    config.effort_rate_limit[TorqueAllocator::kHipLeft] = 1.0;
    config.effort_rate_limit[TorqueAllocator::kHipRight] = 1.0;
    TorqueAllocator allocator(config);
    TorqueAllocator::Input input;
    input.hip_common = 1.5;
    input.hip_diff = 0.5;  // 目标 [2, 1]，按同一比例缩到 [0.1, 0.05]

    const auto result = allocator.allocate(input, zero_feedback());
    EXPECT_NEAR(result.effort[TorqueAllocator::kHipLeft], 0.1, 1e-12);
    EXPECT_NEAR(result.effort[TorqueAllocator::kHipRight], 0.05, 1e-12);
    EXPECT_TRUE(result.diagnostics.rate_limited);
}

TEST(TorqueAllocatorTest, NonFiniteInputProducesImmediateZero)
{
    TorqueAllocator allocator(test_config());
    TorqueAllocator::Input input;
    input.knee_diff = std::numeric_limits<double>::quiet_NaN();

    const auto result = allocator.allocate(input, zero_feedback());
    EXPECT_TRUE(result.diagnostics.invalid_input);
    for (const double effort : result.effort)
    {
        EXPECT_DOUBLE_EQ(effort, 0.0);
    }
}

TEST(TorqueAllocatorTest, ArbitraryCommonDifferenceCombinationsNeverExceedLimits)
{
    auto config = test_config();
    config.effort_limit = {0.28, 0.28, 1.5, 1.2, 1.1, 1.5};
    TorqueAllocator allocator(config);

    for (double common = -20.0; common <= 20.0; common += 0.5)
    {
        for (double difference = -20.0; difference <= 20.0; difference += 0.5)
        {
            TorqueAllocator::Input input;
            input.wheel_common = common;
            input.wheel_diff = difference;
            input.hip_common = common;
            input.hip_diff = difference;
            input.knee_common = common;
            input.knee_diff = difference;
            const auto result = allocator.allocate(input, zero_feedback());
            for (std::size_t i = 0; i < result.effort.size(); ++i)
            {
                EXPECT_LE(std::abs(result.effort[i]), config.effort_limit[i] + 1e-12);
            }
        }
    }
}
