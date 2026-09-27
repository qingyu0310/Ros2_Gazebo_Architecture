#include <chrono>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "lqr_gain_schedule.hpp"
#include "leg_gravity_compensation.hpp"
#include "safety_supervisor.hpp"

namespace
{
SafetySupervisor make_supervisor(double recovery_dwell_s = 0.2)
{
    SafetyParams safety;
    safety.feedback_timeout_s = 0.1;
    safety.max_control_period_s = 0.02;
    safety.recovery_dwell_s = recovery_dwell_s;
    return SafetySupervisor(safety, 0.7,
                            {-0.2, -0.2, -1.0, -1.0},
                            {0.5, 0.5, 0.3, 0.3});
}

SafetySupervisor::Input healthy_input()
{
    SafetySupervisor::Input input;
    input.has_imu = true;
    input.has_joint_state = true;
    input.imu_age_s = 0.001;
    input.joint_state_age_s = 0.001;
    input.control_period_s = 0.001;
    return input;
}
}

TEST(LqrGainScheduleTest, SortsAndInterpolatesPitchAndRollParameters)
{
    LqrGainSchedule::Point high;
    high.height = 0.10;
    high.pitch.gain.setConstant(10.0);
    high.pitch.x0.setConstant(20.0);
    high.pitch.u0.setConstant(30.0);
    high.roll.gain.setConstant(40.0);
    high.roll.roll_per_q.setConstant(50.0);

    LqrGainSchedule::Point low;
    low.height = 0.06;
    low.pitch.gain.setConstant(2.0);
    low.pitch.x0.setConstant(4.0);
    low.pitch.u0.setConstant(6.0);
    low.roll.gain.setConstant(8.0);
    low.roll.roll_per_q.setConstant(10.0);

    LqrGainSchedule schedule(std::vector<LqrGainSchedule::Point>{high, low});
    const auto middle = schedule.at(0.08);

    EXPECT_TRUE(middle.pitch.gain.isConstant(6.0));
    EXPECT_TRUE(middle.pitch.x0.isConstant(12.0));
    EXPECT_TRUE(middle.pitch.u0.isConstant(18.0));
    EXPECT_TRUE(middle.roll.gain.isConstant(24.0));
    EXPECT_TRUE(middle.roll.roll_per_q.isConstant(30.0));
    EXPECT_TRUE(schedule.at(0.01).pitch.gain.isConstant(2.0));
    EXPECT_TRUE(schedule.at(0.20).pitch.gain.isConstant(10.0));
}

TEST(LegGravityCompensationTest, NominalPoseMatchesGeneratedOperatingPoint)
{
    LegGravityCompensation compensation(ModelParams{}, HeightParams{});
    const auto [hip, knee] = compensation.effort(0.0, 0.0);

    EXPECT_NEAR(hip, -0.0147292, 1e-6);
    EXPECT_NEAR(knee, -0.256709, 1e-6);
}

TEST(SafetySupervisorTest, FirstHealthyFeedbackActivatesImmediately)
{
    auto supervisor = make_supervisor();
    const auto now = SafetySupervisor::Clock::now();
    const auto decision = supervisor.update(healthy_input(), now);

    EXPECT_TRUE(decision.allow_control);
    EXPECT_TRUE(decision.reset_control_state);
    EXPECT_EQ(decision.event, SafetySupervisor::Event::kFirstActivation);
}

TEST(SafetySupervisorTest, TimeoutRequiresHealthyDwellBeforeRecovery)
{
    auto supervisor = make_supervisor(0.2);
    const auto start = SafetySupervisor::Clock::now();
    supervisor.update(healthy_input(), start);

    auto timeout = healthy_input();
    timeout.imu_age_s = 0.11;
    auto decision = supervisor.update(timeout, start + std::chrono::milliseconds(1));
    EXPECT_FALSE(decision.allow_control);
    EXPECT_EQ(decision.state, SafetySupervisor::State::kFeedbackTimeout);

    decision = supervisor.update(healthy_input(), start + std::chrono::milliseconds(2));
    EXPECT_FALSE(decision.allow_control);
    EXPECT_EQ(decision.event, SafetySupervisor::Event::kRecoveryStarted);

    decision = supervisor.update(healthy_input(), start + std::chrono::milliseconds(201));
    EXPECT_FALSE(decision.allow_control);
    decision = supervisor.update(healthy_input(), start + std::chrono::milliseconds(203));
    EXPECT_TRUE(decision.allow_control);
    EXPECT_TRUE(decision.reset_control_state);
    EXPECT_EQ(decision.event, SafetySupervisor::Event::kRecovered);
}

TEST(SafetySupervisorTest, RejectsFallJointLimitAndInvalidState)
{
    const auto now = SafetySupervisor::Clock::now();

    auto fallen_supervisor = make_supervisor();
    auto fallen = healthy_input();
    fallen.state.roll = 0.71;
    EXPECT_EQ(fallen_supervisor.update(fallen, now).state, SafetySupervisor::State::kFallen);

    auto joint_supervisor = make_supervisor();
    auto joint = healthy_input();
    joint.state.leg_position_rad[2] = -1.01;
    EXPECT_EQ(joint_supervisor.update(joint, now).state, SafetySupervisor::State::kJointLimit);

    auto invalid_supervisor = make_supervisor();
    auto invalid = healthy_input();
    invalid.state.pitch = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(invalid_supervisor.update(invalid, now).state, SafetySupervisor::State::kInvalidState);
}
