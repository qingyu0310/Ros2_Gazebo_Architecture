#include <limits>

#include <gtest/gtest.h>

#include "pitch_lqr.hpp"
#include "roll_lqr.hpp"

TEST(PitchLqrControllerTest, ComputesFeedforwardMinusStateFeedback)
{
    PitchLqrController::Parameters parameters;
    parameters.gain(0, 0) = 2.0;
    parameters.gain(1, 2) = -3.0;
    parameters.gain(2, 7) = 4.0;
    parameters.x0(0) = 1.0;

    PitchLqrController controller;
    controller.configure(parameters);

    PitchLqrController::State state = PitchLqrController::State::Zero();
    state(0) = 1.5;
    state(2) = 0.2;
    state(7) = -0.1;
    PitchLqrController::State target = parameters.x0;
    PitchLqrController::Control feedforward;
    feedforward << 0.1, 0.2, 0.3;

    const auto control = controller.update(state, target, feedforward);

    EXPECT_NEAR(control(0), -0.9, 1e-12);
    EXPECT_NEAR(control(1), 0.8, 1e-12);
    EXPECT_NEAR(control(2), 0.7, 1e-12);
}

TEST(PitchLqrControllerTest, InterpolatesWholeWorkingPoint)
{
    PitchLqrController::Parameters lower;
    PitchLqrController::Parameters upper;
    lower.gain.setConstant(2.0);
    lower.x0.setConstant(4.0);
    lower.u0.setConstant(6.0);
    upper.gain.setConstant(10.0);
    upper.x0.setConstant(12.0);
    upper.u0.setConstant(14.0);

    const auto result = PitchLqrController::interpolate(lower, upper, 0.25);

    EXPECT_TRUE(result.gain.isConstant(4.0));
    EXPECT_TRUE(result.x0.isConstant(6.0));
    EXPECT_TRUE(result.u0.isConstant(8.0));
}

TEST(RollLqrControllerTest, ComputesDifferenceEffortAndGeometryPrediction)
{
    RollLqrController::Parameters parameters;
    parameters.gain(0, 0) = 2.0;
    parameters.gain(1, 2) = -4.0;
    parameters.x0(0) = 0.1;
    parameters.u0 << 0.05, -0.03;
    parameters.roll_per_q << 0.5, -1.95;

    RollLqrController controller;
    controller.configure(parameters);

    RollLqrController::State state = RollLqrController::State::Zero();
    state(0) = 0.2;
    state(2) = -0.1;
    const auto control = controller.update(state);

    EXPECT_NEAR(control(0), -0.15, 1e-12);
    EXPECT_NEAR(control(1), -0.43, 1e-12);
    EXPECT_NEAR(controller.predict_roll(0.2, -0.1), 0.295, 1e-12);
    EXPECT_NEAR(controller.predict_roll_rate(0.4, 0.2), -0.19, 1e-12);
}

TEST(RuntimeLqrControllerTest, RejectsInvalidParametersAndUseBeforeConfigure)
{
    PitchLqrController pitch;
    EXPECT_THROW(pitch.parameters(), std::logic_error);

    PitchLqrController::Parameters invalid_pitch;
    invalid_pitch.gain(0, 0) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(pitch.configure(invalid_pitch), std::invalid_argument);

    RollLqrController roll;
    EXPECT_THROW(roll.predict_roll(0.0, 0.0), std::logic_error);

    RollLqrController::Parameters invalid_roll;
    invalid_roll.roll_per_q(0) = std::numeric_limits<double>::infinity();
    EXPECT_THROW(roll.configure(invalid_roll), std::invalid_argument);
}
