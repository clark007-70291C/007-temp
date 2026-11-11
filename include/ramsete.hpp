#pragma once

#include "lemlib/chassis/chassis.hpp"
#include "pros/motor_group.hpp"

#include <cstddef>
#include <initializer_list>

namespace ramsete {

using Action = void (*)();

// Attach a no-capture function or lambda to a waypoint. It runs once when
// odometry comes within Config::waypointToleranceInches of that waypoint.
Action execute(Action action);

// One time-stamped point in a trajectory. Positions are inches, heading is
// degrees, linear velocity is inches per second, and angular velocity is
// radians per second. Use LemLib standard coordinates for the pose.
struct TrajectorySample {
    double timeSeconds;
    double x;
    double y;
    double headingDegrees;
    double linearVelocity;
    double angularVelocity;
    Action action = nullptr;
};

struct Config {
    double b = 2.0;
    double zeta = 0.7;
    double trackWidthInches = 12.0;
    double wheelDiameterInches = 2.75;
    double maximumMotorRpm = 450.0;
    double waypointToleranceInches = 2.0;
    double finalPositionToleranceInches = 1.0;
    double finalHeadingToleranceDegrees = 3.0;
    double finalSettleTimeoutSeconds = 1.5;
    double finalLinearGain = 2.0;
    double finalHeadingGain = 3.0;
    double finalLateralGain = 2.0;
    double finalMaximumLinearVelocity = 12.0;
    double finalMaximumAngularVelocity = 2.0;
};

// Follow a time-parameterized trajectory using LemLib's odometry pose.
// Returns false if settings are invalid or the robot does not reach the
// trajectory endpoint within the configured settle timeout.
bool follow(lemlib::Chassis& chassis, pros::MotorGroup& leftMotors,
            pros::MotorGroup& rightMotors, const TrajectorySample* samples,
            std::size_t sampleCount, const Config& config = {});

// Define and run a trajectory using the robot and tuning configured in main.cpp.
// Example: ramsete::execute({{0, 0, 0, 0, 0, 0}, {1, 12, 0, 0, 0, 0}});
bool execute(std::initializer_list<TrajectorySample> samples);

} // namespace ramsete
