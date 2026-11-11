#include "ramsete.hpp"

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

extern lemlib::Chassis chassis;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern ramsete::Config skillsRamseteConfig;

namespace ramsete {
namespace {

constexpr double pi = 3.14159265358979323846;
constexpr double loopPeriodSeconds = 0.01;
constexpr double degreesToRadians = pi / 180.0;

bool validSample(const TrajectorySample& sample) {
    return std::isfinite(sample.timeSeconds) &&
           std::isfinite(sample.x) &&
           std::isfinite(sample.y) &&
           std::isfinite(sample.headingDegrees) &&
           std::isfinite(sample.linearVelocity) &&
           std::isfinite(sample.angularVelocity);
}

bool validConfig(const Config& config) {
    return std::isfinite(config.b) && config.b > 0 &&
           std::isfinite(config.zeta) && config.zeta > 0 &&
           std::isfinite(config.trackWidthInches) && config.trackWidthInches > 0 &&
           std::isfinite(config.wheelDiameterInches) && config.wheelDiameterInches > 0 &&
           std::isfinite(config.maximumMotorRpm) && config.maximumMotorRpm > 0 &&
           std::isfinite(config.waypointToleranceInches) &&
           config.waypointToleranceInches > 0 &&
           std::isfinite(config.finalPositionToleranceInches) &&
           config.finalPositionToleranceInches > 0 &&
           std::isfinite(config.finalHeadingToleranceDegrees) &&
           config.finalHeadingToleranceDegrees > 0 &&
           std::isfinite(config.finalSettleTimeoutSeconds) &&
           config.finalSettleTimeoutSeconds > 0 &&
           std::isfinite(config.finalLinearGain) && config.finalLinearGain > 0 &&
           std::isfinite(config.finalHeadingGain) && config.finalHeadingGain > 0 &&
           std::isfinite(config.finalLateralGain) && config.finalLateralGain >= 0 &&
           std::isfinite(config.finalMaximumLinearVelocity) &&
           config.finalMaximumLinearVelocity > 0 &&
           std::isfinite(config.finalMaximumAngularVelocity) &&
           config.finalMaximumAngularVelocity > 0;
}

void runReachedActions(const TrajectorySample* samples, std::size_t sampleCount,
                       std::size_t& nextAction, const lemlib::Pose& pose,
                       const Config& config) {
    while (nextAction < sampleCount) {
        if (samples[nextAction].action == nullptr) {
            ++nextAction;
            continue;
        }

        double dx = samples[nextAction].x - pose.x;
        double dy = samples[nextAction].y - pose.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance > config.waypointToleranceInches) {
            break;
        }

        samples[nextAction].action();
        ++nextAction;
    }
}

double wrapRadians(double angle) {
    return std::remainder(angle, 2.0 * pi);
}

TrajectorySample interpolate(const TrajectorySample& start,
                             const TrajectorySample& end, double timeSeconds) {
    double fraction =
        (timeSeconds - start.timeSeconds) / (end.timeSeconds - start.timeSeconds);
    return {
        timeSeconds,
        start.x + (end.x - start.x) * fraction,
        start.y + (end.y - start.y) * fraction,
        start.headingDegrees +
            std::remainder(end.headingDegrees - start.headingDegrees, 360.0) * fraction,
        start.linearVelocity +
            (end.linearVelocity - start.linearVelocity) * fraction,
        start.angularVelocity +
            (end.angularVelocity - start.angularVelocity) * fraction
    };
}

void commandWheels(pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors,
                   double linearVelocity, double angularVelocity,
                   const Config& config) {
    double leftVelocity =
        linearVelocity - angularVelocity * config.trackWidthInches / 2.0;
    double rightVelocity =
        linearVelocity + angularVelocity * config.trackWidthInches / 2.0;

    double maximumWheelVelocity =
        config.maximumMotorRpm * pi * config.wheelDiameterInches / 60.0;
    double largestVelocity =
        std::max(std::abs(leftVelocity), std::abs(rightVelocity));
    if (largestVelocity > maximumWheelVelocity) {
        double scale = maximumWheelVelocity / largestVelocity;
        leftVelocity *= scale;
        rightVelocity *= scale;
    }

    double inchesPerMinutePerRpm = pi * config.wheelDiameterInches;
    auto leftRpm = static_cast<std::int32_t>(
        std::lround(leftVelocity * 60.0 / inchesPerMinutePerRpm));
    auto rightRpm = static_cast<std::int32_t>(
        std::lround(rightVelocity * 60.0 / inchesPerMinutePerRpm));

    leftMotors.move_velocity(leftRpm);
    rightMotors.move_velocity(rightRpm);
}

} // namespace

Action execute(Action action) {
    return action;
}

bool follow(lemlib::Chassis& chassis, pros::MotorGroup& leftMotors,
            pros::MotorGroup& rightMotors, const TrajectorySample* samples,
            std::size_t sampleCount, const Config& config) {
    if (samples == nullptr || sampleCount < 2 || !validConfig(config)) {
        return false;
    }

    if (samples[0].timeSeconds != 0.0) {
        return false;
    }

    for (std::size_t i = 0; i < sampleCount; ++i) {
        if (!validSample(samples[i]) || samples[i].timeSeconds < 0 ||
            (i > 0 && samples[i].timeSeconds <= samples[i - 1].timeSeconds)) {
            return false;
        }
    }

    chassis.cancelAllMotions();
    TrajectorySample finalSample = samples[sampleCount - 1];
    double duration = finalSample.timeSeconds;
    std::uint32_t startTime = pros::millis();
    std::size_t nextAction = 0;

    while (true) {
        double elapsed =
            static_cast<std::uint32_t>(pros::millis() - startTime) / 1000.0;

        lemlib::Pose pose = chassis.getPose(true, true);
        runReachedActions(samples, sampleCount, nextAction, pose, config);

        // Express target position error in the robot's coordinate frame.
        double heading = pose.theta;
        bool settling = elapsed >= duration;
        TrajectorySample target = settling
            ? finalSample
            : [&] {
                std::size_t nextSample = 1;
                while (nextSample < sampleCount - 1 &&
                       samples[nextSample].timeSeconds < elapsed) {
                    ++nextSample;
                }
                return interpolate(samples[nextSample - 1], samples[nextSample], elapsed);
            }();
        double dx = target.x - pose.x;
        double dy = target.y - pose.y;
        double errorX = std::cos(heading) * dx + std::sin(heading) * dy;
        double errorY = -std::sin(heading) * dx + std::cos(heading) * dy;
        double targetHeading = target.headingDegrees * degreesToRadians;
        double errorHeading = wrapRadians(targetHeading - heading);

        double correctedVelocity = 0;
        double correctedAngularVelocity = 0;
        if (settling) {
            double distanceToEnd = std::sqrt(dx * dx + dy * dy);
            if (distanceToEnd <= config.finalPositionToleranceInches &&
                std::abs(errorHeading) <=
                    config.finalHeadingToleranceDegrees * degreesToRadians) {
                leftMotors.move(0);
                rightMotors.move(0);
                return true;
            }

            if (elapsed >= duration + config.finalSettleTimeoutSeconds) {
                leftMotors.move(0);
                rightMotors.move(0);
                return false;
            }

            correctedVelocity = std::clamp(
                config.finalLinearGain * errorX,
                -config.finalMaximumLinearVelocity,
                config.finalMaximumLinearVelocity);
            correctedAngularVelocity = std::clamp(
                config.finalHeadingGain * errorHeading +
                    config.finalLateralGain *
                        std::atan2(errorY, std::max(std::abs(errorX), 1.0)),
                -config.finalMaximumAngularVelocity,
                config.finalMaximumAngularVelocity);
        } else {
            double velocity = target.linearVelocity;
            double angularVelocity = target.angularVelocity;
            double gain = 2.0 * config.zeta *
                std::sqrt(angularVelocity * angularVelocity +
                          config.b * velocity * velocity);
            double sinc = std::abs(errorHeading) < 1e-6
                ? 1.0 - errorHeading * errorHeading / 6.0
                : std::sin(errorHeading) / errorHeading;

            correctedVelocity =
                velocity * std::cos(errorHeading) + gain * errorX;
            correctedAngularVelocity =
                angularVelocity + gain * errorHeading +
                config.b * velocity * sinc * errorY;
        }

        commandWheels(leftMotors, rightMotors, correctedVelocity,
                      correctedAngularVelocity, config);
        pros::delay(static_cast<std::uint32_t>(loopPeriodSeconds * 1000.0));
    }
}

bool execute(std::initializer_list<TrajectorySample> samples) {
    return follow(chassis, leftMotors, rightMotors, samples.begin(), samples.size(),
                  skillsRamseteConfig);
}

} // namespace ramsete
