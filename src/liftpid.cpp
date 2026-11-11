#include "liftpid.hpp"

#include "lemlib/pid.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>

namespace liftpid {
namespace {
//put the motors that u want to use the lift pid in here and the rotation sensor 
pros::MotorGroup motors({14, -17}, pros::MotorGearset::blue);
pros::Rotation rotation(-13);
lemlib::PID pid(1, 0, 0.1);

std::atomic<double> targetCentiDegrees{0};
std::atomic<double> commandedPower{0};
std::atomic<bool> pidEnabled{false};
std::atomic<bool> resetPid{false};
std::atomic<bool> targetActive{false};
bool isCoasting = false;

constexpr double inchesPerMotorRevolution = 3.0 * 3.14159265358979323846;
constexpr double bottomLimitInches = 0.0;
constexpr double coastThresholdInches = 5.0;

double inchesToCentiDegrees(double inches) {
    return inches * 36000.0 / inchesPerMotorRevolution;
}

double centiDegreesToInches(double centiDegrees) {
    return centiDegrees * inchesPerMotorRevolution / 36000.0;
}

void updatePositionControl() {
    if (resetPid.exchange(false)) {
        pid.reset();
    }

    const double position = rotation.get_position();
    const double error = (targetCentiDegrees.load() - position) / 100.0;
    const double requestedPower = std::abs(error) < 0.5 ? 0 : pid.update(error);
    const double currentPower = commandedPower.load();
    constexpr double maxPowerChangePerCycle = 5;
    const double change = std::clamp(
        requestedPower - currentPower, -maxPowerChangePerCycle, maxPowerChangePerCycle);
    const double appliedPower = currentPower + change;

    motors.move(appliedPower);
    commandedPower.store(appliedPower);
}

} // namespace

void initialize() {
    motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    static pros::Task controllerTask([] {
        while (true) {
            if (pidEnabled.load()) {
                updatePositionControl();
            }
            pros::delay(10);
        }
    });
}

bool moveTo(double heightInches) {
    if (!std::isfinite(heightInches) || heightInches < 0) {
        return false;
    }

    targetCentiDegrees.store(inchesToCentiDegrees(heightInches));
    resetPid.store(true);
    targetActive.store(true);
    pidEnabled.store(true);
    return true;
}

bool moveTo(Position preset) {
    switch (preset) {
        case BASE:   return moveTo(0.0);
        case FIRST:  return moveTo(5.5);
        case SECOND: return moveTo(9.0);
        case THIRD:  return moveTo(13.5);
        case FOURTH: return moveTo(6.28);
        case FIFTH:  return moveTo(6.86);
        case SIXTH:  return moveTo(7.33);
    }

    return false;
}

void manualControl(double power, bool overrideBottomLimit) {
    double limitedPower = std::clamp(power, -127.0, 127.0);
    if (limitedPower == 0 && targetActive.load()) {
        return;
    }

    if (limitedPower != 0) {
        targetActive.store(false);
        pidEnabled.store(false);
    }

    const double position = rotation.get_position();
    const double heightInches = centiDegreesToInches(position);
    const bool atBottomLimit = heightInches <= bottomLimitInches;
    const bool shouldCoast =
        std::abs(heightInches) < coastThresholdInches && !atBottomLimit;
    if (shouldCoast != isCoasting) {
        motors.set_brake_mode_all(
            shouldCoast ? pros::E_MOTOR_BRAKE_COAST : pros::E_MOTOR_BRAKE_HOLD);
        isCoasting = shouldCoast;
    }

    if (atBottomLimit && limitedPower < 0 && !overrideBottomLimit) {
        limitedPower = 0;
    }

    motors.move(limitedPower);
    commandedPower.store(limitedPower);
}

void manualControl(double power) {
    manualControl(power, false);
}

void manualControl(bool raiseButton, bool lowerButton) {
    const double power = raiseButton == lowerButton ? 0 : (raiseButton ? 127 : -127);
    manualControl(power, false);
}

void manualControl(bool raiseButton, bool lowerButton, bool overrideBottomLimit) {
    const double power = raiseButton == lowerButton ? 0 : (raiseButton ? 127 : -127);
    manualControl(power, overrideBottomLimit);
}

void stop() {
    pidEnabled.store(false);
    targetActive.store(false);
    motors.move(0);
    commandedPower.store(0);
}

void tarePosition() {
    pidEnabled.store(false);
    targetActive.store(false);
    rotation.set_position(0);
    targetCentiDegrees.store(0);
    motors.move(0);
    commandedPower.store(0);
    resetPid.store(true);
}

double getPositionInches() {
    return centiDegreesToInches(rotation.get_position());
}

} // namespace liftpid
