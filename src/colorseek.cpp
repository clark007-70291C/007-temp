#include "colorseek.hpp"

#include "pros/rtos.hpp"

#include <cmath>
#include <cstdint>

namespace colorseek {
namespace {

constexpr std::int32_t searchTimeoutMs = 5000;
constexpr std::int32_t loopDelayMs = 10;
constexpr std::int32_t minimumProximity = 100;
constexpr double minimumSaturation = 0.25;

pros::AbstractMotor* searchMotor = nullptr;
pros::Optical* colorSensor = nullptr;

bool matchesColor(Color color, double hue) {
    switch (color) {
        case BLUE:
            return hue >= 190 && hue <= 260;
        case RED:
            return hue <= 30 || hue >= 330;
        case YELLOW:
            return hue >= 40 && hue <= 80;
    }

    return false;
}

} // namespace

void initialize(pros::AbstractMotor& motor, pros::Optical& sensor) {
    searchMotor = &motor;
    colorSensor = &sensor;
    colorSensor->set_led_pwm(100);
}

bool spinUntil(Color color) {
    if (searchMotor == nullptr || colorSensor == nullptr) {
        return false;
    }

    searchMotor->move(127);
    for (std::int32_t elapsedMs = 0; elapsedMs < searchTimeoutMs;
         elapsedMs += loopDelayMs) {
        const double hue = colorSensor->get_hue();
        const double saturation = colorSensor->get_saturation();
        const std::int32_t proximity = colorSensor->get_proximity();
        if (std::isfinite(hue) && std::isfinite(saturation) &&
            saturation >= minimumSaturation && proximity >= minimumProximity &&
            matchesColor(color, hue)) {
            searchMotor->move(0);
            return true;
        }

        pros::delay(loopDelayMs);
    }

    searchMotor->move(0);
    return false;
}

} // namespace colorseek
