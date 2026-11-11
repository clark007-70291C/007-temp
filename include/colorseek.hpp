#pragma once

#include "pros/optical.hpp"
#include "pros/abstract_motor.hpp"

namespace colorseek {

enum Color {
    BLUE,
    RED,
    YELLOW
};

// Connect this helper to the motor and optical sensor once in initialize().
void initialize(pros::AbstractMotor& motor, pros::Optical& sensor);

// Spin the connected motor until the requested color is seen.
// Returns false if the color is not detected within 5 seconds.
// Example: colorseek::spinUntil(colorseek::BLUE);
bool spinUntil(Color color);

} // namespace colorseek
