#pragma once

#include <cstdint>
#include <initializer_list>

namespace purepursuit {

using Action = void (*)();

// Attach an action to a waypoint. It runs once when the robot reaches it.
Action execute(Action action);

struct Point {
    double x;
    double y;
    Action action = nullptr;
};

struct Config {
    double lookaheadInches = 8.0;
    double maximumLinearVelocity = 20.0;
    double trackWidthInches = 12.0;
    double wheelDiameterInches = 2.75;
    double maximumMotorRpm = 450.0;
    double waypointToleranceInches = 1.5;
    double endpointToleranceInches = 1.0;
    std::uint32_t timeoutMilliseconds = 5000;
};

// Define and follow a path using the robot and tuning configured in main.cpp.
// Example: purepursuit::execute({{0, 0}, {12, 0}, {24, 0}});
bool execute(std::initializer_list<Point> path);

} // namespace purepursuit
