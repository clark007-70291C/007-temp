#include "dsr.hpp"

#include "lemlib/chassis/chassis.hpp"
#include "pros/distance.hpp"

#include <cstdint>

extern lemlib::Chassis chassis;

namespace dsr {
namespace {

constexpr float millimetersPerInch = 25.4f;
constexpr std::int32_t invalidDistance = 9999;

// Set an unused sensor's port to 0; each configured sensor needs a unique port.
#define DSR_EAST_PORT 2
#define DSR_WEST_PORT 0
#define DSR_NORTH_PORT 0
#define DSR_SOUTH_PORT 3

#if DSR_EAST_PORT > 0
pros::Distance eastDistanceSensor(DSR_EAST_PORT);
pros::Distance* const eastSensor = &eastDistanceSensor;
#else
pros::Distance* const eastSensor = nullptr;
#endif

#if DSR_WEST_PORT > 0
pros::Distance westDistanceSensor(DSR_WEST_PORT);
pros::Distance* const westSensor = &westDistanceSensor;
#else
pros::Distance* const westSensor = nullptr;
#endif

#if DSR_NORTH_PORT > 0
pros::Distance northDistanceSensor(DSR_NORTH_PORT);
pros::Distance* const northSensor = &northDistanceSensor;
#else
pros::Distance* const northSensor = nullptr;
#endif

#if DSR_SOUTH_PORT > 0
pros::Distance southDistanceSensor(DSR_SOUTH_PORT);
pros::Distance* const southSensor = &southDistanceSensor;
#else
pros::Distance* const southSensor = nullptr;
#endif

struct ResetConfig {
    float eastWallX;
    float westWallX;
    float northWallY;
    float southWallY;
    float eastSensorOffsetX;
    float westSensorOffsetX;
    float northSensorOffsetY;
    float southSensorOffsetY;
};

// Coordinates and offsets must use the same origin as chassis pose. These defaults
// meassure the walls that you will reset off of based on the robot starting position 
// if you start touching the south wall then meassure the offset from the center of the robot 
constexpr ResetConfig resetConfig{
    72.0f,  // east wall X
    0.0f,   // west wall X (unused)
    0.0f,   // north wall Y (unused)
    -72.0f, // south wall Y
    0.0f,   // east sensor offset from the robot center
    0.0f,   // west sensor offset from the robot center
    0.0f,   // north sensor offset from the robot center
    0.0f    // south sensor offset from the robot center
};

bool readDistanceInches(pros::Distance* sensor, float& distanceInches) {
    if (sensor == nullptr) {
        return false;
    }

    const std::int32_t distanceMm = sensor->get();
    if (distanceMm <= 0 || distanceMm >= invalidDistance) {
        return false;
    }

    distanceInches = distanceMm / millimetersPerInch;
    return true;
}

void addPositionReading(pros::Distance* sensor, float wallCoordinate,
                        float sensorOffset, float distanceSign, float& total,
                        unsigned int& count) {
    float distanceInches = 0.0f;
    if (readDistanceInches(sensor, distanceInches)) {
        total += wallCoordinate + distanceSign * distanceInches - sensorOffset;
        ++count;
    }
}

} // namespace

bool resetPosition() {
    lemlib::Pose pose = chassis.getPose();
    float xTotal = 0.0f;
    float yTotal = 0.0f;
    unsigned int xReadings = 0;
    unsigned int yReadings = 0;

    addPositionReading(eastSensor, resetConfig.eastWallX,
                       resetConfig.eastSensorOffsetX, -1.0f, xTotal, xReadings);
    addPositionReading(westSensor, resetConfig.westWallX,
                       resetConfig.westSensorOffsetX, 1.0f, xTotal, xReadings);
    addPositionReading(northSensor, resetConfig.northWallY,
                       resetConfig.northSensorOffsetY, -1.0f, yTotal, yReadings);
    addPositionReading(southSensor, resetConfig.southWallY,
                       resetConfig.southSensorOffsetY, 1.0f, yTotal, yReadings);

    if (xReadings == 0 && yReadings == 0) {
        return false;
    }
    if (xReadings > 0) {
        pose.x = xTotal / xReadings;
    }
    if (yReadings > 0) {
        pose.y = yTotal / yReadings;
    }

    chassis.setPose(pose);
    return true;
}

} // namespace dsr