#include "dsr.hpp"

#include "lemlib/chassis/chassis.hpp"
#include "pros/distance.hpp"

#include <cstdint>

extern lemlib::Chassis chassis;

namespace dsr {
namespace {

constexpr float millimetersPerInch = 25.4f;
constexpr std::int32_t invalidDistance = 9999;

// Distance sensors: change the port numbers to match your robot.
pros::Distance eastDistanceSensor(2);
pros::Distance southDistanceSensor(3);

// Set a direction to nullptr if you do not have a sensor facing that wall.
pros::Distance* northSensor = nullptr;
pros::Distance* eastSensor = &eastDistanceSensor;
pros::Distance* southSensor = &southDistanceSensor;
pros::Distance* westSensor = nullptr;

struct ResetConfig {
    float northWallY;
    float eastWallX;
    float southWallY;
    float westWallX;
    float northSensorOffsetY;
    float eastSensorOffsetX;
    float southSensorOffsetY;
    float westSensorOffsetX;
};

// Enter wall coordinates and sensor offsets in inches.
const ResetConfig resetConfig{
    0.0f, // north wall Y
    0.0f, // east wall X
    0.0f, // south wall Y
    0.0f, // west wall X
    0.0f, // north sensor offset Y
    0.0f, // east sensor offset X
    0.0f, // south sensor offset Y
    0.0f  // west sensor offset X
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

} // namespace

bool resetWall() {
    float correctedX = 0;
    float correctedY = 0;
    unsigned int xReadings = 0;
    unsigned int yReadings = 0;
    float distanceInches = 0;

    if (readDistanceInches(eastSensor, distanceInches)) {
        correctedX += resetConfig.eastWallX - distanceInches -
            resetConfig.eastSensorOffsetX;
        ++xReadings;
    }
    if (readDistanceInches(westSensor, distanceInches)) {
        correctedX += resetConfig.westWallX + distanceInches -
            resetConfig.westSensorOffsetX;
        ++xReadings;
    }
    if (readDistanceInches(northSensor, distanceInches)) {
        correctedY += resetConfig.northWallY - distanceInches -
            resetConfig.northSensorOffsetY;
        ++yReadings;
    }
    if (readDistanceInches(southSensor, distanceInches)) {
        correctedY += resetConfig.southWallY + distanceInches -
            resetConfig.southSensorOffsetY;
        ++yReadings;
    }

    if (xReadings == 0 && yReadings == 0) {
        return false;
    }

    lemlib::Pose currentPose = chassis.getPose();
    if (xReadings > 0) {
        currentPose.x = correctedX / xReadings;
    }
    if (yReadings > 0) {
        currentPose.y = correctedY / yReadings;
    }
    chassis.setPose(currentPose);
    return true;
}

} // namespace dsr
