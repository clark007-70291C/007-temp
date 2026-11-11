#include "purepursuit.hpp"

#include "lemlib/chassis/chassis.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <vector>

extern lemlib::Chassis chassis;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern purepursuit::Config purePursuitConfig;

namespace purepursuit {
namespace {

constexpr double pi = 3.14159265358979323846;

struct Segment {
    double startX;
    double startY;
    double dx;
    double dy;
    double length;
    double startDistance;
};

bool validConfig(const Config& config) {
    return std::isfinite(config.lookaheadInches) && config.lookaheadInches > 0 &&
           std::isfinite(config.maximumLinearVelocity) &&
           config.maximumLinearVelocity > 0 &&
           std::isfinite(config.trackWidthInches) && config.trackWidthInches > 0 &&
           std::isfinite(config.wheelDiameterInches) && config.wheelDiameterInches > 0 &&
           std::isfinite(config.maximumMotorRpm) && config.maximumMotorRpm > 0 &&
           std::isfinite(config.waypointToleranceInches) &&
           config.waypointToleranceInches > 0 &&
           std::isfinite(config.endpointToleranceInches) &&
           config.endpointToleranceInches > 0 &&
           config.timeoutMilliseconds > 0;
}

bool makeSegments(const Point* path, std::size_t pointCount,
                  std::vector<Segment>& segments, double& totalLength) {
    if (path == nullptr || pointCount < 2) {
        return false;
    }

    totalLength = 0;
    for (std::size_t i = 0; i < pointCount; ++i) {
        if (!std::isfinite(path[i].x) || !std::isfinite(path[i].y)) {
            return false;
        }
        if (i == 0) {
            continue;
        }

        const double dx = path[i].x - path[i - 1].x;
        const double dy = path[i].y - path[i - 1].y;
        const double length = std::hypot(dx, dy);
        if (length <= 1e-6) {
            continue;
        }

        segments.push_back({path[i - 1].x, path[i - 1].y, dx, dy,
                            length, totalLength});
        totalLength += length;
    }
    return !segments.empty();
}

double closestProgress(const std::vector<Segment>& segments, double x, double y,
                       double currentProgress) {
    double bestDistanceSquared = INFINITY;
    double bestProgress = currentProgress;

    for (const Segment& segment : segments) {
        const double segmentEnd = segment.startDistance + segment.length;
        if (segmentEnd < currentProgress) {
            continue;
        }

        const double t = std::clamp(
            ((x - segment.startX) * segment.dx +
             (y - segment.startY) * segment.dy) /
                (segment.length * segment.length),
            0.0, 1.0);
        const double projectedX = segment.startX + t * segment.dx;
        const double projectedY = segment.startY + t * segment.dy;
        const double dx = x - projectedX;
        const double dy = y - projectedY;
        const double distanceSquared = dx * dx + dy * dy;
        const double progress = segment.startDistance + t * segment.length;

        if (progress >= currentProgress && distanceSquared < bestDistanceSquared) {
            bestDistanceSquared = distanceSquared;
            bestProgress = progress;
        }
    }
    return bestProgress;
}

Point pointAtDistance(const std::vector<Segment>& segments, double distance,
                      const Point& endpoint) {
    for (const Segment& segment : segments) {
        if (distance <= segment.startDistance + segment.length) {
            const double t = std::clamp(
                (distance - segment.startDistance) / segment.length, 0.0, 1.0);
            return {segment.startX + t * segment.dx,
                    segment.startY + t * segment.dy};
        }
    }
    return endpoint;
}

Point circleLookaheadPoint(const std::vector<Segment>& segments, double x, double y,
                           double radius, double currentProgress, double totalLength,
                           const Point& endpoint) {
    double furthestIntersection = -1;
    Point target = pointAtDistance(
        segments, std::min(currentProgress + radius, totalLength), endpoint);

    for (const Segment& segment : segments) {
        if (segment.startDistance + segment.length < currentProgress) {
            continue;
        }

        const double offsetX = segment.startX - x;
        const double offsetY = segment.startY - y;
        const double a = segment.dx * segment.dx + segment.dy * segment.dy;
        const double b = 2.0 * (offsetX * segment.dx + offsetY * segment.dy);
        const double c = offsetX * offsetX + offsetY * offsetY - radius * radius;
        const double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0) {
            continue;
        }

        const double root = std::sqrt(discriminant);
        const double candidates[] = {
            (-b - root) / (2.0 * a),
            (-b + root) / (2.0 * a)
        };
        for (double t : candidates) {
            if (t < 0 || t > 1) {
                continue;
            }

            const double progress = segment.startDistance + t * segment.length;
            if (progress >= currentProgress && progress > furthestIntersection) {
                furthestIntersection = progress;
                target = {segment.startX + t * segment.dx,
                          segment.startY + t * segment.dy};
            }
        }
    }
    return target;
}

void stopDrive(pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors) {
    leftMotors.move(0);
    rightMotors.move(0);
}

void runReachedActions(const Point* path, std::size_t pointCount,
                       std::size_t& nextAction, const lemlib::Pose& pose,
                       double toleranceInches) {
    while (nextAction < pointCount) {
        if (path[nextAction].action == nullptr) {
            ++nextAction;
            continue;
        }

        const double dx = path[nextAction].x - pose.x;
        const double dy = path[nextAction].y - pose.y;
        if (std::hypot(dx, dy) > toleranceInches) {
            break;
        }

        path[nextAction].action();
        ++nextAction;
    }
}

void drive(pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors,
           double linearVelocity, double angularVelocity, const Config& config) {
    double leftVelocity =
        linearVelocity - angularVelocity * config.trackWidthInches / 2.0;
    double rightVelocity =
        linearVelocity + angularVelocity * config.trackWidthInches / 2.0;
    const double maximumWheelVelocity =
        config.maximumMotorRpm * pi * config.wheelDiameterInches / 60.0;
    const double fastest = std::max(std::abs(leftVelocity), std::abs(rightVelocity));
    if (fastest > maximumWheelVelocity) {
        const double scale = maximumWheelVelocity / fastest;
        leftVelocity *= scale;
        rightVelocity *= scale;
    }

    const double inchesPerRevolution = pi * config.wheelDiameterInches;
    leftMotors.move_velocity(static_cast<int32_t>(
        std::lround(leftVelocity * 60.0 / inchesPerRevolution)));
    rightMotors.move_velocity(static_cast<int32_t>(
        std::lround(rightVelocity * 60.0 / inchesPerRevolution)));
}

} // namespace

Action execute(Action action) {
    return action;
}

static bool followPath(lemlib::Chassis& chassis, pros::MotorGroup& leftMotors,
                       pros::MotorGroup& rightMotors, const Point* path,
                       std::size_t pointCount, const Config& config) {
    std::vector<Segment> segments;
    double totalLength = 0;
    if (!validConfig(config) ||
        !makeSegments(path, pointCount, segments, totalLength)) {
        stopDrive(leftMotors, rightMotors);
        return false;
    }

    const Point endpoint = path[pointCount - 1];
    double progress = 0;
    std::size_t nextAction = 0;
    const std::uint32_t startTime = pros::millis();

    while (static_cast<std::uint32_t>(pros::millis() - startTime) <
           config.timeoutMilliseconds) {
        // Standard pose gives a conventional math heading in radians.
        const lemlib::Pose pose = chassis.getPose(true, true);
        runReachedActions(path, pointCount, nextAction, pose,
                          config.waypointToleranceInches);
        const double dxToEnd = endpoint.x - pose.x;
        const double dyToEnd = endpoint.y - pose.y;
        const double distanceToEnd = std::hypot(dxToEnd, dyToEnd);
        if (distanceToEnd <= config.endpointToleranceInches) {
            stopDrive(leftMotors, rightMotors);
            return true;
        }

        progress = closestProgress(segments, pose.x, pose.y, progress);
        const Point target = circleLookaheadPoint(
            segments, pose.x, pose.y, config.lookaheadInches, progress,
            totalLength, endpoint);

        const double targetX = target.x - pose.x;
        const double targetY = target.y - pose.y;
        const double forwardError =
            std::cos(pose.theta) * targetX + std::sin(pose.theta) * targetY;
        const double lateralError =
            -std::sin(pose.theta) * targetX + std::cos(pose.theta) * targetY;
        const double targetDistanceSquared =
            targetX * targetX + targetY * targetY;
        const double curvature = targetDistanceSquared > 1e-6
            ? 2.0 * lateralError / targetDistanceSquared
            : 0.0;

        const double speed = std::min(
            config.maximumLinearVelocity,
            std::max(4.0, distanceToEnd * 2.0));
        drive(leftMotors, rightMotors, speed, speed * curvature, config);
        pros::delay(10);
    }

    stopDrive(leftMotors, rightMotors);
    return false;
}

bool execute(std::initializer_list<Point> path) {
    return followPath(chassis, leftMotors, rightMotors, path.begin(), path.size(),
                      purePursuitConfig);
}

} // namespace purepursuit
