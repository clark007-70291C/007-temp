## Setting Up the Robot Code

Before running the robot, check the device ports and measurements in the source files against the actual robot. Every V5 smart device must use a unique port.

### Main robot setup

Most drivetrain and odometry configuration is in `src/main.cpp`:

- `leftMotors` and `rightMotors` define the drive motors and their directions.
- `drivetrain` sets the track width, wheel type, maximum RPM, and lateral drift.
- `imu`, `verticalEnc`, and `horizontalEnc` define the inertial sensor and tracking-wheel rotation sensors.
- `vertical` and `horizontal` define the tracking-wheel size and each wheel’s offset from the robot center. Measure these offsets from the center of the robot.
- `linearController` and `angularController` contain LemLib’s drive and turn PID settings.
- `chassis` combines the drivetrain, controllers, and odometry sensors.

The autonomous functions set the chassis pose to `(0, 0, 0)` at the robot’s starting point. Use the same coordinate system for all autonomous paths and distance-sensor wall coordinates.

**Check for port conflicts before running:** the current configuration assigns port 19 to both a right drive motor and the vertical rotation sensor. It also assigns port 2 to both the toggle motor and the east distance sensor. Change the port assignments so each device has its own port.

### Lift PID

Lift hardware and tuning are in `src/liftpid.cpp`:

- Set the lift motor ports in the `motors` group.
- Set the rotation sensor port in `rotation`.
- Adjust the PID gains in `lemlib::PID pid(...)` to tune how the lift reaches and holds its target.
- Update `inchesPerMotorRevolution` to match the lift’s actual spool or gear mechanism. The conversion is used to report lift position and move to targets in inches.
- Change the named heights in `moveTo(Position)` to match the robot’s lift positions.

`liftpid::initialize()` starts the background controller and is called during program initialization. Call `liftpid::tarePosition()` when the lift is at its chosen zero point. To move to a height, call `liftpid::moveTo(5.5)` or use a named preset such as `liftpid::moveTo(liftpid::FIRST)`. Manual control is also available for driver control.

### Color seeking

The color-seeking helper is configured in `src/main.cpp` and implemented in `src/colorseek.cpp`:

- Pass the motor that should run while searching and the optical sensor to `colorseek::initialize(...)`.
- Set the motor and optical sensor ports to match the robot.
- Call `colorseek::spinUntil(colorseek::BLUE)`, `RED`, or `YELLOW` where you want the robot to search.

The helper runs the motor while checking hue, saturation, and proximity. It stops the motor when it detects the requested color or when its five-second search timeout expires. The hue and detection thresholds can be adjusted in `src/colorseek.cpp`. Initializing the helper does not start a search by itself; a routine must call `spinUntil`.

### Pure Pursuit

Pure Pursuit follows a path made of `(x, y)` points. Its settings are in `purePursuitConfig` in `src/main.cpp`, and paths are defined in the autonomous functions.

Example:

```cpp
purepursuit::execute({
    {0.0, 0.0},
    {12.0, 0.0},
    {24.0, 8.0}
});
```

The coordinates are in inches and must use the same origin as odometry. Pure Pursuit looks ahead along the path and steers toward a target point. Adjust `lookaheadInches` to change how far ahead it aims, and `maximumLinearVelocity` to limit speed. The track width, wheel diameter, and maximum RPM should match the drivetrain.

An action can be attached to a waypoint. It runs once when odometry says the robot is within `waypointToleranceInches` of that point:

```cpp
{12.0, 0.0, purepursuit::execute([] {
    liftpid::moveTo(liftpid::FIRST);
})}
```

The final point uses `endpointToleranceInches` to determine when the path is complete. `timeoutMilliseconds` limits how long the follower will try to finish. Paths need at least two distinct points; replace the current placeholder coordinates with the actual route before using them.

### Ramsete

Ramsete follows a time-based trajectory rather than just a list of positions. Its tuning is in `skillsRamseteConfig` in `src/main.cpp`.

Each trajectory sample has this order:

```text
time in seconds, X in inches, Y in inches, heading in degrees,
linear velocity in inches/second, angular velocity in radians/second
```

Example:

```cpp
ramsete::execute({
    {0.0,  0.0, 0.0, 0.0,  0.0, 0.0},
    {1.0, 12.0, 0.0, 0.0, 10.0, 0.0},
    {2.0, 20.0, 6.0, 45.0, 0.0, 0.0}
});
```

Ramsete interpolates between samples to calculate the target for the current time, then corrects the drive commands using the difference between the target and the odometry pose. Once the trajectory time ends, it continues correcting toward the final position and heading until it reaches the configured tolerances or the settling timeout expires.

Sample times must start at `0` and increase. The trajectory should use realistic positions, headings, and velocities for the drivetrain. Like Pure Pursuit, actions can be attached to samples and run when the robot is within the waypoint tolerance. The example Ramsete route in the Skills function is commented out, so it must be filled in and enabled before it will run.

### Odometry starting position

Odometry combines the tracking wheels and inertial sensor to estimate the robot’s position and heading. Make sure the tracking-wheel size, offsets, IMU port, and drivetrain measurements match the robot.

The code sets the autonomous starting pose with `chassis.setPose(0, 0, 0)`. This defines the robot’s starting point as `(0, 0)`; it does not mean the field center is `(0, 0)`. All path coordinates and configured wall coordinates must use this same origin.

### Distance sensor reset (DSR)

The optional distance sensor reset is implemented in `src/dsr.cpp`. It supports sensors facing east, west, north, and south:

- Change `DSR_EAST_PORT`, `DSR_WEST_PORT`, `DSR_NORTH_PORT`, and `DSR_SOUTH_PORT` to the sensor ports. Set an unused direction to `0`.
- Set `eastWallX`, `westWallX`, `northWallY`, and `southWallY` to the wall coordinates, in inches, relative to the odometry origin.
- Set each sensor offset to its position relative to the robot center. For example, an east-facing sensor mounted to the right of center has a positive X offset.
- If two sensors on the same axis are enabled and both return valid readings, their position estimates are averaged.

Call `dsr::resetPosition()` after setting the starting pose and before starting the autonomous path if you want the sensor readings to correct odometry. It updates only the coordinates with valid readings, preserves heading, and returns `false` if none of the enabled sensors provides a usable reading. It is not called automatically by the current autonomous code.

Include `dsr.hpp` in `main.cpp` to use the reset helper; do not include `dsr.cpp` directly. The `.cpp` file should be compiled as a normal project source file.

### Autonomous routines

The Brain screen selects which autonomous function to run. Add the route to `autonomousClose()`, `autonomousFar()`, or `autonomousSkills()` in `src/main.cpp`. The current routes are examples/placeholders: replace their coordinates and trajectory samples with the actual field route, and enable the commented Skills examples if you want to use them.
