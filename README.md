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


## Installing and Running the Project

This is a PROS project for a VEX V5 robot. You can get a local copy with Git Bash, another Windows terminal, or by downloading the project as a ZIP file.

### Option 1: Install Git Bash, then clone the project

Git Bash is included with **Git for Windows**.

1. Download Git for Windows from [git-scm.com/download/win](https://git-scm.com/download/win).
2. Run the installer. The default options are fine.
3. Open **Git Bash** from the Windows Start menu.
4. Choose where you want the project saved. For example, to use your Documents folder, run:

   ```bash
   cd ~/Documents
   ```

5. Clone the repository and enter its folder:

   ```bash
   git clone https://github.com/clark007-70291C/CHS-LEMLIB_LIB.git
   cd CHS-LEMLIB_LIB
   ```

The project files will now be in a `CHS-LEMLIB_LIB` folder inside Documents.

### Option 2: Use PowerShell or Command Prompt

Git Bash is optional. If **Git for Windows** is installed, you can run Git commands from PowerShell or Command Prompt.

**PowerShell:**

```powershell
Set-Location "$HOME\Documents"
git clone https://github.com/clark007-70291C/CHS-LEMLIB_LIB.git
Set-Location "CHS-LEMLIB_LIB"
```

**Command Prompt:**

```cmd
cd /d "%USERPROFILE%\Documents"
git clone https://github.com/clark007-70291C/CHS-LEMLIB_LIB.git
cd CHS-LEMLIB_LIB
```

If Git is not installed, install Git for Windows as described in Option 1, or use the ZIP method below.

### Option 3: Download without using Git

1. Open the repository page: [github.com/clark007-70291C/CHS-LEMLIB_LIB](https://github.com/clark007-70291C/CHS-LEMLIB_LIB).
2. Select **Code**, then **Download ZIP**.
3. Extract the ZIP file to a location you can find, such as Documents.
4. Open the extracted project folder. It should contain `project.pros`.

This method does not require Git or Git Bash. You also won’t be able to use Git to pull updates unless you install Git later.

### Open, build, and upload the project

1. Install **Visual Studio Code** and the **PROS extension/toolchain** for VEX V5.
2. In VS Code, select **File > Open Folder** and choose the project folder containing `project.pros`.
3. Connect and power on the V5 Brain.
4. Build and upload using the PROS extension, or open a terminal in the project folder and run:

   ```bash
   pros make
   pros upload
   ```

The PROS toolchain must be installed for these commands to work. Uploading requires the Brain to be connected to the computer.

### Before running on the robot

Check the hardware port assignments and measurements in the source files and make sure they match your robot. Each V5 smart device must use a unique port. The current configuration assigns port 19 to both a drive motor and the vertical rotation sensor, and port 2 to both the toggle motor and the east distance sensor. Resolve those conflicts before using the code.

Also, `main.cpp` should include `dsr.hpp`, not `dsr.cpp`. The `dsr.cpp` implementation should be compiled as a normal project source file.
