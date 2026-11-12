## 007-Temp


## Path Following

The project includes two path-following options: Pure Pursuit and Ramsete. Both use the robot’s odometry pose and command the left and right drive motors to follow a route. Coordinates are in inches and use the same field coordinate system as odometry, where the robot starts at `(0, 0)`.

### Pure Pursuit

Pure Pursuit follows a path made from a sequence of `(x, y)` waypoints. It finds the robot’s progress along the path, selects a target point a configurable distance ahead (the *lookahead*), and steers toward that point. As the robot approaches the endpoint, it slows down. It stops when it gets within the endpoint tolerance or if the path times out.

An optional action can be attached to a waypoint. That action runs once when odometry says the robot is close enough to the waypoint. Pure Pursuit is useful for following a path defined by positions without specifying an exact time or speed for every point.

### Ramsete

Ramsete follows a time-based trajectory. Each sample specifies a time, position `(x, y)`, heading, linear velocity, and angular velocity. The controller interpolates between samples to get the target for the current time, then corrects the drive commands based on the difference between the target and the robot’s measured pose.

After the trajectory time ends, Ramsete continues correcting toward the final position and heading until it reaches the configured tolerances or the settling timeout expires. Actions can also be attached to samples and run once the robot is within the waypoint tolerance.

### Choosing and configuring a path

Use Pure Pursuit when you want to provide a series of positions and let the controller choose how to steer toward them. Use Ramsete when you have a time-parameterized trajectory with desired speeds and headings.

Both options depend on accurate odometry and correct drivetrain settings, such as track width, wheel diameter, and maximum motor RPM. The example routes in the source need to be filled in and checked for the robot’s starting position and field layout before use.
