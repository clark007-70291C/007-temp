# VEX V5 Robot Code

This project contains the PROS C++ code for a VEX V5 robot. It uses LemLib for drivetrain control and odometry, and includes driver-control features, selectable autonomous routines, and helper code for the robot’s mechanisms.

## What the code does

- **Driver control:** Uses the controller joysticks to drive and turn the robot. Controller buttons operate the lift, claw/pneumatic mechanism, and toggle motor.
- **Lift control:** Includes PID-based lift movement, manual control, position taring, and a lower-limit override.
- **Autonomous selection:** Provides Close, Far, and Skills routine options selectable from the Brain screen. The Far routine contains a basic Pure Pursuit example; the Skills code includes commented Pure Pursuit and Ramsete examples to customize.
- **Position tracking:** Uses tracking-wheel encoders and an inertial sensor to estimate the robot’s position and heading. Autonomous starts with the robot pose set to `(0, 0, 0)`.
- **Distance sensor reset helper:** Supports up to four sensors facing the east, west, north, and south walls. Valid readings can reset the corresponding position coordinates while preserving heading. East (port 2) and south (port 3) are configured; west and north are disabled. The reset helper is available but is not called automatically.
- **Additional helpers:** Includes optical color-seeking support, pneumatic control, Brain-screen functions, and chassis-pose telemetry.

Autonomous paths and hardware configuration should be checked and customized for the robot before use.
chatgpt made this becaues I didn't want to make one
