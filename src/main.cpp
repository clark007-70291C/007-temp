#include "lemlib/api.hpp" // IWYU pragma: keep
#include "dsr.hpp" // IWYU pragma: keep
#include "liftpid.hpp"
#include "colorseek.hpp"
#include "controller_buttons.hpp"
#include "piston.hpp"
#include "pros/misc.h"
#include "purepursuit.hpp"
#include "ramsete.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "main.h"
#include "brain_screen.hpp"
#include "pros/rtos.hpp"


// Controller used for driver buttons and joystick input.
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// this so its easy to read for whoever reads but you can define the other buttons ont he controller the same way just rename it
//that is the easiest way to make a driver command compatable for anyone
bool L1= controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
bool L2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
bool R1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
bool R2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
bool A = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
bool B = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
bool X = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
bool Y = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
 bool DOWN = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    bool UP = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    bool RIGHT = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    bool LEFT = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
// Robot mechanisms.
pros::adi::DigitalOut Claw('A');
pros::Motor TOGGLE(2);
pros::Optical toggleColorSensor(6);

// Drive motors. A negative port number reverses that motor.
pros::MotorGroup leftMotors({-11, -12 },
                            pros::MotorGearset::blue);
pros::MotorGroup rightMotors({19, 20}, pros::MotorGearset::blue);

// Inertial sensor is used to measure robot heading.
pros::Imu imu(1);

// Odometry tracking wheels. Offsets are measured from the robot's center.
pros::Rotation verticalEnc(19);
pros::Rotation horizontalEnc(18);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 0.0);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -2.0);

// Drivetrain measurements and speed settings.
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12, // Wheel-to-wheel distance, in inches
                              lemlib::Omniwheel::NEW_275, // 2.75-inch wheels
                              450, // Maximum drivetrain RPM
                              8 // Lateral drift setting
);

// PID settings for straight driving and turning.
lemlib::ControllerSettings linearController(8, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            20, // derivative gain (kD)
                                            3, // Integral anti-windup limit
                                            1, // Small position error, in inches
                                            100, // Time within small error, in ms
                                            3, // Large position error, in inches
                                            500, // Time within large error, in ms
                                            20 // Maximum acceleration
);

lemlib::ControllerSettings angularController(4.50, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             30.0, // derivative gain (kD)
                                             3, // Integral anti-windup limit
                                             1, // Small heading error, in degrees
                                             100, // Time within small error, in ms
                                             3, // Large heading error, in degrees
                                             500, // Time within large error, in ms
                                             0 // Maximum acceleration
);

// Sensors used to track the robot's position.
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // second vertical wheel (not used)
                            &horizontal, // horizontal tracking wheel
                            nullptr, // second horizontal wheel (not used)
                            &imu // inertial sensor
);

// Make joystick driving easier to control at low speeds.
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019); // curve strength
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019); // curve strength

// LemLib combines the motors, movement controllers, and odometry here.
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// Choose the follower used by the Skills test routine.



// Pure Pursuit tuning. Waypoint tolerance is how close the robot must get to a
// path point before that point's attached action runs.
purepursuit::Config purePursuitConfig{
    .lookaheadInches = 8.0,
    .maximumLinearVelocity = 20.0,
    .trackWidthInches = 12.0,
    .wheelDiameterInches = 2.75,
    .maximumMotorRpm = 450.0,
    .waypointToleranceInches = 1.5,
    .endpointToleranceInches = 1.0,
    .timeoutMilliseconds = 5000
};

// Ramsete tuning for the alternative Skills path.
ramsete::Config skillsRamseteConfig{
    .trackWidthInches = 12.0,
    .wheelDiameterInches = 2.75,
    .maximumMotorRpm = 450.0,
    .waypointToleranceInches = 1.5,
    .finalPositionToleranceInches = 1.0,
    .finalHeadingToleranceDegrees = 3.0,
    .finalSettleTimeoutSeconds = 1.5
};

// Reset odometry from the east and south walls but teh sensors are facing north and west to begin with distance sensors configured in dsr.cpp.
// Call this at a spot where the robot is square to those walls.
// dsr::resetWall();

// Set up screens, sensors, and background tasks at program start.
void initialize() {
    brain_screen::initialize();
    colorseek::initialize(TOGGLE, toggleColorSensor);
    PISTON::set(false);

    chassis.calibrate();
    wall_localization::start();
    liftpid::initialize();

    // Keep the robot's current position visible in the brain log.
    pros::Task telemetryTask([&]() {
        while (true) {
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
}

// Stop lift PID movement while disabled.
void disabled() {
    liftpid::stop();
}

// Optional setup hook called before a competition match.
void competition_initialize() {
   
}



// Close-side autonomous path. Positions are field coordinates in inches.
void autonomousClose() {
   // this is how you use the custom liftpid but you can also just use inches instead of teh positons if you want liftpid::moveTo(liftpid::FIRST);
}

// Far-side autonomous path.
void autonomousFar() {
    purepursuit::execute({
{0.0 , 0.0},
{0.0 , 0.0, purepursuit::execute([] { liftpid::moveTo(liftpid::FIRST); })},
    });
}

// Add the skills route here.
void autonomousSkills() {
    
        // Points are inches; attached actions run when the robot reaches them.
        /*purepursuit::execute({
            {0.0, 0.0},
            {11.0, 0.0, purepursuit::execute([] { TOGGLE.move(127); })},
            {22.0, 0.0, purepursuit::execute([] { TOGGLE.move(0); })}
        });
       

    // Ramsete rows: time, X, Y, heading degrees, linear speed, angular speed.
    ramsete::execute({
        {0.0, 0.0,  0.0, 0.0, 0.0, 0.0},
        {0.5, 3.0,  0.0, 0.0, 10.0, 0.0,
         ramsete::execute([] { TOGGLE.move(127); })},
        {1.5, 14.0, 0.0, 0.0, 12.0, 0.0,
         ramsete::execute([] { TOGGLE.move(0); })},
        {2.5, 22.0, 0.0, 0.0,  4.0, 0.0,
         ramsete::execute([] { PISTON::set(true); })},
        {3.0, 22.0, 0.0, 0.0,  0.0, 0.0}
    });
      
    pure pursuit and ramsete examples are shown here to demostrate how to use them. pure pursuit is alot easier for beginners 
    ramsete is more advanced and alot better for more complex routes and path tracing
    */
}    

// Run the route selected on the brain screen.
void runSelectedAuton() {
    switch (brain_screen::getSelectedAutonRoutine()) {
        case brain_screen::AutonRoutine::CLOSE:
            autonomousClose();
            break;
        case brain_screen::AutonRoutine::FAR:
            autonomousFar();
            break;
        case brain_screen::AutonRoutine::SKILLS:
            autonomousSkills();
            break;
    }
}

// Press the chosen button to run the selected autonomous when not connected to field controller 
void runAuton(pros::controller_digital_e_t button) {
    if (!controller.get_digital_new_press(button) || pros::competition::is_connected()) {
        return;
    }

    brain_screen::requestAutonScreenTransition();
    chassis.setPose(0, 0, 0);
    liftpid::tarePosition();
    runSelectedAuton();
    liftpid::stop();
}

// PROS calls this when autonomous mode starts.
void autonomous() {
    brain_screen::requestAutonScreenTransition();
    liftpid::tarePosition();

    runSelectedAuton();
}
// PROS calls this repeatedly during driver-control mode.
void opcontrol() {
    liftpid::stop(); // this just for my code but it disables the liftpid when driver starts

    while (true) {
        // Press LEFT to run the selected autonomous when field control is disconnected.
        runAuton(BUTTON_LEFT);

        // R1 toggles the claw.
        PISTON::toggle(R1);

        // L2 raises the lift; R2 lowers it but stops at zero. RIGHT overrides
        // the zero stop while held, so use it only when the lift is stuck.
         bool liftUp = L2;
        bool liftDown = R2;
         bool lowerOverride = RIGHT;
        if (A) { 
            liftpid::tarePosition(); // this if A just reset the lift position to 0
        }
        liftpid::manualControl(liftUp, liftDown || lowerOverride, lowerOverride);
TOGGLE.move(DOWN ? -127 : 0); // this is just a simplified if and else statement
        // Hold L1 to run the intake/search motor.
   
        TOGGLE.move(L1 ? -127 : 0); // this is just a simplified if and else statement 
        // -127 is spinning backwards and at full speed :0 says otherwise stop the motor the question mark is a ternary operator
        // a ternary operator means that if the condition is true then the first value is used if not then the second value is used

       // you can swap chassis.arcade with chassis.tank if you want single stick arcade just change ANALOG_RIGHT_X to ANALOG_LEFT_X
       int forwardPower = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
       int turnPower = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(forwardPower, turnPower);

        // Run this control loop every 10 milliseconds.
        pros::delay(10);
    }
}
//this is so i dont forget how to update the repo when i update the code 
//git add .
//git commit -m "Describe your change" just change this to whatever i did 
//git push