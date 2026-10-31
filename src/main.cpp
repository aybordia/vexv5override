#include "main.h"              // Core PROS header; pulls in the main competition-control hooks and base API
#include "lemlib/api.hpp"       // LemLib API: Chassis, PID controllers, odometry, motion commands (moveToPoint, etc.)
#include "pros/adi.hpp"         // ADI (3-wire) ports: pneumatics, legacy sensors plugged into the brain's 3-wire ports
#include "pros/misc.h"          // Misc PROS stuff: Controller class, competition status, LCD helpers
#include "pros/motors.hpp"      // Motor and MotorGroup classes plus gear-cartridge enums (blue/green/red)
#include "pros/rtos.h"          // Real-time OS helpers: pros::delay (sleep the task), task scheduling
#include <cmath>                // Math functions (not strictly needed here, but handy for sqrt/atan2/etc.)
#include <cerrno>               // Error-number globals; included but unused in this file
#include <cstring>              // C string helpers (strcpy, memset...); included but unused here
#include <cstdio>               // C-style I/O (printf/sprintf); included but unused here

// ---- Hardware declarations: every physical device gets a software object bound to its port ----
//R1 and L2 are raise and lower cascade
//R2 and L1 are to raise and lower the 2 bar
//A for claw expand and A to contract
//Y for flip the claw
//
// Left side of the drivetrain. Two motors on ports 10 and 18.
// The MINUS signs REVERSE those motors, so "forward" voltage spins them the right way for that side.
// Blue gearset = the high-speed 600 RPM cartridge.
pros::MotorGroup left_motors({-10, -18}, pros::MotorGears::blue);

// Right side of the drivetrain. Motors on ports 20 and 11, NOT reversed (positive port numbers).
pros::MotorGroup right_motors({20, 11}, pros::MotorGears::blue);


// Inertial sensor (IMU / gyro) on smart port 1. Used to track the robot's heading (rotation).
pros::Imu imu(1);

// Keep lift motors on ports that do not conflict with the IMU smart port.
pros::Motor lift_motor_left(-15, pros::MotorGears::blue);
pros::Motor lift_motor_right(14, pros::MotorGears::blue);

// 2-bar lift: two motors. PLACEHOLDER PORTS (2 and 3) — change to the real ports.
// Port 3 is negative (reversed) assuming the motors face each other; flip the sign if it fights the other one.
pros::MotorGroup two_bar({2, -3}, pros::MotorGears::green);

// Claw: the 5.5W half motor on port 7 opens and closes it.
// The claw must be CLOSED when the program starts — that spot counts as 0 degrees.
pros::Motor claw_motor(7, pros::MotorGears::green);
const double CLAW_OPEN_DEG = -450;  // how far the motor turns to open (negative = open direction). Bigger number = opens wider
const double CLAW_CLOSED_DEG = 30;   // where the claw closes to. Bigger number = squeezes tighter (0 = where it started)

  // how far the motor turns to open — tune this on the robot (watch the brain screen)
bool claw_open = false;

// Claw flip: a piston on 3-wire port 'H'.
pros::adi::Pneumatics claw_flip_piston('H', false);

// pros::Motor claw_expand;
// ---- Drivetrain geometry: tells LemLib the physical dimensions so it can convert math into motor commands ----
// Args: left group, right group, track width (12.5" = distance between left & right wheels),
//       wheel type (old 3.25" omni), drive RPM (480), horizontal drift factor (2 = how much the robot slides).
lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 12.5, lemlib::Omniwheel::OLD_325, 480, 2);

// Tracking wheels for odometry (position tracking). Here the DRIVE MOTORS double as tracking wheels.
// Left tracking: uses left_motors, 3.25" wheel, offset -6.25" from center (half of track width, left side), 480 RPM.
lemlib::TrackingWheel left_tracking(&left_motors, lemlib::Omniwheel::OLD_325, -12.5 / 2, 480);
// Right tracking: same idea, offset +6.25" (right side of center).
lemlib::TrackingWheel right_tracking(&right_motors, lemlib::Omniwheel::OLD_325, 12.5 / 2, 480);

// Bundle the sensors LemLib uses to figure out where the robot is on the field.
lemlib::OdomSensors sensors(&left_tracking,   // vertical tracking wheel 1 (left)
                            &right_tracking,  // vertical tracking wheel 2 (right)
                            nullptr,          // horizontal tracking wheel 1 — none installed
                            nullptr,          // horizontal tracking wheel 2 — none installed
                            &imu);            // the inertial sensor for heading

// ---- Lateral (forward/backward driving) PID controller settings ----
// These numbers tune how the robot accelerates toward and settles at a target distance.
lemlib::ControllerSettings lateral_controller(2.5,   // kP  — proportional gain (how hard it pushes toward target)
                                              0,     // kI  — integral gain (corrects long-term drift); 0 = off
                                              7,     // kD  — derivative gain (dampens/slows approach to reduce overshoot)
                                              3,     // anti-windup range for the integral term
                                              0.5,   // small-error exit threshold (inches) — "close enough" distance
                                              250,   // small-error timeout (ms) — must stay within that distance this long
                                              2.0,   // large-error exit threshold (inches)
                                              600,   // large-error timeout (ms)
                                              10     // max acceleration (slew rate limit) to prevent wheel slip
);

// ---- Angular (turning in place) PID controller settings ----
// Same structure, but these tune how the robot rotates to a heading.
lemlib::ControllerSettings angular_controller(2,    // kP  — turning proportional gain
                                              0,    // kI  — turning integral gain (off)
                                              10,   // kD  — turning derivative gain (damps the turn)
                                              3,    // anti-windup range
                                              1,    // small-error exit threshold (degrees)
                                              200,  // small-error timeout (ms)
                                              2.0,  // large-error exit threshold (degrees)
                                              600,  // large-error timeout (ms)
                                              0     // max acceleration (0 = no slew limit on turns)
);

// Assemble everything into the main Chassis object. This is what you call move/turn commands on.
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

void initialize() {
    pros::lcd::initialize();                      // Turn on the brain's LCD screen so we can print status text
    pros::lcd::print(0, "1 calibrating...");      // Show "calibrating" on line 0 before calibration begins
    chassis.calibrate();                          // Calibrate IMU + reset odometry; MUST finish before auton runs
    pros::lcd::print(0, "2 calibrate DONE");      // Overwrite line 0 to confirm calibration finished
    pros::delay(300);                             // Wait 300 ms so the "DONE" message is visible / things settle

    // HOLD = lifts lock in place when no button is pressed instead of falling down
    lift_motor_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    lift_motor_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    two_bar.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    claw_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);   // HOLD keeps the claw gripping
    claw_motor.tare_position();                   // wherever the claw starts counts as closed (0 degrees)
}

void disabled() {}                 // Runs when the robot is disabled by the field; empty = do nothing
void competition_initialize() {}   // Runs during comp setup (e.g. auton selector); empty = nothing here


// ---- A specific autonomous routine: "red alliance, right-side starting position" ----
void auton_red_right(){
    // chassis.setPose(0,0,0);                              // Declare current position as origin: x=0, y=0, heading=0
    // chassis.moveToPoint(-14,24,1000);                    // Drive to field point (x=-14, y=24), giving it up to 1000 ms
    // chassis.turnToPoint(chassis.getPose().x,24,2000);    // Turn to face the point directly ahead in Y (same x, y=24), 2000 ms max
    // chassis.moveToPoint(chassis.getPose().x,0,500, {.forwards = false});  // (disabled) would back straight up to y=0 in 500 ms
}
void auton_blue_left(){
    chassis.tank(80, 80, true);
    pros::c::delay(350);
    chassis.tank(-80, -80, true);
    pros::c::delay(350);
    chassis.tank(80, 80, true);
    pros::c::delay(350);
    chassis.tank(-80, -80, true);
    pros::c::delay(350);
    chassis.tank(0, 0);


}
// ---- autonomous(): the 15-second auton period; the field calls this automatically ----
void autonomous() {
    // auton_red_right();   // Run the routine defined above
//     chassis.setPose(0, 0, 0);                 // (all commented out — an alternate routine kept for reference)
//     chassis.moveToPoint(0, 36,3000);
//     chassis.turnToHeading(90,2000);
//     pros::delay(1000);
//     chassis.moveToPoint(20,36,3000);
    // auton_blue_left();
}

// ---- opcontrol(): the driver-control period; a loop that reads the controller forever ----
void opcontrol() {
    while (true) {   // Keep running for the whole driver period
        // Read the left stick's vertical axis (-127 to 127) → forward/back speed
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        // Read the right stick's horizontal axis (-127 to 127) → turning amount
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);   // Drive the robot with the sticks

        // Cascade: hold R1 to raise, hold L2 to lower, let go and it stops
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            lift_motor_left.move(127);
            lift_motor_right.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            lift_motor_left.move(-127);
            lift_motor_right.move(-127);
        } else {
            lift_motor_left.brake();
            lift_motor_right.brake();
        }

        // 2-bar: hold R2 to raise, hold L1 to lower, let go and it holds still
        // if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) two_bar.move(127);
        // else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) two_bar.move(-127);
        // else two_bar.brake();

        // A: press once to open the claw, press again to close it
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            claw_open = !claw_open;
            claw_motor.move_absolute(claw_open ? CLAW_OPEN_DEG : CLAW_CLOSED_DEG, 100);
        }

        // Shows the claw motor's angle so you can find the right CLAW_OPEN_DEG
        pros::lcd::print(1, "Claw: %s  %.0f deg", claw_open ? "OPEN" : "CLOSED", claw_motor.get_position());

        // Y: flip the claw over, press again to flip it back
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) claw_flip_piston.toggle();
        pros::lcd::print(2, "Flip piston (H): %s", claw_flip_piston.is_extended() ? "OUT" : "IN");

        pros::delay(20);   // Pause 20 ms each loop so the CPU isn't pegged and other tasks can run
    }
}