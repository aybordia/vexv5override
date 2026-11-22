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
//A toggles the ratchet claw spin on/off; B opens it
//Y for flip the claw
// Drive motors.  Left:  front=15, back=10   Right: front=20, back=19
// Direction signs follow Ajay's working convention (one motor per side negated);
// flip a sign if a side spins backward.
pros::MotorGroup left_motors ({15, -10}, pros::MotorGears::blue);   // {front, back}, positive = forward
pros::MotorGroup right_motors({-20, 19}, pros::MotorGears::blue);   // {front, back}, positive = forward

// Inertial sensor (IMU / gyro).  Was on port 10; moved to 11 because 10 is now a drive motor.
pros::Imu imu(11);

// Lift motors (red cartridges).  Left=18, Right=2 (port negated so the two motors don't fight each other).
pros::Motor lift_motor_left (18, pros::MotorGears::red);
pros::Motor lift_motor_right(-2, pros::MotorGears::red);

// Ratcheting claw motor on port 1.
pros::Motor ratchet_motor(1, pros::MotorGears::blue);
const double CLAW_OPEN_DEG = -450;  // how far the motor turns to open (negative = open direction). Bigger number = opens wider
const double CLAW_CLOSED_DEG = 30;   // where the claw closes to. Bigger number = squeezes tighter (0 = where it started)

  // how far the motor turns to open — tune this on the robot (watch the brain screen)
bool claw_open = false;
bool ratchet_spinning = false;   // toggle state for the ratchet claw (A flips it)

// Claw flip: a piston on 3-wire port 'H'.
pros::adi::Pneumatics claw_flip_piston('H', false);

// pros::Motor claw_expand;
// ---- Drivetrain geometry: tells LemLib the physical dimensions so it can convert math into motor commands ----
// Args: left group, right group, track width (12.5" = distance between left & right wheels),
//       wheel type (old 3.25" omni), drive RPM (480), horizontal drift factor (2 = how much the robot slides).
lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 12.5, lemlib::Omniwheel::OLD_325, 360, 2);

// Tracking wheels for odometry (position tracking). Here the DRIVE MOTORS double as tracking wheels.
// Left tracking: uses left_motors, 3.25" wheel, offset -6.25" from center (half of track width, left side), 480 RPM.
// ODOM_RPM: tune this if auton drives short/long. Odom thought it went 24" but the robot only went ~19",
// so encoders were over-counting by 24/19 -> scale 480 down to 380. Rule: new = old * (actual / target).
const float ODOM_RPM = 360;   // blue 600rpm motor on 36T -> 60T wheel gear = 600*36/60
lemlib::TrackingWheel left_tracking(&left_motors, lemlib::Omniwheel::OLD_325, -12.5 / 2, ODOM_RPM);
// Right tracking: same idea, offset +6.25" (right side of center).
lemlib::TrackingWheel right_tracking(&right_motors, lemlib::Omniwheel::OLD_325, 12.5 / 2, ODOM_RPM);

// Bundle the sensors LemLib uses to figure out where the robot is on the field.
lemlib::OdomSensors sensors(&left_tracking,   // vertical tracking wheel 1 (left)
                            &right_tracking,  // vertical tracking wheel 2 (right)
                            nullptr,          // horizontal tracking wheel 1 — none installed
                            nullptr,          // horizontal tracking wheel 2 — none installed
                            &imu);            // the inertial sensor for heading

// ---- Lateral (forward/backward driving) PID controller settings ----
// These numbers tune how the robot accelerates toward and settles at a target distance.
lemlib::ControllerSettings lateral_controller(12,    // kP  — was 50: robot stayed at full speed until ~2" out, then slammed. 12 starts slowing ~10" out
                                              0,    // kI  — off; integral was winding up and jerking at the end
                                              6,     // kD  — light damping so it eases in instead of stopping hard
                                              3,     // anti-windup (integral starts accumulating within 3")
                                              0.5,   // small-error exit threshold (inches)
                                              200,   // small-error timeout (ms)
                                              1,     // large-error exit threshold (inches)
                                              300,   //] large-error timeout (ms)
                                              8      // max acceleration (slew)
);

// ---- Angular (turning in place) PID controller settings ----
// Same structure, but these tune how the robot rotates to a heading.
lemlib::ControllerSettings angular_controller(5.127, // kP
                                              0,    // kI
                                              39,   // kD  — was 39.812; bumped to stop the small overshoot
                                              3,    // anti-windup
                                              1,    // small-error exit threshold (degrees)
                                              100,  // small-error timeout (ms)
                                              3,    // large-error exit threshold (degrees)
                                              500,  // large-error timeout (ms)
                                              0     // no slew on turns
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
    // Both drive sides MUST use the same brake mode, or the robot yaws when it stops.
    // COAST = wheels glide to a stop instead of actively clamping; softer end-of-motion feel.
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}

void disabled() {}
void competition_initialize() {}


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
// Turn test: 90 right, back to 0, then 180. Prints final heading so you can see over/undershoot.
void angular_test() {
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 2000);
    chassis.waitUntilDone();
    pros::lcd::print(3, "after 90:  %.1f deg", chassis.getPose().theta);
    pros::delay(1000);
    chassis.turnToHeading(0, 2000);
    chassis.waitUntilDone();
    pros::lcd::print(4, "after 0:   %.1f deg", chassis.getPose().theta);
    pros::delay(1000);
    chassis.turnToHeading(180, 2000);
    chassis.waitUntilDone();
    pros::lcd::print(5, "after 180: %.1f deg", chassis.getPose().theta);
}

const bool RUN_TURN_TEST = true;   // true = run angular_test() in auton; false = normal 24" drive

void autonomous() {
    chassis.setPose(0, 0, 0);   // reset odometry to origin (0,0) facing forward (0 deg)
    chassis.turnToHeading(90, 2000);   // turn to 90 deg heading, 2 sec timeout
}
void drive_port_test() {
    const int ports[] = {2, 4, 7, 8};
    for (int port : ports) {
        pros::Motor m(port);
        pros::lcd::print(4, "TEST: port %d spinning +   ", port);
        controller.print(0, 0, "Port %d +      ", port);
        m.move(50);
        pros::delay(1500);
        m.move(0);
        pros::delay(1000);
    }
    pros::lcd::print(4, "TEST done                ");
    controller.print(0, 0, "Test done      ");
}

// ---- opcontrol(): the driver-control period; a loop that reads the controller forever ----
void opcontrol() {
    pros::lcd::print(5, "build: drive-fix-2");   // proves this exact code is on the robot
    while (true) {   // Keep running for the whole driver period

        // Read the left stick's vertical axis (-127 to 127) → forward/back speed
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        // Read the right stick's horizontal axis (-127 to 127) → turning amount
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Deadband: ignore tiny stick values from joystick drift so the robot doesn't creep
        if (std::abs(leftY) < 5) leftY = 0;
        if (std::abs(rightX) < 5) rightX = 0;

        chassis.arcade(leftY, rightX);   // Drive the robot with the sticks)

        // Ratchet claw: A toggles continuous spin on/off, B opens while held
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            ratchet_spinning = !ratchet_spinning;   // each A press flips it
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            ratchet_motor.move(127);                // B held = open
            ratchet_spinning = false;               // opening cancels the spin
        } else if (ratchet_spinning) {
            ratchet_motor.move(-127);               // toggled on = keep spinning
        } else {
            ratchet_motor.brake();                  // toggled off = stop
        }

        // Lift: direction flip is already baked in at the motor declaration (lift_motor_right uses port -2),
        // so both sides take the SAME sign here.
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            lift_motor_left.move(90);
            lift_motor_right.move(90);
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
        // NOTE: A is ALSO toggling the ratchet above — move this to a free button.
        // Claw disabled: claw_motor was on port 7, which is a drive motor
        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        //     claw_open = !claw_open;
        //     claw_motor.move_absolute(claw_open ? CLAW_OPEN_DEG : CLAW_CLOSED_DEG, 100);
        // }
        // pros::lcd::print(1, "Claw: %s  %.0f deg", claw_open ? "OPEN" : "CLOSED", claw_motor.get_position());

        // Y: flip the claw over, press again to flip it back
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) claw_flip_piston.toggle();
        pros::lcd::print(2, "Flip piston (H): %s", claw_flip_piston.is_extended() ? "OUT" : "IN");

        pros::delay(20);   // Pause 20 ms each loop so the CPU isn't pegged and other tasks can run
    }
}
