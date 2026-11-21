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
// pros::MotorGroup dr4b_motors{1,-2};   // DISABLED: port 2 is a drive motor
pros::MotorGroup left_motors({8, -4}, pros::MotorGears::blue);   // physical LEFT side; positive = forward

// Right side of the drivetrain.
pros::MotorGroup right_motors({-2, 7}, pros::MotorGears::blue);  // physical RIGHT side; positive = forward


// Inertial sensor (IMU / gyro) on smart port 6. Used to track the robot's heading (rotation).
pros::Imu imu(10);

// Keep lift motors on por  ts that do not conflict with the IMU smart port.
pros::Motor lift_motor_left(20, pros::MotorGears::red);
pros::Motor lift_motor_right(11, pros::MotorGears::red);
pros::Motor ratchet_motor(5, pros::MotorGears::blue);

// 2-bar lift: two motors. PLACEHOLDER PORTS (2 and 3) — change to the real ports.
// Port 3 is negative (reversed) assuming the motors face each other; flip the sign if it fights the other one.
// pros::MotorGroup two_bar({2, -3}, pros::MotorGears::green);   // DISABLED: port 2 is a drive motor

// Claw: the 5.5W half motor on port 7 opens and closes it.
// The claw must be CLOSED when the program starts — that spot counts as 0 degrees.
// pros::Motor claw_motor(7, pros::MotorGears::green);   // DISABLED: port 7 is a drive motor
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
lemlib::ControllerSettings lateral_controller(50,    // kP
                                              0,     // kI
                                              40,    // kD
                                              3,     // anti-windup
                                              1,     // small-error exit threshold (inches)
                                              100,   // small-error timeout (ms)
                                              3,     // large-error exit threshold (inches)
                                              500,   // large-error timeout (ms)
                                              8      // max acceleration (slew)
);

// ---- Angular (turning in place) PID controller settings ----
// Same structure, but these tune how the robot rotates to a heading.
lemlib::ControllerSettings angular_controller(14,   // kP
                                              0,    // kI
                                              17,   // kD
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
    // Both drive sides MUST use the same brake mode, or the robot yaws when it stops
    // BRAKE = wheels stop when sticks are released instead of rolling (COAST)
    left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
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
    // forward / back / forward / back
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0,  4, 1000);
    chassis.moveToPoint(0,  0, 1000, {.forwards = false});
    chassis.moveToPoint(0,  4, 1000);
    chassis.moveToPoint(0,  0, 1000, {.forwards = false});
}

// ---- opcontrol(): the driver-control period; a loop that reads the controller forever ----
void opcontrol() {
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