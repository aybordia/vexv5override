#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/rtos.h"

/**
 * A callback functioxn for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
 // motor setups
pros::MotorGroup left_motors({-8, 10, -2}); // left motors on ports 1, 2, 3
pros::MotorGroup right_motors({19, -13, 16}); // right motors on ports 4, 5, 6
pros::Motor double_intake(6);
pros::Motor top_gintake(4);
pros::Motor top_fintake(11);
pros::Motor bottom_gintake(17);
void bucket_intake() {
    double_intake.move(127);
    top_gintake.move(127);
    top_fintake.move(127);

}
void high_intake() {
    double_intake.move(127);
    top_gintake.move(127);
    top_fintake.move(-227);
	bottom_gintake.move(-127);

}

void middle_intake() {
	double_intake.move(127);
	top_gintake.move(-127);
	bottom_gintake.move(-127);

}
void low_intake() {
	double_intake.move(-127);
	bottom_gintake.move(-127);

}
void stopbucket_intake() {
    double_intake.move(0);
    top_gintake.move(0);
    top_fintake.move(0);
    bottom_gintake.move(0);

}
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.5, // 10 inch track width
                              lemlib::Omniwheel::OLD_325, //3.25 in omnis
                              480, // drivetrain rpm is 360
                             
                              2 // horizontal drift is 2 (for now)
);
pros::Imu imu(15);
// pneumatics class
pros::adi::Pneumatics loader('h', false);
pros::adi::Pneumatics descore('g', false);
// create a v5 rotation sensor on port 1
pros::Rotation vertical_sensor(10);
// replace 1 with the port the rotation sensor is connected to
pros::Rotation horizontal_sensor(14);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_sensor, lemlib::Omniwheel::NEW_275, -2.5);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, lemlib::Omniwheel::NEW_275, -2.5);



lemlib::OdomSensors sensors(
    nullptr, // horizontal tracking wheel
    nullptr, // vertical tracking wheel 1 
    &horizontal_tracking_wheel,
    nullptr, // inertial sensor 1 (optional)
    &imu     // inertial sensor (REQUIRED)
);


// lateral PID controller
lemlib::ControllerSettings lateral_controller(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              37, // derivative gain (kD)
                                              3, // anti windup
                                              0.5, // small error range, in inches
                                              250, // small error range timeout, in milliseconds
                                              2.0, // large error range, in inches
                                              600, // large error range timeout, in milliseconds
                                              10 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
    6, // kP
    0,
    52, // kD
    3,
    0.75,
    200,
    2.0,
    600,
    0
);



// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**sss
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
	chassis.calibrate();
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.s
 *ss
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.

 */
enum intake_state {
    INTAKE,
    STOP
};

intake_state current = STOP;
void middle_goaal() {
    chassis.setPose(15, 49.5, 3000);
    chassis.moveToPoint(33,48,3000);
    chassis.turnToPoint(22,22,3000);
    bucket_intake();
    chassis.moveToPoint(26, 24, 3000, {.maxSpeed = 30});    
    chassis.moveToPoint(12, 12,3000, {.maxSpeed = 80});
    chassis.moveToPoint(13,13, 3000);
    stopbucket_intake();
    pros::delay(1500);
    middle_intake();
}
// void skills_goal() {
//     chassis.setPose(15, -49.5, 90); 
//     chassis.moveToPoint(34,48,3000);
//     chassis.turnToPoint(24,24,3000);
//     bucket_intake();
//     chassis.moveToPoint(24, 24,3000, {.maxSpeed = 80});
//     chassis.moveToPoint(23, 19,3000, {.maxSpeed = 80});
//     chassis.turnToPoint(0,0,3000);
//     stopbucket_intake();
//     pros::delay(500);
//     middle_intake();
// }  
void autonomous() {
	// chassis.setPose(0,0,0);
	// chassis.moveToPoint(0,48,7000);
	// return;  
	chassis.setPose(15,-49.5,90); // starting pose

    ////middle goal
    // chassis.moveToPoint(34,48,3000);
    // chassis.turnToPoint(24,24,3000);
    // bucket_intake();
    // chassis.moveToPoint(24, 24,3000, {.maxSpeed = 80});
    // chassis.moveToPoint(23, 19,3000, {.maxSpeed = 80});
    // chassis.turnToPoint(0,0,3000);
    // stopbucket_intake();
    // pros::delay(500);
    // middle_intake();
    // return;
    // pros::delay(1500);
    // chassis.turnToPoint(48,chassis.getPose().y,3000);
    // chassis.moveToPoint(59,chassis.getPose().y,3000);
    // stopbucket_intake();
    // loader.set_value(true);
    // chassis.turnToHeading(180,1000);
    // chassis.moveToPoint(61,57,3000);
    // chassis.moveToPoint(61,65,3000, {.maxSpeed = 40});


    // // low goal
	// chassis.setPose(15,-49.5,90); // starting pose
	// pros::delay(10);
    // chassis.moveToPoint(33,-48,3000);
    // chassis.turnToPoint(22,-22,3000);
    // bucket_intake();
    // chassis.moveToPoint(26,-24,3000, {.maxSpeed = 30});
    // chassis.moveToPoint(12, -12,3000, {.maxSpeed = 80});
    // chassis.moveToPoint(13,-13, 3000);
    // chassis.turnToPoint(0,0,3000);
    // stopbucket_intake();
    // pros::delay(1500);
    // low_intake();

	chassis.moveToPoint(50,-48, 3000);
	pros::delay(500);
	loader.set_value(true);
    chassis.turnToHeading(180,1000);
	chassis.moveToPoint(chassis.getPose().x,-70,1000, {.maxSpeed = 100});
	pros::delay(500);
	current = INTAKE;
	bucket_intake();
	pros::delay(2500);
	stopbucket_intake();
    chassis.moveToPoint(48,-48,3000);
    pros::delay(500);
    chassis.turnToPoint(48,-36,3000);
    chassis.moveToPoint(42,-43,3000);
    chassis.turnToPoint(48,-48,3000);
    high_intake();
    pros::delay(2000);
    chassis.moveToPoint(chassis.getPose().x, -72, 3000);
    pros::delay(1500);
    chassis.turnToPoint(24,-24, 3000);
    chassis.moveToPoint(24,-24,3000);
    bucket_intake();
    pros::delay(500);
    stopbucket_intake();
    chassis.moveToPoint(8,-8,3000);
    low_intake();
	// chassis.turnToPoint(24, -24, 3000);
	// chassis.moveToPoint(24,-24, 30000);
	//			
    // chassis.moveToPoint(20,48,7000);
}
   
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * 
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

// void opcontrol() {
	
// 	pros::Controller master(pros::E_CONTROLLER_MASTER);
// 	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
// 	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


// 	while (true) {
// 		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
// 		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
// 		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

// 		// Arcade control scheme
// 		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
// 		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
// 		left_mg.move(dir - turn);                      // Sets left motor voltage
// 		right_mg.move(dir + turn);                     // Sets right motor voltage
// 		pros::delay(20); 
		
// 		// Run for 20 ms then update

// 		pros
// 	}
// }

pros::Controller controller(pros::E_CONTROLLER_MASTER);



void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        // move the robot
        chassis.arcade(leftY  , leftX);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
            if  (current == STOP){
                current = INTAKE;
                bucket_intake();
            }
            else if (current == INTAKE){
                current = STOP;
                stopbucket_intake();
            }

        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
            if  (current == STOP){
                current = INTAKE;
                high_intake();
            }
            else if (current == INTAKE){
                current = STOP;
                stopbucket_intake();
            }

        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            if  (current == STOP){
                current = INTAKE;
                middle_intake();
            }
            else if (current == INTAKE){
                current = STOP;
                stopbucket_intake();
            }

        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
            if  (current == STOP){
                current = INTAKE;
                low_intake();
            }
            else if (current == INTAKE){
                current = STOP;
                stopbucket_intake();
            }

        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            loader.toggle();
        } 
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            descore.toggle();
        }
        // delay to save resources
        pros::delay(25);
        // if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        //     bottom_gintake.move(127);

        // }
        // else if (controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_B)) {
        //     bottom_gintake.move(0);
        // }

    }
}
//16.5,55