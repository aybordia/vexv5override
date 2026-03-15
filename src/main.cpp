#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include "pros/rtos.h"
#include <cmath>
#include <cstdio>
//cpp test 2
pros::MotorGroup left_motors({-11, -16, -20});
pros::MotorGroup right_motors({1, 6, 10});
pros::Motor double_intake(-6);
pros::Motor top_gintake(4);
pros::Motor top_fintake(11);
pros::Motor bottom_gintake(17);

void bucket_intake(){ double_intake.move(127); top_gintake.move(127); top_fintake.move(127); }
void high_intake(){ double_intake.move(127); top_gintake.move(127); top_fintake.move(-127); bottom_gintake.move(-127); }
void middle_intake(){ double_intake.move(127); top_gintake.move(-127); bottom_gintake.move(-127); }
void low_intake(){ double_intake.move(-127); bottom_gintake.move(-127); }
void stopbucket_intake(){ double_intake.move(0); top_gintake.move(0); top_fintake.move(0); bottom_gintake.move(0); }

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 12.5, lemlib::Omniwheel::OLD_325, 480, 2);

pros::Imu imu(5);
pros::adi::Pneumatics loader('h', false);
pros::adi::Pneumatics descore('g', false);

pros::Rotation vertical_sensor(10);
pros::Rotation horizontal_sensor(14);
pros::Distance distance_sensor(20);

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_sensor, lemlib::Omniwheel::NEW_275, -2.5);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, lemlib::Omniwheel::NEW_275, -2.5);

lemlib::OdomSensors sensors(nullptr, nullptr, &horizontal_tracking_wheel, nullptr, &imu);
lemlib::ControllerSettings lateral_controller(0.6,0,0.1,3,0.5,250,2.0,600,10);
lemlib::ControllerSettings angular_controller(0.8,1,3,3,1,200,2.0,600,0);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    chassis.setPose(15, -48, 90);
}

void disabled(){}
void competition_initialize(){}

enum intake_state { INTAKE, STOP };
intake_state current = STOP;


struct Pose2D { double x, y, theta; };
Pose2D pose = {15, -48, 90};

struct KalmanState { double estimate, covariance; };

KalmanState kfX = {pose.x,1};
KalmanState kfY = {pose.y,1};
KalmanState kfTheta = {pose.theta,1};


Pose2D poseExponentialUpdate(Pose2D current, double vx, double vy, double omega, double dt){

    double theta = current.theta * M_PI/180.0;
    double omegaRad = omega * M_PI/180.0;

    Pose2D newPose = current;

    if(fabs(omegaRad) < 1e-5){

        newPose.x += (vx*cos(theta) - vy*sin(theta)) * dt;
        newPose.y += (vx*sin(theta) + vy*cos(theta)) * dt;

    } else {

        double r_x = vx/omegaRad;
        double r_y = vy/omegaRad;

        newPose.x +=
        r_x*(sin(theta + omegaRad*dt) - sin(theta))
        - r_y*(cos(theta + omegaRad*dt) - cos(theta));

        newPose.y +=
        r_x*(cos(theta) - cos(theta + omegaRad*dt))
        - r_y*(sin(theta) - sin(theta + omegaRad*dt));
    }

    newPose.theta += omega*dt;

    return newPose;
}


double kalmanUpdate(KalmanState &state,double measurement,double R,double Q){

    state.covariance += Q;

    double K = state.covariance/(state.covariance + R);

    state.estimate += K*(measurement - state.estimate);

    state.covariance *= (1-K);

    return state.estimate;
}

struct Point { double x,y; };

Point bezier(Point p0,Point p1,Point p2,Point p3,double t){

    double u = 1 - t;
    double tt = t*t;
    double uu = u*u;

    double uuu = uu*u;
    double ttt = tt*t;

    Point p;

    p.x = uuu*p0.x;
    p.x += 3*uu*t*p1.x;
    p.x += 3*u*tt*p2.x;
    p.x += ttt*p3.x;

    p.y = uuu*p0.y;
    p.y += 3*uu*t*p1.y;
    p.y += 3*u*tt*p2.y;
    p.y += ttt*p3.y;

    return p;
}


#include "lemlib/api.hpp"
ASSET(example_txt);


void autonomous() {
    chassis.setPose(15, -48, 90);
    chassis.follow(example_txt, 15, 2000);
}


void opcontrol(){

    static double last_left = 0;
    static double last_right = 0;
    static double last_horiz = 0;
    static double last_heading = 0;
    static uint32_t last_time = pros::millis();

    while(true){

        uint32_t now = pros::millis();
        double dt = (now-last_time)/1000.0;

        if(dt<=0) dt=0.01;

        double left_in = left_motors.get_position()*M_PI*3.25/360.0;
        double right_in = right_motors.get_position()*M_PI*3.25/360.0;

        double vx = ((left_in-last_left)+(right_in-last_right))/(2.0*dt);

        double horiz_in = horizontal_sensor.get_position()*M_PI*2.75/360.0;
        double vy = (horiz_in-last_horiz)/dt;

        double heading = imu.get_heading();

        double delta_heading = heading-last_heading;

        if(delta_heading>180) delta_heading-=360;
        if(delta_heading<-180) delta_heading+=360;

        double omega = delta_heading/dt;

        last_left = left_in;
        last_right = right_in;
        last_horiz = horiz_in;
        last_heading = heading;
        last_time = now;

        pose = poseExponentialUpdate(pose,vx,vy,omega,dt);

        double measX = distance_sensor.get();
        double measY = pose.y;
        double measTheta = heading;

        pose.x = kalmanUpdate(kfX,measX,1.0,0.01);
        pose.y = kalmanUpdate(kfY,measY,1.0,0.01);
        pose.theta = kalmanUpdate(kfTheta,measTheta,0.5,0.01);

        chassis.setPose(pose.x,pose.y,pose.theta);


        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY,rightX);

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
            if(current==STOP){ current=INTAKE; bucket_intake(); }
            else{ current=STOP; stopbucket_intake(); }
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
            if(current==STOP){ current=INTAKE; high_intake(); }
            else{ current=STOP; stopbucket_intake(); }
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            if(current==STOP){ current=INTAKE; middle_intake(); }
            else{ current=STOP; stopbucket_intake(); }
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
            if(current==STOP){ current=INTAKE; low_intake(); }
            else{ current=STOP; stopbucket_intake(); }
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) loader.toggle();
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) descore.toggle();

        pros::delay(20);
    }
}