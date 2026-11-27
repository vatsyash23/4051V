#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({11, 13}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({-16, -20}, pros::MotorGearset::blue); 
pros::MotorGroup left_dt({11, 13, -12}); 
pros::MotorGroup right_dt({-16, -20, 19});  
pros::MotorGroup cascade ({2, -10}, pros::MotorGearset::green); 
pros::Motor toggle(9, pros::MotorGearset::green); 
pros::Distance LeftDistance(21); //port
pros::Distance RightDistance(20); //port
pros::Rotation cascadeRot(7); 
pros::Rotation vertodom(-5); 	
pros::Rotation horizodom(18); 
pros::Imu inertial(15); 
pros::adi::Pneumatics claw('A', false, false); 

lemlib::TrackingWheel vert_tracking_wheel(&vertodom, 2, 1.2);
lemlib::TrackingWheel horiz_tracking_wheel(&horizodom, 2, -1.7);
lemlib::OdomSensors odometry(&vert_tracking_wheel, nullptr, &horiz_tracking_wheel, nullptr, &inertial);
lemlib::Drivetrain drivetrain (&right_mg, &left_mg, 11.5, lemlib::Omniwheel::NEW_275, 450, 2);
lemlib::Drivetrain dt(&right_dt, &left_dt, 11.5, lemlib::Omniwheel::NEW_275, 450, 2);

lemlib::ControllerSettings straight_settings//tune
(
    15, // proportional gain (kP)
	0, // integral gain (kI)
    3, // derivative gain (kD)
    0, // anti windup
    1, // small error range, in inches
    250, // small error range timeout, in milliseconds
    2, // large error range, in inches
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

lemlib::ControllerSettings turn_settings //tune
(
    4.5,//kP 7
    0, //kI
    33, //kD  80
    3.63, //windup ranges
    1, //small error range, inches
    250, //small error range timeout, milliseconds
    2, //large error range, inches
    500, //large error range timeout, milliseconds
    0 //max acceleration (slew)
); 

lemlib::ExpoDriveCurve throttle_curve //tune
(
    12.7, //joystick deadband out of 127
    12.7, //min output out of 127
    1.02 //amount of curve
); 

lemlib::ExpoDriveCurve turn_curve //tune
(
    12.7, //joystick deadband out of 127
    19.05, //min output out of 127
    1.02 //amount of curve
); 

lemlib::Chassis chassis(drivetrain, straight_settings, turn_settings, odometry, &throttle_curve, &turn_curve); 
lemlib::Chassis dt_chassis(dt, straight_settings, turn_settings, odometry, &throttle_curve, &turn_curve);
void moveLift(double target, int timeout = 3000)
{
    double kP = 0.8;
    int startTime = pros::millis();

    while (pros::millis() - startTime < timeout)
    {
        double position = cascadeRot.get_position() / 100.0;
        double error = target - position;

        if (std::abs(error) <= 1.0)
        {
            break;
        }

        double speed = error * kP;

        if (speed > 200) speed = 200;
        if (speed < -200) speed = -200;

        cascade.move_velocity(speed);

        pros::delay(10);
    }

    cascade.move_velocity(0);
}

void auton()
{
    chassis.setPose(12, 0, 0);
    toggle.move_relative(750, 200);
    pros::delay(600);
    moveLift(600);
  //pros::delay(100);
    chassis.moveToPose(12, -12, 0, 1000, {.forwards = false});
    chassis.moveToPose(24, -24, 60, 1000, {.forwards = false});
    chassis.waitUntilDone();
    moveLift(10);
    claw.set_value(true);
    chassis.moveToPose(24, -24, 270, 1000, {.forwards = true});
}
//FUNCTIONS FOR AUTON AND DRIVER 
void initialize() 
{
    chassis.calibrate();
    pros::delay(3000);
    cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void disabled() {} 
void competition_initialize() {} 

void autonomous() 
{
    auton();
}

void opcontrol() 
{
    bool lowering = false;
     while (true)  
     {
         // Arcade Drive
         int leftY = -master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
         int rightX = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
         dt_chassis.arcade(leftY, rightX, false, 0.75);

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
        {
            lowering = true; 
        }
        // master.print(0, 0, "Dist: %d mm  ", distance.get());
        master.print(0, 0, "%.1f", inertial.get_heading());

         // Cascade Manual Controls
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
         {
            lowering = false; 
            cascade.move_velocity(200);
         }
         else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
         {
            lowering = false;
            cascade.move_velocity(-200);
         }
         else if(1)
         {
            if (lowering && cascadeRot.get_position() > 500)
            {
                cascade.move_velocity(-200);
            }
            else
            {
                cascade.move_velocity(0);
                lowering = false;
            }
         }
         // Toggle Manual Controls
         if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
         {
             toggle.move_velocity(200);
         }
         else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
         {
             toggle.move_velocity(-200);
         }
         else
         {
            toggle.move_velocity(0);
         }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
            {
                claw.toggle();
                cascade.move_relative(600, 200);
                pros::delay(90);
            }

        //1 toggle flip
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
            {
                toggle.move_relative(190, 200);
                pros::delay(100);
            }

        // 2 toggle flip    
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
            {
                toggle.move_relative(360, 200);
                pros::delay(100);
            }
        pros::delay(10);
     }
}