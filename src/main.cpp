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
// pros::Rotation cascadeRot(13); //port
pros::Rotation vertodom(8); 	
pros::Rotation horizodom(18); 
pros::Imu inertial(15); 
pros::adi::Pneumatics claw('A', false, false); //port

lemlib::TrackingWheel vert_tracking_wheel(&vertodom, 2, 1.2);
lemlib::TrackingWheel horiz_tracking_wheel(&horizodom, 2, -1.7);
lemlib::OdomSensors odometry(&vert_tracking_wheel, nullptr, &horiz_tracking_wheel, nullptr, &inertial);
lemlib::Drivetrain drivetrain (&right_mg, &left_mg, 11.5, lemlib::Omniwheel::NEW_275, 450, 2);
lemlib::Drivetrain dt(&right_dt, &left_dt, 11.5, lemlib::Omniwheel::NEW_275, 450, 2);

lemlib::ControllerSettings straight_settings//tune
(
    12.5, // proportional gain (kP)
	0, // integral gain (kI)
    7.5, // derivative gain (kD)
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

void auton()
{
    double test_angles[] = {10.0, 45.0, 90.0, 180.0};
    int num_tests = 4;
    double total_error = 0.0;

    for (int i = 0; i < num_tests; i++) {
        // Execute the turning motion
        chassis.turnToHeading(test_angles[i], 1750);
        chassis.waitUntilDone();
        
        // Wait briefly for the robot to fully settle, then measure final heading error
        pros::delay(500);
        double final_heading = chassis.getPose().theta;
        double current_error = std::abs(test_angles[i] - final_heading);
        total_error += current_error;

        // Return to 0 heading to prepare for the next test motion
        chassis.turnToHeading(0, 1750);
        chassis.waitUntilDone();
        pros::delay(500);
    }

    // Step 3: Calculate average steady-state error and increase it by 50%
    double avg_steady_state_error = total_error / num_tests;
    double anti_windup_range = avg_steady_state_error * 1.5;

    // Step 4: Print the final anti-windup range to the brain screen

    master.print(0, 0, "%.2f", anti_windup_range);
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
    auton();
    //  while (true)  
    //  {
    //      // Arcade Drive
    //      int leftY = -master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    //      int rightX = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    //      dt_chassis.arcade(leftY, rightX, false, 0.75);

    
    //     // master.print(0, 0, "Dist: %d mm  ", distance.get());
    //     master.print(0, 0, "%.1f", inertial.get_heading());

    //      // Cascade Manual Controls
    //     if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    //      {
    //          cascade.move_velocity(200);
    //      }
    //      else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    //      {
    //          cascade.move_velocity(-200);
    //      }
    //      else
    //      {
    //          cascade.move_velocity(0);
    //      }
    //      // Toggle Manual Controls
    //      if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    //      {
    //          toggle.move_velocity(200);
    //      }
    //      else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    //      {
    //          toggle.move_velocity(-200);
    //      }
    //      else
    //      {
    //         toggle.move_velocity(0);
    //      }
    //      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
    //         {
    //             claw.toggle();
    //         }
    //     pros::delay(10);
    //  }
}