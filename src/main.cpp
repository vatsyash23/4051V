//INCLUDE FILES - ANY NECESSARY FILES FOR THE PROGRAM TO RUN
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

//VARIABLES 
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup cascade ({11, -20}, pros::MotorGearset::blue); //sign tune
pros::MotorGroup left_mg({-12, -13, -14}); //sign tune
pros::MotorGroup right_mg({17, 18, 19}); //sign tune 
pros::Motor intake(15, pros::MotorGearset::blue); //port tune
pros::adi::Pneumatics wrist('A', false); //port tune
pros::adi::Pneumatics claw('B', false); //port tune
pros::Distance distance(9); //port tune
pros::Imu inertial(1); 
pros::Rotation vertodom(3); 
pros::Rotation horizodom(2); //sign tune
lemlib::TrackingWheel vert_tracking_wheel(&vertodom, 2, -0.5); //tune dimensions
lemlib::TrackingWheel horiz_tracking_wheel(&horizodom, 2, -0.5);//tune dimensions 
lemlib::Drivetrain drivetrain (&left_mg, &right_mg, 11.25, lemlib::Omniwheel::NEW_275, 450, 2);
lemlib::OdomSensors odometry(nullptr, nullptr, nullptr, nullptr, &inertial);
lemlib::ControllerSettings straight_settings//tune
(
	5, //kP
	0, //kI
	1, //kD
	0, //windup range
	1, //small error range, inches
	100, //small error range timeout, milliseconds
	3, //large error range, inches
	500, //large error range timeout, milliseconds
	3 //max acceleration (slew)
); 
lemlib::ControllerSettings turn_settings //tune
(
	5.93, //kP - 5.8 - 5.93 
	0, //kI
	44.9, //kD - 41.92 - 42.46/42.5/43.9/44.87(small oscillate but APPROACH 44.9)
	0, //windup range
	0, //small error range, inches
	0, //small error range timeout, milliseconds
	0, //large error range, inches
	0, //large error range timeout, milliseconds
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

//AUTON FUNCTIONS
bool cascadeResetting = false;	
void resetCascade()
{
	cascadeResetting = true;
	cascade.move_velocity(-300);
	while(distance.get() > 125)
	{
		pros::delay(10);
	}
	cascade.move_velocity(0);
	cascadeResetting = false;
}
void resetCascadeTask(void*)
{
	resetCascade();
}

void ninety_test(){
	chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 5000);
}
void turnPIDtest()
{
	pros::delay(2500);
	chassis.setPose(0, 0, 0);
	pros::delay(2500);
    chassis.turnToHeading(45, 5000);
	pros::delay(2500);
	chassis.turnToHeading(135, 5000);
	pros::delay(2500);
	chassis.turnToHeading(90, 5000);
	pros::delay(2500);
	chassis.turnToHeading(180,5000);
	pros::delay(2500);
	chassis.turnToHeading(0, 5000);

} 
void PIDtask(void*)
{
	turnPIDtest();
}
//FUNCTIONS FOR AUTON AND DRIVER 
void initialize() 
{
	chassis.calibrate();
}
void disabled() {} //runs while robot is disabled after auton or opcontrol. ends when robot is enabled
void competition_initialize() {} //runs after initialize, b4 auton. ends when robot is enabled. 
//intended for comp-specific initialization routines, like an auton selector.
void autonomous() //runs auton code. if disabled and restarted, the task will restart from the beginning, not resume.
{
	ninety_test();
}
void opcontrol() ////runs driver code. if disabled and restarted, the task will restart from the beginning, not resume.
//if no comp control: will run immediately after initialize. 
{
	pros::Controller controller(pros::E_CONTROLLER_MASTER);
    while (true) 
	{
        // get left y and right x positions
        int leftY = (-controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot 
        chassis.arcade(leftY, rightX,false, 0.5);

   		master.print(0, 0, "%.1f", inertial.get_heading()); 
	

        if(!cascadeResetting)
		{
			if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
			{
				cascade.move_velocity(400);
			}
			else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
			{
				cascade.move_velocity(-400);
			} 
			else
			{
				cascade.move_velocity(0);
			}
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			intake.move_velocity(600);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			intake.move_velocity(-600);
		}
		else
		{
			intake.move_velocity(0);
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
		{
			wrist.toggle();
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
		{
			claw.toggle();
		}
		//MACROS
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
    		pros::Task PIDTask(PIDtask);

		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
    		if(!cascadeResetting)
			{
				pros::Task resetTask(resetCascadeTask);
			}
		}
		pros::delay(10);
	}

}
