//INCLUDE FILES - ANY NECESSARY FILES FOR THE PROGRAM TO RUN
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

//VARIABLES 
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup cascade ({16, -20}, pros::MotorGearset::blue); //sign tune
pros::Motor intake(15, pros::MotorGearset::blue); //port tune
pros::adi::Pneumatics wrist('A', false); //port tune
pros::adi::Pneumatics claw('B', false); //port tune
pros::Distance distance(10); //port tune
pros::MotorGroup left_mg({12, 14}); //sign tune
pros::MotorGroup right_mg({-5, -18}); //sign tune 
pros::MotorGroup left_dt({-12, -14, -13}); //sign tune
pros::MotorGroup right_dt({5, 18, 19}); //sign tune 
lemlib::Drivetrain drivetrain (&right_mg, &left_mg, 11.25, lemlib::Omniwheel::NEW_275, 450, 2);
lemlib::Drivetrain dt(&left_dt, &right_dt, 11.25, lemlib::Omniwheel::NEW_275, 450, 2);
pros::Imu inertial(1); 
pros::Rotation vertodom(-3); 	
pros::Rotation horizodom(2); //sign tune
lemlib::TrackingWheel vert_tracking_wheel(&vertodom, 2, 0); //tune dimensions
lemlib::TrackingWheel horiz_tracking_wheel(&horizodom, 2, 0);//tune dimensions 
lemlib::OdomSensors odometry(&vert_tracking_wheel, nullptr, &horiz_tracking_wheel, nullptr, &inertial);


lemlib::ControllerSettings straight_settings//tune
(
    12.5, // proportional gain (kP)
	0, // integral gain (kI)
    7.5, // derivative gain (kD)
    0, // anti windup
    0, // small error range, in inches
    0, // small error range timeout, in milliseconds
    0, // large error range, in inches
    0, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

lemlib::ControllerSettings turn_settings //tune
(
    3.5,//kP 7
    0, //kI
    20, //kD  80
    0, //windup range
    1, //small error range, inches
    250, //small error range timeout, milliseconds
    3, //large error range, inches
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
//AUTON FUNCTION
bool cascadeMacroActive = false;
bool intakeMacroActive = false;
void intakeMacro()
{
	int startTime2 = pros::millis();
	wrist.extend();
	intakeMacroActive = true;
	cascade.move_velocity(-600);
    while(true)
    {
        int cascadePos = distance.get();
        // 1. Target Condition: Adjust <= or >= depending on physical mounting
        if(cascadePos > 250) 
        {
            break;
        }
        // 2. Safety Timeout: Stop after 3 seconds no matter what
        if(pros::millis() - startTime2 > 3000) 
        {
			// master.print(0, 0, "timeout");
			break;
		}	
	}
	cascade.move_velocity(0);
	wrist.retract();
	intakeMacroActive = false;   
}

void intakeMacroTask(void*)
{
    intakeMacro();
}   

void resetCascade()
{
    cascadeMacroActive = true;
	int startTime = pros::millis();
	cascade.move_velocity(600);
    while(true)
    {
        int cascadePos = distance.get();
        // 1. Target Condition: Adjust <= or >= depending on physical mounting
        if(cascadePos <= 175) 
        {
            break;
        }
        // 2. Safety Timeout: Stop after 3 seconds no matter what
        if(pros::millis() - startTime > 3000) 
        {
			// master.print(0, 0, "timeout");
			break;
		}	
	}
	cascade.move_velocity(0);
	cascadeMacroActive = false;   
	master.clear_line(0);	
}

void resetCascadeTask(void*)
{
    resetCascade();
}

void ninety_test(){
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 5000);
}

void turnPIDtest()
{
    inertial.reset();
    pros::delay(2500);
    chassis.setPose(0, 0, 0);
    pros::delay(2500);
    chassis.turnToHeading(45, 5000);
    pros::lcd::print(1, "Heading: %.1f", inertial.get_heading());
    pros::delay(2500);
    chassis.turnToHeading(120, 5000);
    pros::lcd::print(1, "Heading: %.1f", inertial.get_heading());
    pros::delay(2500);
    chassis.turnToHeading(90, 5000);
    pros::lcd::print(1, "Heading: %.1f", inertial.get_heading());
    pros::delay(2500);
    chassis.turnToHeading(180,5000);
    pros::lcd::print(1, "Heading: %.1f", inertial.get_heading());
    pros::delay(2500);
    chassis.turnToHeading(15, 5000);
    pros::lcd::print(1, "Heading: %.1f", inertial.get_heading());
} 

void PIDtask(void*)
{
    turnPIDtest();
}
void twopin()
{
    chassis.setPose(-61, 0, 270);
    chassis.moveToPoint(-60, 0, 250, {.forwards=false, .minSpeed = 90});
    chassis.waitUntilDone();
    chassis.moveToPoint(-64, 0, 750, {.minSpeed = 120});
    chassis.waitUntilDone();
    chassis.setPose(-63, 0, 270);
    chassis.moveToPoint(-59, 0, 250, {.forwards=false, .minSpeed = 90});
    chassis.waitUntilDone();
    chassis.moveToPoint(-64, 0, 500, {.minSpeed = 90});
    chassis.waitUntilDone();
    chassis.setPose(-63, 0, 270);
    chassis.moveToPoint(-46, 0, 1000, {.forwards=false});
    wrist.extend();
    cascade.move_relative(1500, 600);
    chassis.turnToHeading(180, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-46, 11, 625, {.forwards=false});
    chassis.waitUntilDone();
    chassis.moveToPoint(-46, 20, 1000, {.forwards=false, .maxSpeed = 90});
    chassis.waitUntilDone();
    cascade.move_relative(-1800, 600);
    pros::delay(500);
    claw.extend(); 
    // chassis.setPose(-45, 17, 180);
    pros::delay(250);
    chassis.moveToPoint(-46, 4, 1000);
    chassis.waitUntilDone(); 
}
void twopinright()
{
    chassis.setPose(-61, 0, 270);
    chassis.moveToPoint(-60, 0, 250, {.forwards=false, .minSpeed = 90});
    chassis.waitUntilDone();
    chassis.moveToPoint(-64, 0, 750, {.minSpeed = 120});
    chassis.waitUntilDone();
    chassis.setPose(-63, 0, 270);
    chassis.moveToPoint(-59, 0, 250, {.forwards=false, .minSpeed = 90});
    chassis.waitUntilDone();
    chassis.moveToPoint(-64, 0, 500, {.minSpeed = 90});
    chassis.waitUntilDone();
    chassis.setPose(-63, 0, 270);
    chassis.moveToPoint(-46, 0, 1000, {.forwards=false});
    wrist.extend();
    cascade.move_relative(1500, 600);
    chassis.turnToHeading(0, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-46, -11, 625, {.forwards=false});
    chassis.waitUntilDone();
    chassis.moveToPoint(-46, -20, 1000, {.forwards=false, .maxSpeed = 90});
    chassis.waitUntilDone();
    cascade.move_relative(-1800, 600);
    pros::delay(500);
    claw.extend(); 
    // chassis.setPose(-45, 17, 180);
    pros::delay(250);
    chassis.moveToPoint(-46, -4, 1000);
    chassis.waitUntilDone(); 
}
void threepin()
{
    chassis.setPose(-61, 0, 270);
    chassis.moveToPoint(-60, 0, 250, {.forwards=false, .minSpeed = 90});
    chassis.waitUntilDone();
    chassis.moveToPoint(-64, 0, 750, {.minSpeed = 127});
    chassis.waitUntilDone();
    chassis.setPose(-63, 0, 270);
    chassis.moveToPoint(-59, 0, 250, {.forwards=false, .minSpeed = 90});
    chassis.waitUntilDone();
    chassis.moveToPoint(-64, 0, 1000, {.minSpeed = 127});
    chassis.waitUntilDone();
    chassis.setPose(-63, 0, 270);
    chassis.moveToPoint(-46, 0, 1000, {.forwards=false});
    wrist.extend();
    cascade.move_relative(1500, 600);
    chassis.turnToHeading(180, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-46, 12, 625, {.forwards=false});
    chassis.waitUntilDone();
    chassis.moveToPoint(-46, 18, 875, {.forwards=false, .maxSpeed = 90});
    chassis.waitUntilDone();
    cascade.move_relative(-2250, 600);
    pros::delay(900);
    claw.extend(); 
    // chassis.setPose(-45, 17, 180);
    pros::delay(100);
    chassis.moveToPoint(-46, 5, 1000);
    chassis.waitUntilDone(); 
    chassis.turnToHeading(227, 1000); 
    chassis.waitUntilDone();
    chassis.moveToPoint(-34, 14, 750, {.forwards=false});
    chassis.waitUntilDone();    
    chassis.moveToPoint(-30, 20, 1750, {.forwards=false, .maxSpeed = 45});
    chassis.waitUntilDone();
    claw.retract();
    cascade.move_relative(3000, 600);
    pros::delay(500); 
    chassis.swingToHeading(18, lemlib::DriveSide::RIGHT, 1750, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed=60});
    chassis.waitUntilDone();
    chassis.moveToPoint(-49, 23, 1000, {.forwards=false});
    chassis.waitUntilDone();
    cascade.move_relative(-2100, 600);
    pros::delay(500);
    claw.extend();
    pros::delay(250);
    chassis.moveToPoint(-46, 18, 1000);
    chassis.waitUntilDone();  
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-52, 21, 1000, {.forwards=false}); 
    // chassis.waitUntilDone();  
    // pros::delay(250);
    // chassis.turnToHeading(130, 750);
    // chassis.waitUntilDone(); 
    // chassis.moveToPoint(-62, 29, 1000, {.forwards=false, .maxSpeed = 60});   
    // chassis.waitUntilDone();
    // chassis.swingToHeading(90,lemlib::DriveSide::RIGHT, 1000, {.minSpeed=60});
    // chassis.waitUntilDone();    
    // claw.retract();
    // cascade.move_relative(1800, 600);
    // chassis.turnToHeading(180, 1000);
    // chassis.waitUntilDone();   
    // cascade.move_velocity(0);
    master.print(0, 0, "%d,%d,%d", (int)std::round(chassis.getPose().x), (int)std::round(chassis.getPose().y), (int)std::round(chassis.getPose().theta));    // chassis.moveToPoint(-61, 20, 1000, {.forwards=false, .minSpeed =60});
    // chassis.waitUntilDone();
    // pros::delay(250);
    // claw.retract();

}
void simpleguesscheckauton()
{
	chassis.setPose(0, 0, 0);
	chassis.moveToPoint(0, -3, 750, {.forwards = false});
	chassis.waitUntilDone();
	chassis.moveToPoint(0, 6, 1250, {.minSpeed = 127});
	chassis.waitUntilDone();
	chassis.moveToPoint(0, -3, 1000, {.forwards = false});
	chassis.waitUntilDone();
	chassis.moveToPoint(0, 9, 1500, {.minSpeed = 75});
	wrist.extend();
	cascade.move_velocity(-300);
	chassis.moveToPoint(0, -6, 500, {.forwards = false});
	cascade.move_velocity(0);
	chassis.moveToPoint(0, -7, 800, {.forwards = false});
	chassis.waitUntilDone();
	dt_chassis.setPose(chassis.getPose());
	dt_chassis.turnToHeading(90, 1000);
	dt_chassis.waitUntilDone();
	chassis.setPose(dt_chassis.getPose().x, dt_chassis.getPose().y, dt_chassis.getPose().theta);
	chassis.arcade(-95, 0);
	pros::delay(725);
	chassis.arcade(0,0);
	cascade.move_velocity(500);
    pros::delay(800);
	cascade.move_velocity(0);
	claw.extend();

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
    threepin();
}

void opcontrol() 
{
    // while   (1)
    // {
    //     ninety_test();
    //     pros::delay(5000);
    // }
     while (true)  
     {
         // Arcade Drive
         int leftY = -master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
         int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
         dt_chassis.arcade(leftY, rightX, false, 0.5);

    
        // master.print(0, 0, "Dist: %d mm  ", distance.get());
        master.print(0, 0, "%.1f", inertial.get_heading());

         // Cascade Manual Controls
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
         {
             cascade.move_velocity(600);
         }
         else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
         {
             cascade.move_velocity(-600);
         }
	 	else if(!cascadeMacroActive && !intakeMacroActive)
	 	{
	 		cascade.move_velocity(0);
	 	}
        
         // Intake Controls
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

         // Pneumatics Toggles
         if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
            {
                wrist.toggle();
            }
         if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
            {
                claw.toggle();
            }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
            {
              pros::Task intakeTask(intakeMacroTask);
            }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
            {
              pros::Task resetTask(resetCascadeTask);
            }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
            {
             cascade.move_velocity(0);
            }

        pros::delay(10);
     }
}