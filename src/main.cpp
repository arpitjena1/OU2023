#include "main.h"

#include "AutonomousFunctions/DriveFunctions.h"
#include "Subsystems/RobotBuilder.h"
#include "Programs/Driver.h"
#include "Programs/CompetitionDriver.h"
#include "Programs/FlywheelDriver.h"
#include "Programs/CataDriver.h"
#include "Programs/TuningDriver.h"
#include "Programs/Autonomous.h"
#include "TuneFlywheel.h"
#include "Programs/TestFunction/TurnTest.h"
#include "Programs/TestFunction/ForwardTest.h"
#include "pros/vision.hpp"

#define IS_FIFTEEN // uncomment for 15, comment for 18
#define GTU_TURN_PRECISE DoubleBoundedPID({1.25, 0.005, 0.13, 0.17, 1}, getRadians(0.5), 3)
bool isSkills = false;
//#define TEST_TUNE_PID // uncomment to adjust pid using TuningDriver class. Should comment out RUN_AUTON

#ifdef IS_FIFTEEN
    #define IS_THREE_TILE
    Robot robot = getRobot15(isSkills);
#else
    #define IS_TWO_TILE
    Robot robot = getRobot18(isSkills);
#endif

#ifdef IS_FIFTEEN
FlywheelDriver driver(robot, TANK_DRIVE, 2500);
#else
CataDriver driver(robot, TANK_DRIVE);
#endif

//#define RUN_TEST
//#define RUN_AUTON // uncomment to run auton, comment to run teleop / actual comp
//#define TUNE_FLYWHEEL // uncomment to run flywheel tuning program intsead, comment to disable this

using namespace pros;

bool centerButtonReady = false;

void ready() {
    centerButtonReady = true;
}

void lowerCata() {

    // no cata to lower
    if (!robot.cata) return;

    shootCataNonblocking(robot);
}
pros::Vision vis(20);
void VisionAim() {
  // Sets zero point to center of FOV
  vis.set_zero_point(pros::E_VISION_ZERO_CENTER);

  // Gets largest object, filters if it is the right sig
  auto object = vis.get_by_size(0);
  if (object.signature == 2) {

    // Find angle of goal compared to robot
    int center_x = object.left_coord + (object.width / 2) - (VISION_FOV_WIDTH / 2);
    float center_x_percentage = center_x / (VISION_FOV_WIDTH / 2.0f);
    float direction_radian = std::atan(center_x_percentage * std::tan(61 / 2.0f / 360 * (23.14)));
    float offdegrees = direction_radian/(23.14)/ 360;

    // Reset chassis to zero to do a relative turn
    

    // Set LED to look cool
    vis.set_led(COLOR_WHITE);

    goTurnU(robot, GTU_TURN_PRECISE, getRadians(offdegrees));


    // Update object postion after aimed
    center_x = object.left_coord + (object.width / 2) - (VISION_FOV_WIDTH / 2);
    center_x_percentage = center_x / (VISION_FOV_WIDTH / 2.0f);
    direction_radian = std::atan2(center_x_percentage, std::tan(61 / 2.0f / 360 * (23.14)));
    offdegrees = direction_radian/(23.14) *360;
    int absdeg = (fabs(offdegrees));
    double error = 3;

    //If goal is in center, give driver some input
    if (absdeg < error) {
    vis.set_led(COLOR_PURPLE);
    pros::delay(200);
    }
  }

  pros::delay(20);
  vis.set_led(COLOR_RED);
}
void initialize() {

    pros::lcd::initialize();
    pros::lcd::register_btn1_cb (ready);
    pros::lcd::register_btn0_cb(lowerCata);

    
    if (robot.shooterFlap) robot.shooterFlap->set_value(true); // start flap up

	robot.localizer->init();

    #ifdef RUN_AUTON
    #ifndef TUNE_FLYWHEEL
    while (!centerButtonReady) {

        pros::lcd::print(3, "Heading (deg): %f", robot.localizer->getHeading() * 180 / 3.1415);
        
        pros::delay(10);
    }
    #endif
    #endif

}

  
void disabled() {}


void competition_initialize() {}


void autonomous() {  


    if (true && robot.localizer) {
        pros::Task taskOdometry([&] {
            robot.localizer->updatePositionTask();
        });
    }

    try {

       testAuton(robot);

    } catch (std::runtime_error &e) {
        pros::lcd::clear();
        pros::lcd::print(0, "IMU disconnect, force shutdown.");
        robot.drive->stop();
        robot.intake->brake();
    }

    
}




void opcontrol() {

    #ifdef TUNE_FLYWHEEL
    tuneFlywheel(robot, driver.controller);
    return;
    #endif



	#ifdef RUN_AUTON
	autonomous();
	return;
	#endif
    pros::Motor motor1(1);
    pros::Motor motor2(2);
    pros::Motor motor3(3);
    pros::ADIDigitalOut puncher1('A');
    pros::ADIDigitalOut puncher2('B');
    while(true){
        //puncher1.set_value(true) && puncher2.set_value(true);
        //puncher1.set_value(false) && puncher2.set_value(false);
       motor1.move_voltage(-12000);
       motor2.move_voltage(12000);
       motor3.move_voltage(3000);
    }
    

	//driver.runDriver();
	
}
