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


#define GTU_TURN_PRECISE DoubleBoundedPID({1.25, 0.005, 0.13, 0.17, 1}, getRadians(0.5), 3)
bool isSkills = false;
Robot robot = getRobot15(isSkills);

FlywheelDriver driver(robot, TANK_DRIVE, 2500);



using namespace pros;


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
    robot.localizer->init();

    
	

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

    while(true){
        
    }
    
   //run driver control
   driver.runDriver();
	
}
