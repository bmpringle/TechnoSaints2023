#include "Robot.h"

#include <iostream>
#include <functional>
#include <chrono>

//CAN ids
#define FL_MOTOR_ID 2
#define FR_MOTOR_ID 0
#define BL_MOTOR_ID 3
#define BR_MOTOR_ID 1

//Motor Hardware Specifications
#define FL_MOTOR_TYPE TALON_FX
#define FR_MOTOR_TYPE TALON_FX
#define BL_MOTOR_TYPE TALON_FX
#define BR_MOTOR_TYPE TALON_FX

#define FL_MOTOR_BRUSHLESS true
#define FR_MOTOR_BRUSHLESS true
#define BL_MOTOR_BRUSHLESS true
#define BR_MOTOR_BRUSHLESS true

#define TEST_RIG false

//initalize member variables in the constructor and not in RobotInit because otherwise the compiler will complain since there's no default constructor for Motor and frc::XboxController
Robot::Robot() : frontLeft(Motor(FL_MOTOR_TYPE, FL_MOTOR_ID, FL_MOTOR_BRUSHLESS)), frontRight(Motor(FR_MOTOR_TYPE, FR_MOTOR_ID, FR_MOTOR_BRUSHLESS)), 
                    backLeft(Motor(BL_MOTOR_TYPE, BL_MOTOR_ID, BL_MOTOR_BRUSHLESS)), backRight(Motor(BR_MOTOR_TYPE, BR_MOTOR_ID, BR_MOTOR_BRUSHLESS)), 
                    controller(LogitechController(0)), cameraFunctionThread(std::thread(pds::cameraThreadFunction, 1)), gameField(pds::createField()), movementUpdateTimer(Timer()) {
     
     if(TEST_RIG) {
          frontRight.setIsReversed(true);
          backRight.setIsReversed(true);
     }else {
          frontLeft.setIsReversed(true);
          backLeft.setIsReversed(true);
     }  
}

Robot::~Robot() {
     pds::freeField(gameField);
}

void Robot::TeleopInit() {
     std::cout << "Teleop Init Complete!" << std::endl;

     //std::function<void()> movementPeriodicFunction = std::bind(&Robot::teleopMovementPeriodic, this);
     //this->AddPeriodic(movementPeriodicFunction, std::chrono::duration<double>(teleopMovementPeriodicCallRate));
     movementUpdateTimer.startTimer("movementPeriodicTimer");
}

int double_sign_function(double x) {
     return -(x < 0) + (x > 0);
}

void Robot::teleopMovementPeriodic() {
     if(controller.GetLeftTriggerAxis() > 0) { //turn left if left trigger is being used
          frontLeft.setMotorPower(-controller.GetLeftTriggerAxis() * maxTurnSpeed);
          frontRight.setMotorPower(controller.GetLeftTriggerAxis() * maxTurnSpeed);
          backLeft.setMotorPower(-controller.GetLeftTriggerAxis() * maxTurnSpeed);
          backRight.setMotorPower(controller.GetLeftTriggerAxis() * maxTurnSpeed);
          return;
     }

     if(controller.GetRightTriggerAxis() > 0) { //turn right if right trigger is being used
          frontLeft.setMotorPower(controller.GetRightTriggerAxis() * maxTurnSpeed);
          frontRight.setMotorPower(-controller.GetRightTriggerAxis() * maxTurnSpeed);
          backLeft.setMotorPower(controller.GetRightTriggerAxis() * maxTurnSpeed);
          backRight.setMotorPower(-controller.GetRightTriggerAxis() * maxTurnSpeed);
          return;
     }

     /*if((controller.GetRightTriggerAxis() > 0) && (controller.GetLeftTriggerAxis() > 0)) { //if either both or neither triggers are being used, set all power to 0
          frontLeft.setMotorPower(0);
          frontRight.setMotorPower(0);
          backLeft.setMotorPower(0);
          backRight.setMotorPower(0);
     }*/

     double timeSinceLastCallMilliseconds = movementUpdateTimer.resetTimer("movementPeriodicTimer");

     std::cout << controller.GetLeftY() << std::endl;

     //combine the turn and straight movement powers to allow for both to happen at the same time (positive straight movement)
     if(controller.GetLeftY() > 0.025) {  //0.05 is a dead zone, used both in order to both prevent accidental movement and to keep a left stick that reports a resting value that is slightly greater than 0 from constantly moving the robot forwards
          double powerDiff = std::min(1.0 - frontLeft.getMotorPower(), std::min(1.0 - frontRight.getMotorPower(), std::min(1.0 - backLeft.getMotorPower(), std::min(1.0 - backRight.getMotorPower(), controller.GetLeftY()))));
          
          //take minimum magnitude between previously calculated powerDiff and max acceleration per second * elapsed time in seconds
          powerDiff = double_sign_function(powerDiff) * std::min(std::abs(powerDiff), maxAcceleratePerSecond * timeSinceLastCallMilliseconds / 1000.0);
          frontLeft.setMotorPower(frontLeft.getMotorPower() + powerDiff);
          frontRight.setMotorPower(frontRight.getMotorPower() + powerDiff);
          backLeft.setMotorPower(backLeft.getMotorPower() + powerDiff);
          backRight.setMotorPower(backRight.getMotorPower() + powerDiff);
     }else if(controller.GetLeftY() < -0.025) { //0.05 is a dead zone, used both in order to both prevent accidental movement and to keep a left stick that reports a resting value that is slightly less than 0 from constantly moving the robot backwards
          double powerDiff = -std::min(1.0 + frontLeft.getMotorPower(), std::min(1.0 + frontRight.getMotorPower(), std::min(1.0 + backLeft.getMotorPower(), std::min(1.0 + backRight.getMotorPower(), -controller.GetLeftY()))));
          
          //take minimum magnitude between previously calculated powerDiff and max acceleration per second * elapsed time in seconds
          powerDiff = double_sign_function(powerDiff) * std::min(std::abs(powerDiff), maxAcceleratePerSecond * timeSinceLastCallMilliseconds / 1000.0);

          frontLeft.setMotorPower(frontLeft.getMotorPower() + powerDiff);
          frontRight.setMotorPower(frontRight.getMotorPower() + powerDiff);
          backLeft.setMotorPower(backLeft.getMotorPower() + powerDiff);
          backRight.setMotorPower(backRight.getMotorPower() + powerDiff);
     }else {
          //double powerDiff = std::min()
          frontLeft.setMotorPower(0);
          frontRight.setMotorPower(0);
          backLeft.setMotorPower(0);
          backRight.setMotorPower(0);
     }
}

void Robot::TeleopExit() {
     frontLeft.setMotorPower(0);
     frontRight.setMotorPower(0);
     backLeft.setMotorPower(0);
     backRight.setMotorPower(0);

     std::cout << "Teleop Exit Complete!" << std::endl;
}

void Robot::AutonomousInit() {
     std::cout << "Autonomous Init Complete!" << std::endl;
}

void Robot::AutonomousPeriodic() {

}

void Robot::AutonomousExit() {
     std::cout << "Autonomous Exit Complete!" << std::endl;
}

void Robot::RobotInit() {
     std::cout << "Robot Init Complete!" << std::endl;
}

void Robot::RobotPeriodic() {
     
}

void Robot::TeleopPeriodic() {
     teleopMovementPeriodic();
}

void Robot::DisabledInit() {

}

void Robot::DisabledPeriodic() {

}

void Robot::DisabledExit() {

}

#ifndef RUNNING_FRC_TESTS
int main() {
     return frc::StartRobot<Robot>();
}
#endif
