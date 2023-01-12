#include "Robot.h"

#include <iostream>

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

//initalize member variables in the constructor and not in RobotInit because otherwise the compiler will complain since there's no default constructor for Motor and frc::XboxController
Robot::Robot() : frontLeft(Motor(FL_MOTOR_TYPE, FL_MOTOR_ID, FL_MOTOR_BRUSHLESS)), frontRight(Motor(FR_MOTOR_TYPE, FR_MOTOR_ID, FR_MOTOR_BRUSHLESS)), 
                    backLeft(Motor(BL_MOTOR_TYPE, BL_MOTOR_ID, BL_MOTOR_BRUSHLESS)), backRight(Motor(BR_MOTOR_TYPE, BR_MOTOR_ID, BR_MOTOR_BRUSHLESS)), controller(frc::XboxController(0)) {

}

void Robot::RobotInit() {
     std::cout << "Robot Init Complete!" << std::endl;
}

void Robot::RobotPeriodic() {

}

void Robot::TeleopInit() {
     std::cout << "Teleop Init Complete!" << std::endl;
}

void Robot::TeleopPeriodic() {

     if(controller.GetLeftTriggerAxis()) { //turn left if left trigger is being used
          frontLeft.setMotorPower(-controller.GetLeftTriggerAxis() * maxTurnSpeed);
          frontRight.setMotorPower(controller.GetLeftTriggerAxis() * maxTurnSpeed);
          backLeft.setMotorPower(-controller.GetLeftTriggerAxis() * maxTurnSpeed);
          backRight.setMotorPower(controller.GetLeftTriggerAxis() * maxTurnSpeed);
     }

     if(controller.GetRightTriggerAxis()) { //turn right if right trigger is being used
          frontLeft.setMotorPower(-controller.GetLeftTriggerAxis() * maxTurnSpeed);
          frontRight.setMotorPower(controller.GetLeftTriggerAxis() * maxTurnSpeed);
          backLeft.setMotorPower(-controller.GetLeftTriggerAxis() * maxTurnSpeed);
          backRight.setMotorPower(controller.GetLeftTriggerAxis() * maxTurnSpeed);
     }

     if((controller.GetRightTriggerAxis() > 0) == (controller.GetLeftTriggerAxis() > 0)) { //if either both or neither triggers are being used, set all power to 0
          frontLeft.setMotorPower(0);
          frontRight.setMotorPower(0);
          backLeft.setMotorPower(0);
          backRight.setMotorPower(0);
     }

     //combine the turn and straight movement powers to allow for both to happen at the same time (positive straight movement)
     if(controller.GetLeftY() > 0.05) {  //0.05 is a dead zone, used both in order to both prevent accidental movement and to keep a left stick that reports a resting value that is slightly greater than 0 from constantly moving the robot forwards
          double powerDiff = std::min(1.0 - frontLeft.getMotorPower(), std::min(1.0 - frontRight.getMotorPower(), std::min(1.0 - backLeft.getMotorPower(), std::min(1.0 - backRight.getMotorPower(), controller.GetLeftY()))));
          
          frontLeft.setMotorPower(frontLeft.getMotorPower() + powerDiff);
          frontRight.setMotorPower(frontRight.getMotorPower() + powerDiff);
          backLeft.setMotorPower(backLeft.getMotorPower() + powerDiff);
          backRight.setMotorPower(backRight.getMotorPower() + powerDiff);
     } 

     //combine the turn and straight movement powers to allow for both to happen at the same time (negative straight movement)
     if(controller.GetLeftY() < -0.05) { //0.05 is a dead zone, used both in order to both prevent accidental movement and to keep a left stick that reports a resting value that is slightly less than 0 from constantly moving the robot backwards
          double powerDiff = -std::min(1.0 + frontLeft.getMotorPower(), std::min(1.0 + frontRight.getMotorPower(), std::min(1.0 + backLeft.getMotorPower(), std::min(1.0 + backRight.getMotorPower(), -controller.GetLeftY()))));
          
          frontLeft.setMotorPower(frontLeft.getMotorPower() + powerDiff);
          frontRight.setMotorPower(frontRight.getMotorPower() + powerDiff);
          backLeft.setMotorPower(backLeft.getMotorPower() + powerDiff);
          backRight.setMotorPower(backRight.getMotorPower() + powerDiff);
     } 
}

void Robot::AutonomousInit() {
     std::cout << "Autonomous Init Complete!" << std::endl;
}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopExit() {
     frontLeft.setMotorPower(0);
     frontRight.setMotorPower(0);
     backLeft.setMotorPower(0);
     backRight.setMotorPower(0);

     std::cout << "Teleop Exit Complete!" << std::endl;
}

void Robot::AutonomousExit() {
     std::cout << "Autonomous Exit Complete!" << std::endl;
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
