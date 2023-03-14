#include "Robot.h"

#include <iostream>
#include <functional>
#include <chrono>

//CAN ids
#define FL_MOTOR_ID 2
#define FR_MOTOR_ID 0
#define BL_MOTOR_ID 3
#define BR_MOTOR_ID 1
#define ARM_PIVOT_MOTOR_ID 4
#define ARM_EXTENSION_MOTOR_ID 5

//Motor Hardware Specifications
#define FL_MOTOR_TYPE TALON_FX
#define FR_MOTOR_TYPE TALON_FX
#define BL_MOTOR_TYPE TALON_FX
#define BR_MOTOR_TYPE TALON_FX
#define ARM_PIVOT_MOTOR_TYPE REV_SPARK_MAX
#define ARM_EXTENSION_MOTOR_TYPE REV_SPARK_MAX

#define FL_MOTOR_BRUSHLESS true
#define FR_MOTOR_BRUSHLESS true
#define BL_MOTOR_BRUSHLESS true
#define BR_MOTOR_BRUSHLESS true
#define ARM_PIVOT_MOTOR_BRUSHLESS true
#define ARM_EXTENSION_MOTOR_BRUSHLESS true

#define TEST_RIG false

//initalize member variables in the constructor and not in RobotInit because otherwise the compiler will complain since there's no default constructor for Motor and frc::XboxController
Robot::Robot() : frontLeft(Motor(FL_MOTOR_TYPE, FL_MOTOR_ID, FL_MOTOR_BRUSHLESS)), frontRight(Motor(FR_MOTOR_TYPE, FR_MOTOR_ID, FR_MOTOR_BRUSHLESS)), 
                    backLeft(Motor(BL_MOTOR_TYPE, BL_MOTOR_ID, BL_MOTOR_BRUSHLESS)), backRight(Motor(BR_MOTOR_TYPE, BR_MOTOR_ID, BR_MOTOR_BRUSHLESS)), 
                    controller(LogitechController(0)), cameraFunctionThread(std::thread(pds::cameraThreadFunction, 1)), gameField(pds::createField()), 
                    movementUpdateTimer(Timer()), armPivot(ARM_PIVOT_MOTOR_TYPE, ARM_PIVOT_MOTOR_ID, ARM_PIVOT_MOTOR_BRUSHLESS), 
                    armExtension(ARM_EXTENSION_MOTOR_TYPE, ARM_EXTENSION_MOTOR_ID, ARM_EXTENSION_MOTOR_BRUSHLESS) {
     
     if(TEST_RIG) {
          frontRight.setIsReversed(true);
          backRight.setIsReversed(true);
     }else {
          frontLeft.setIsReversed(true);
          backLeft.setIsReversed(true);
     }  

     frontLeft.setDataForEncoderMovement(8.27, 4 * M_PI);
     frontRight.setDataForEncoderMovement(8.27, 4 * M_PI);
     backLeft.setDataForEncoderMovement(8.27, 4 * M_PI);
     backRight.setDataForEncoderMovement(8.27, 4 * M_PI);
}

Robot::~Robot() {
     pds::freeField(gameField);
}

void Robot::TeleopInit() {
     std::cout << "Teleop Init Complete!" << std::endl;

     std::function<void()> teleopMovementFunction = std::bind(&Robot::teleopMovementPeriodic, this);

     endTeleopMovement = false;
     teleopMovementThread = std::thread(teleopMovementFunction);
}

int double_sign_function(double x) {
     return -(x < 0) + (x > 0);
}

void Robot::teleopMovementPeriodic() {
     const double DEAD_ZONE = 0.05;
     const double MAX_TURN_SPEED = 0.25;
     const double MAX_SLOW_TURN_SPEED = 0.10;
     const double MAX_STRAIGHT_SPEED = 0.4;
     const double ACCELERATION_SPEED = 3.0;
     const double UPDATE_FREQUENCY = 100;
     const double DECELERATION_SPEED = 10.0;
     const double MAX_DRIFT_FACTOR = 0.5;

     double doing_point_turn = false;
     double doing_drift_turn = false;

     int i = 0;

     while(!endTeleopMovement) {
          sleep(1.0 / UPDATE_FREQUENCY); //should be called every 1/100 of a second bc of this

          double left_stick_y_value = controller.GetLeftY();
          double right_stick_y_value = controller.GetRightY();

          double left_trigger_axis = controller.GetLeftTriggerAxis();
          double right_trigger_axis = controller.GetRightTriggerAxis();
          double net_trigger_axis = right_trigger_axis - left_trigger_axis;
          double turn_value = net_trigger_axis * MAX_TURN_SPEED;// + right_stick_y_value * MAX_SLOW_TURN_SPEED;

          const double current_back_left_power = backLeft.getMotorPower();
          const double current_back_right_power = backRight.getMotorPower();
          const double current_front_left_power = frontLeft.getMotorPower();
          const double current_front_right_power = frontRight.getMotorPower();

          ++i;

          if(i == UPDATE_FREQUENCY) {
               i = 0;
               std::cout << "should do turn: " << (((abs(turn_value) > DEAD_ZONE) == true) ? "TRUE" : "FALSE") << std::endl;
               std::cout << "should do lateral movement: " << (((abs(left_stick_y_value) > DEAD_ZONE) == true) ? "TRUE" : "FALSE") << std::endl;
          }

          if(/*abs(left_stick_y_value) < DEAD_ZONE && */ abs(turn_value) > DEAD_ZONE) { //point turn
               frontLeft.setMotorPower(-turn_value);
               frontRight.setMotorPower(turn_value);
               backLeft.setMotorPower(-turn_value);
               backRight.setMotorPower(turn_value);

               doing_point_turn = true;

               continue;
          }else if(doing_point_turn) {
               doing_point_turn = false;

               frontLeft.setMotorPower(0);
               frontRight.setMotorPower(0);
               backLeft.setMotorPower(0);
               backRight.setMotorPower(0);

               continue;
          }

          double acceleration = ACCELERATION_SPEED; //normal acceleration speed

          if(abs(left_stick_y_value) < DEAD_ZONE && abs(turn_value) < DEAD_ZONE) { //if decelerating, set different acceleration speed
               acceleration = DECELERATION_SPEED;
          }

          const double start_power = (current_back_left_power + current_back_right_power + current_front_left_power + current_front_right_power) / 4.0;
          const double goal = MAX_STRAIGHT_SPEED * left_stick_y_value;

          if(abs(left_stick_y_value) > DEAD_ZONE && abs(turn_value) > DEAD_ZONE) { //drift
               double left_drift_factor = 1;
               double right_drift_factor = 1;

               if(turn_value > DEAD_ZONE) {
                    right_drift_factor = MAX_DRIFT_FACTOR * (1 - abs(turn_value));
                    left_drift_factor = 1 - right_drift_factor;
               }

               if(turn_value < DEAD_ZONE) {
                    left_drift_factor = MAX_DRIFT_FACTOR * (1 - abs(turn_value));
                    right_drift_factor = 1 - left_drift_factor;
               }

               const double end_power = start_power + double_sign_function(goal - start_power) * acceleration / UPDATE_FREQUENCY;

               frontLeft.setMotorPower(left_drift_factor * end_power);
               frontRight.setMotorPower(right_drift_factor * end_power);
               backLeft.setMotorPower(left_drift_factor * end_power);
               backRight.setMotorPower(right_drift_factor * end_power);

               doing_drift_turn = true;

               continue;
          }else if(doing_drift_turn) {
               doing_drift_turn = false;

               frontLeft.setMotorPower(start_power);
               frontRight.setMotorPower(start_power);
               backLeft.setMotorPower(start_power);
               backRight.setMotorPower(start_power);

               continue;
          }

          //move straight

          if(abs(goal - start_power) <= acceleration / UPDATE_FREQUENCY) { //can do change instantly
               frontLeft.setMotorPower(goal);
               frontRight.setMotorPower(goal);
               backLeft.setMotorPower(goal);
               backRight.setMotorPower(goal);

               continue;
          }

          //get closer to target speed

          const double end_power = start_power + double_sign_function(goal - start_power) * acceleration / UPDATE_FREQUENCY;

          frontLeft.setMotorPower(end_power);
          frontRight.setMotorPower(end_power);
          backLeft.setMotorPower(end_power);
          backRight.setMotorPower(end_power);
     }
}

void Robot::armMovePeriodic() {
     enum ARM_MOVE_ENUM {
          FORWARDS,
          BACKWARDS,
          HOLD
     };

     ARM_MOVE_ENUM arm_enum;
     
     switch(controller.GetPOV()) {
          case 0:
               arm_enum = FORWARDS;
               break;
          case 45:
               arm_enum = FORWARDS;
               break;
          case 90:
               arm_enum = HOLD;
               break;
          case 135:
               arm_enum = BACKWARDS;
               break;
          case 180:
               arm_enum = BACKWARDS;
               break;
          case 225:
               arm_enum = BACKWARDS;
               break;
          case 270:
               arm_enum = HOLD;
               break;
          case 315:
               arm_enum = HOLD;
               break;
          default:
               arm_enum = HOLD;
               break;
     }
    
    if(arm_enum == FORWARDS) {
          armPivot.setMotorPower(1);
    }

    if(arm_enum == BACKWARDS) {
          armPivot.setMotorPower(-1);
    }

    if(arm_enum == HOLD || armPivotLimit.Get()) {
          armPivot.setMotorPower(0);
    }

    if(controller.GetLeftBumper() && !controller.GetRightBumper()) {
          armExtension.setMotorPower(-0.2);
    }else if(!controller.GetLeftBumper() && controller.GetRightBumper()) {
          armExtension.setMotorPower(0.2);
    }else {
          armExtension.setMotorPower(0);  
    }
}

void Robot::TeleopExit() {
     endTeleopMovement = true;
     teleopMovementThread.join();

     frontLeft.setMotorPower(0);
     frontRight.setMotorPower(0);
     backLeft.setMotorPower(0);
     backRight.setMotorPower(0);

     std::cout << "Teleop Exit Complete!" << std::endl;
}

void Robot::AutonomousInit() {
     std::cout << "Autonomous Init Complete!" << std::endl;

     frontLeft.goTo(2);
     frontRight.goTo(2);
     backLeft.goTo(2);
     backRight.goTo(2);
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
     armMovePeriodic();
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
