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

//set to "HIGH", "MIDDLE", "LOW", or "NOTHING"
#define AUTO_DROP "MIDDLE" 

//initalize member variables in the constructor and not in RobotInit because otherwise the compiler will complain since there's no default constructor for Motor and frc::XboxController
Robot::Robot() : frontLeft(Motor(FL_MOTOR_TYPE, FL_MOTOR_ID, FL_MOTOR_BRUSHLESS)), frontRight(Motor(FR_MOTOR_TYPE, FR_MOTOR_ID, FR_MOTOR_BRUSHLESS)), 
                    backLeft(Motor(BL_MOTOR_TYPE, BL_MOTOR_ID, BL_MOTOR_BRUSHLESS)), backRight(Motor(BR_MOTOR_TYPE, BR_MOTOR_ID, BR_MOTOR_BRUSHLESS)), 
                    armPivot(ARM_PIVOT_MOTOR_TYPE, ARM_PIVOT_MOTOR_ID, ARM_PIVOT_MOTOR_BRUSHLESS), armExtension(ARM_EXTENSION_MOTOR_TYPE, ARM_EXTENSION_MOTOR_ID, ARM_EXTENSION_MOTOR_BRUSHLESS),  
                    armServo1(frc::Servo(0)), armServo2(frc::Servo(1)), controller(LogitechController(0)), cameraFunctionThread(std::thread(pds::cameraThreadFunction, 1)), 
                    gameField(pds::createField()), movementUpdateTimer(Timer()) {
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
     const double ACCELERATION_SPEED = 2.0;
     const double UPDATE_FREQUENCY = 100;
     const double DECELERATION_SPEED = 10.0;

     double doing_point_turn = false;

     while(!endTeleopMovement) {
          sleep(1.0 / UPDATE_FREQUENCY); //should be called every 1/100 of a second bc of this

          double MAX_STRAIGHT_SPEED = (controller.GetYButton()) ? 0.8 : 0.65;

          double left_stick_y_value = controller.GetLeftY();
          double right_stick_x_value = controller.GetRightX();
          double turn_value = right_stick_x_value * MAX_TURN_SPEED;

          const double current_back_left_power = backLeft.getMotorPower();
          const double current_back_right_power = backRight.getMotorPower();
          const double current_front_left_power = frontLeft.getMotorPower();
          const double current_front_right_power = frontRight.getMotorPower();

          if(fabs(turn_value) > DEAD_ZONE) { //point turn
               frontLeft.setMotorPower(turn_value);
               frontRight.setMotorPower(-turn_value);
               backLeft.setMotorPower(turn_value);
               backRight.setMotorPower(-turn_value);

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

          if(fabs(left_stick_y_value) < DEAD_ZONE && fabs(turn_value) < DEAD_ZONE) { //if decelerating, set different acceleration speed
               acceleration = DECELERATION_SPEED;
          }

          const double start_power = (current_back_left_power + current_back_right_power + current_front_left_power + current_front_right_power) / 4.0;
          const double goal = MAX_STRAIGHT_SPEED * left_stick_y_value;

          //move straight

          if(fabs(goal - start_power) <= acceleration / UPDATE_FREQUENCY) { //can do change instantly
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

     ARM_MOVE_ENUM arm_enum = HOLD;

     const double DEAD_ZONE = 0.05;
     double pivot_power_speed = 0;

     if(controller.GetLeftTriggerAxis() > DEAD_ZONE) {
          arm_enum = BACKWARDS;
          pivot_power_speed = (controller.GetXButton() ? 0.7 : 0.5) * controller.GetLeftTriggerAxis();
     }

     if(controller.GetRightTriggerAxis() > DEAD_ZONE) {
          arm_enum = FORWARDS;
          pivot_power_speed = (controller.GetXButton() ? 0.7 : 0.5) * controller.GetRightTriggerAxis();
     }

     double pivot_power = 0;

     if(arm_enum == FORWARDS) {
          pivot_power = pivot_power_speed;
     }

     if(arm_enum == BACKWARDS) {
          pivot_power = -pivot_power_speed;
     }

     if(arm_enum == HOLD || (arm_enum == BACKWARDS && !armPivotLimit.Get()) || (arm_enum == FORWARDS && !armPivotLimit2.Get())) {
          pivot_power = 0;
     }

     if(armPivot.getMotorPower() != pivot_power) {
          armPivot.setMotorPower(pivot_power);
     }

     if(controller.GetLeftBumper() && !controller.GetRightBumper()) {
          armExtension.setMotorPower(-0.2);
     }else if(!controller.GetLeftBumper() && controller.GetRightBumper()) {
          armExtension.setMotorPower(0.2);
     }else {
          armExtension.setMotorPower(0);  
     }

     if(controller.GetAButtonPressed()) {
          switch(arm_servo_state) {
               case CLOSED_CONE:
                    arm_servo_state = OPEN;
                    break;
               case CLOSED_CUBE:
                    arm_servo_state = OPEN;
                    break;
               case OPEN:
                    arm_servo_state = CLOSED_CONE;
                    break;
          }
     }

     if(controller.GetBButtonPressed()) {
          switch(arm_servo_state) {
               case CLOSED_CONE:
                    arm_servo_state = OPEN;
                    break;
               case CLOSED_CUBE:
                    arm_servo_state = OPEN;
                    break;
               case OPEN:
                    arm_servo_state = CLOSED_CUBE;
                    break;
          }
     }

     if(arm_servo_state == OPEN) {
          if(armServo1.GetPosition() != 1) {
               armServo1.SetPosition(1);
          }
          if(armServo2.GetPosition() != 1) {
               armServo2.SetPosition(1);
          }
     }

     if(arm_servo_state == CLOSED_CONE) {
          if(armServo1.GetPosition() != 0) {
               armServo1.SetPosition(0);
          }
          if(armServo2.GetPosition() != 0) {
               armServo2.SetPosition(0);
          }
     }

     if(arm_servo_state == CLOSED_CUBE) {
          if(armServo1.GetPosition() != 0.5) {
               armServo1.SetPosition(0.5);
          }
          if(armServo2.GetPosition() != 0.5) {
               armServo2.SetPosition(0.5);
          }
     }
}

void Robot::TeleopExit() {
     endTeleopMovement = true;
     teleopMovementThread.join();

     frontLeft.setMotorPower(0);
     frontRight.setMotorPower(0);
     backLeft.setMotorPower(0);
     backRight.setMotorPower(0);
     armExtension.setMotorPower(0);
     armPivot.setMotorPower(0);


     std::cout << "Teleop Exit Complete!" << std::endl;
}

void Robot::AutonomousInit() {
     std::cout << "Autonomous Init Complete!" << std::endl;
}

void Robot::middleDropAutonomous() {
     double ms_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count();

     //close servos AROUND CUBE (500 ms)
     if(ms_since_start < 500) {
          armExtension.setMotorPower(0);
          armPivot.setMotorPower(0);
          armServo1.SetPosition(0.5);
          armServo2.SetPosition(0.5);
          return;
     }

     //pivot arm down (1500 ms)
     if(ms_since_start < 2000) {
          armPivot.setMotorPower(-0.2);
          armExtension.setMotorPower(0);
          return;
     }

     //extend arm out (1500 ms)
     if(ms_since_start < 3500) {
          armExtension.setMotorPower(0.2);
          armPivot.setMotorPower(0);
          return;
     }

     //open servos (500 ms)
     if(ms_since_start < 4000) {
          armExtension.setMotorPower(0);
          armPivot.setMotorPower(0);
          armServo1.SetPosition(1);
          armServo2.SetPosition(1);
          return;
     }    

     //contract arm back in (1500 ms)
     if(ms_since_start < 5500) {
          armExtension.setMotorPower(-0.2);
          armPivot.setMotorPower(0);
          return;
     }

     //pivot arm up (1500 ms)
     if(ms_since_start < 7000) {
          armPivot.setMotorPower(0.2);
          armExtension.setMotorPower(0);
          return;
     }

     //move out of community (2000 ms)
     if(ms_since_start < 9000) {
          frontLeft.setMotorPower(0.5);
          frontRight.setMotorPower(0.5);
          backLeft.setMotorPower(0.5);
          backRight.setMotorPower(0.5);
          armExtension.setMotorPower(0);
          armPivot.setMotorPower(0);
          return;
     }

     frontLeft.setMotorPower(0);
     frontRight.setMotorPower(0);
     backLeft.setMotorPower(0);
     backRight.setMotorPower(0);
     armExtension.setMotorPower(0);
     armPivot.setMotorPower(0);
}

void Robot::lowDropAutonomous() {
     double ms_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count();

     //close servos AROUND CUBE (500 ms)
     if(ms_since_start < 500) {
          armExtension.setMotorPower(0);
          armPivot.setMotorPower(0);
          armServo1.SetPosition(0.5);
          armServo2.SetPosition(0.5);
          return;
     }

     //pivot arm down (4500 ms)
     if(ms_since_start < 5000) {
          armPivot.setMotorPower(-0.2);
          armExtension.setMotorPower(0);
          return;
     }

     //open servos (500 ms)
     if(ms_since_start < 5500) {
          armExtension.setMotorPower(0);
          armPivot.setMotorPower(0);
          armServo1.SetPosition(1);
          armServo2.SetPosition(1);
          return;
     }    

     //pivot arm up (4500 ms)
     if(ms_since_start < 10000) {
          armPivot.setMotorPower(0.2);
          armExtension.setMotorPower(0);
          return;
     }

     //move out of community (2000 ms)
     if(ms_since_start < 12000) {
          frontLeft.setMotorPower(0.5);
          frontRight.setMotorPower(0.5);
          backLeft.setMotorPower(0.5);
          backRight.setMotorPower(0.5);
          armExtension.setMotorPower(0);
          armPivot.setMotorPower(0);
          return;
     } 

     frontLeft.setMotorPower(0);
     frontRight.setMotorPower(0);
     backLeft.setMotorPower(0);
     backRight.setMotorPower(0);
     armExtension.setMotorPower(0);
     armPivot.setMotorPower(0);
}

void Robot::AutonomousPeriodic() {
     if(set_start) {
          set_start = false;
          start = std::chrono::system_clock::now();
     }

     if(AUTO_DROP == "HIGH") {
          //todo: create a drop function to get the cube in the highest bucket
          return;
     }

     if(AUTO_DROP == "MIDDLE") {
          middleDropAutonomous();
          return;
     }

     if(AUTO_DROP == "LOW") {
          lowDropAutonomous();
          return;
     }

     //do nothing
}

void Robot::AutonomousExit() {
     std::cout << "Autonomous Exit Complete!" << std::endl;
     set_start = true;
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
