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
     //armExtension.setDataForEncoderMovement(16, 1);

     /*int32_t max = 0;
     int32_t deadband_max = 0;
     int32_t center = 0;
     int32_t deadband_min = 0;
     int32_t min = 0;

     armServo1.GetRawBounds(&max, &deadband_max, &center, &deadband_min, &min);

     std::cout << "settings: " << max << ", " << deadband_max << ", " << center << ", " << deadband_min << ", " << min << std::endl;

     armServo1.SetRawBounds(2000, -1500, -1500, -1500, 99);
     armServo2.SetRawBounds(2000, -1500, -1500, -1500, 99);*/
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
     const double MAX_TURN_SPEED = 0.3;
     const double MAX_SLOW_TURN_SPEED = 0.1;
     const double MAX_STRAIGHT_SPEED = 0.55;
     const double ACCELERATION_SPEED = 2.0;
     const double UPDATE_FREQUENCY = 100;
     const double DECELERATION_SPEED = 10.0;
     //const double MAX_DRIFT_FACTOR = 0.5;

     double doing_point_turn = false;
     //double doing_drift_turn = false;

     while(!endTeleopMovement) {
          sleep(1.0 / UPDATE_FREQUENCY); //should be called every 1/100 of a second bc of this

          double left_stick_y_value = controller.GetLeftY();
          double right_stick_x_value = controller.GetRightX();

          double left_trigger_axis = controller.GetLeftTriggerAxis();
          double right_trigger_axis = controller.GetRightTriggerAxis();
          double net_trigger_axis = right_trigger_axis - left_trigger_axis;
          double turn_value = net_trigger_axis * MAX_TURN_SPEED + right_stick_x_value * MAX_SLOW_TURN_SPEED;

          const double current_back_left_power = backLeft.getMotorPower();
          const double current_back_right_power = backRight.getMotorPower();
          const double current_front_left_power = frontLeft.getMotorPower();
          const double current_front_right_power = frontRight.getMotorPower();

          if(/*fabs(left_stick_y_value) < DEAD_ZONE &&*/ fabs(turn_value) > DEAD_ZONE) { //point turn
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

          /*if(fabs(left_stick_y_value) > DEAD_ZONE && fabs(turn_value) > DEAD_ZONE) { //drift
               double left_drift_factor = 1;
               double right_drift_factor = 1;

               if(turn_value > DEAD_ZONE) {
                    right_drift_factor = MAX_DRIFT_FACTOR * (1 - fabs(turn_value));
                    left_drift_factor = 1 - right_drift_factor;
               }

               if(turn_value < DEAD_ZONE) {
                    left_drift_factor = MAX_DRIFT_FACTOR * (1 - fabs(turn_value));
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
          }*/

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

int i = 0;

void Robot::armMovePeriodic() {
     enum ARM_MOVE_ENUM {
          FORWARDS,
          BACKWARDS,
          HOLD
     };

     ARM_MOVE_ENUM arm_enum;
     
     switch(controller.GetPOV()) {
          case 0:
               arm_enum = BACKWARDS;
               break;
          case 45:
               arm_enum = BACKWARDS;
               break;
          case 90:
               arm_enum = HOLD;
               break;
          case 135:
               arm_enum = FORWARDS;
               break;
          case 180:
               arm_enum = FORWARDS;
               break;
          case 225:
               arm_enum = FORWARDS;
               break;
          case 270:
               arm_enum = HOLD;
               break;
          case 315:
               arm_enum = BACKWARDS;
               break;
          default: 
               arm_enum = HOLD;
               break;
     }

     double pivot_power_speed = controller.GetXButton() ? 0.7 : 0.5;

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
          std::cout << "arm pivot encoder position (change): " << armPivot.getPosition() << std::endl;
     }

     if(controller.GetLeftBumper() && !controller.GetRightBumper() /*&& armExtension.getEncoderPosition() > 0.1*/) {
          armExtension.setMotorPower(-0.1);
          if(i == 60) {
               i = 0;
               std::cout << "arm extension encoder position (contract): " << armExtension.getPosition() << std::endl;
          }
     }else if(!controller.GetLeftBumper() && controller.GetRightBumper()/* && armExtension.getEncoderPosition() < 0.4*/) {
          armExtension.setMotorPower(0.1);
          if(i == 60) {
               i = 0;
               std::cout << "arm extension encoder position (expand): " << armExtension.getPosition() << std::endl;
          }
     }else {
          armExtension.setMotorPower(0);  
     }

     ++i;

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
