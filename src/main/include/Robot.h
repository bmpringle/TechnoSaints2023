#ifndef ROBOT_H
#define ROBOT_H

#include <frc/TimedRobot.h>
#include <frc/DigitalInput.h>
#include "io/LogitechController.h"

#include "motors/Motor.h"
#include "vision/PositionDetectionSystem.h"
#include "time/Timer.h"
#include <atomic>
#include <frc/Servo.h>

class Robot : public frc::TimedRobot {
     public:
          Robot();

          void RobotInit() override;
          void RobotPeriodic() override;
          void DisabledInit() override;
          void DisabledPeriodic() override;
          void DisabledExit() override;
          void AutonomousInit() override;
          void AutonomousPeriodic() override;
          void AutonomousExit() override;
          void TeleopInit() override;
          void TeleopPeriodic() override;
          void TeleopExit() override; 

          ~Robot() override;

     private:
          void teleopMovementPeriodic();
          void armMovePeriodic();
          void middleDropAutonomous();
          void lowDropAutonomous();

          Motor frontLeft;
          Motor frontRight;
          Motor backLeft;
          Motor backRight;

          Motor armPivot;
          Motor armExtension;
          frc::DigitalInput armPivotLimit = frc::DigitalInput(0);
          frc::DigitalInput armPivotLimit2 = frc::DigitalInput(1);

          frc::Servo armServo1;
          frc::Servo armServo2;

          enum {
               CLOSED_CONE,
               CLOSED_CUBE,
               OPEN
          } arm_servo_state;

          LogitechController controller;

          std::thread cameraFunctionThread;

          Field gameField;

          Timer movementUpdateTimer;

          std::thread teleopMovementThread;
          std::atomic<bool> endTeleopMovement;

          std::chrono::_V2::system_clock::time_point start = std::chrono::system_clock::now();
          bool set_start = true;
          
};

#endif