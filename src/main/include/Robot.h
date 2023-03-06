#ifndef ROBOT_H
#define ROBOT_H

#include <frc/TimedRobot.h>
#include "io/LogitechController.h"

#include "motors/Motor.h"
#include "vision/PositionDetectionSystem.h"
#include "time/Timer.h"

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

          Motor frontLeft;
          Motor frontRight;
          Motor backLeft;
          Motor backRight;

          LogitechController controller;

          double maxTurnSpeed = 0.25;

          double maxAcceleratePerSecond = 1.0;
          
          double teleopMovementPeriodicCallRate = 1.0 / 60.0;

          std::thread cameraFunctionThread;

          Field gameField;

          Timer movementUpdateTimer;
          
};

#endif