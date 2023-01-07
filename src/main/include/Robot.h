#ifndef ROBOT_H
#define ROBOT_H

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>

#include "motors/Motor.h"

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

     private:
          Motor frontLeft;
          Motor frontRight;
          Motor backLeft;
          Motor backRight;

          frc::XboxController controller;

          double maxTurnSpeed = 0.5;
};

#endif