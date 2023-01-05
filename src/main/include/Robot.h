#ifndef ROBOT_H
#define ROBOT_H

#include <frc/TimedRobot.h>

class Robot : public frc::TimedRobot {
     public:
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

};

#endif