#ifndef LOGITECHCONTROLLER_H
#define LOGITECHCONTROLLER_H

#include <frc/XboxController.h>

class LogitechController : public frc::XboxController {
     public:
          explicit LogitechController(int port) : frc::XboxController(port) {

          }

          double GetLeftY() const {
               return -frc::XboxController::GetLeftY();
          }
};

#endif