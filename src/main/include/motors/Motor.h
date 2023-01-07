#ifndef MOTOR_H
#define MOTOR_H

#include "MotorTypes.h"

#include "rev/CanSparkMax.h"
#include <memory>

class Motor {
     public:
          Motor(MotorTypes type, uint id, bool isMotorBrushless);

          uint getID();

          bool isInReverse();

          void setIsReversed(bool reversed);

          void setMotorPower(double power);

          double getMotorPower();



     private:
          MotorTypes motorType;
          uint canID;
          bool isBrushless = true;
          bool isReversed = false;
          double power = 0.0; 
          
          std::unique_ptr<rev::CANSparkMax> internalMotorSparkMax; 
};

#endif