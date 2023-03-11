#ifndef MOTOR_H
#define MOTOR_H

#include "MotorTypes.h"

#include <rev/CanSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>

#include <memory>
#include <vector>

//Motor class to handle hardware specific details
class Motor {
     public:
          Motor(MotorTypes type, uint id, bool isMotorBrushless);

          uint getID(); //returns CAN id of the motor

          bool isInReverse(); //returns member variable isReversed

          void setIsReversed(bool reversed); //sets member variable isReversed

          void setMotorPower(double power); //sets member variable power and sets the hardware-specific motor's power

          double getMotorPower(); //returns member variable power

          void goTo(double displacement); //closed loop encoder, displacement is in meters. CURRENTLY ASSUMES IT IS BEING CALLED ON TALON FX

          void setDataForEncoderMovement(double gear_ratio, double wheel_circumference); //set encoder data

     private:
          MotorTypes motorType;
          uint canID;
          bool isBrushless = true;
          bool isReversed = false;
          double power = 0.0;
          double unitsPerMeter = 0;

          std::vector<double> encoderPositionQueue = {0};
          
          std::unique_ptr<rev::CANSparkMax> internalMotorSparkMax; 
          std::unique_ptr<ctre::phoenix::motorcontrol::can::TalonFX> internalMotorTalonFX; 
};

#endif