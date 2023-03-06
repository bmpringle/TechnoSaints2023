#include "motors/Motor.h"

#include <iostream>

Motor::Motor(MotorTypes type, uint id, bool isMotorBrushless) : motorType(type), canID(id), isBrushless(isMotorBrushless) {
     switch(type) {
          case REV_SPARK_MAX:
               internalMotorSparkMax = std::make_unique<rev::CANSparkMax>(id, (isMotorBrushless) ? rev::CANSparkMaxLowLevel::MotorType::kBrushless : rev::CANSparkMaxLowLevel::MotorType::kBrushed);
               break;
          case TALON_FX:
               internalMotorTalonFX = std::make_unique<ctre::phoenix::motorcontrol::can::TalonFX>(id);
               break;
          default:
               throw std::runtime_error(std::string("Motor type for motor with CAN id ") + std::to_string(id) + std::string(" has not been properly implemented in the Motor class"));
     }
}

uint Motor::getID() {
     return canID;
}

bool Motor::isInReverse() {
     return isReversed;
}

void Motor::setIsReversed(bool reversed) {
     isReversed = reversed;
     switch(motorType) {
          case REV_SPARK_MAX:
               internalMotorSparkMax->SetInverted(reversed);
               break;
          case TALON_FX:
               internalMotorTalonFX->SetInverted(reversed);
               break;
          default:
               break;
     }
}

void Motor::setMotorPower(double power) {
     this->power = power;

     std::cout << "setting motor power of motor " << canID << " to " << power << std::endl;

     switch(motorType) {
          case REV_SPARK_MAX:
               internalMotorSparkMax->Set(power);
               break;
          case TALON_FX:
               internalMotorTalonFX->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, power);
               break;
          default:
               std::cout << "Motor type for motor with CAN id " << canID << " has not been properly implemented in the Motor class" << std::endl;
               break;
     }
}

double Motor::getMotorPower() {
     return power;
}