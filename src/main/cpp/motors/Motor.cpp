#include "motors/Motor.h"

#include <iostream>

Motor::Motor(MotorTypes type, uint id, bool isMotorBrushless) : motorType(type), canID(id), isBrushless(isMotorBrushless) {
     switch(type) {
          case REV_SPARK_MAX:
               internalMotorSparkMax = std::make_unique<rev::CANSparkMax>(id, (isMotorBrushless) ? rev::CANSparkMaxLowLevel::MotorType::kBrushless : rev::CANSparkMaxLowLevel::MotorType::kBrushed);
               break;
          default:
               std::cout << "Motor type for motor with CAN id " << id << " has not been properly implemented in the Motor class" << std::endl;
               break;
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
}

void Motor::setMotorPower(double power) {
     this->power = power;

     switch(motorType) {
          case REV_SPARK_MAX:
               internalMotorSparkMax->Set(power * ((isReversed) ? -1 : 1));
               break;
          default:
               std::cout << "Motor type for motor with CAN id " << canID << " has not been properly implemented in the Motor class" << std::endl;
               break;
     }
}

double Motor::getMotorPower() {
     return power;
}