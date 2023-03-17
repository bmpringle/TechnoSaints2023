#include "motors/Motor.h"

#include <iostream>

Motor::Motor(MotorTypes type, uint id, bool isMotorBrushless) : motorType(type), canID(id), isBrushless(isMotorBrushless) {
     switch(type) {
          case REV_SPARK_MAX:
               internalMotorSparkMax = std::make_unique<rev::CANSparkMax>(id, (isMotorBrushless) ? rev::CANSparkMaxLowLevel::MotorType::kBrushless : rev::CANSparkMaxLowLevel::MotorType::kBrushed);
               internalEncoderSparkMax = std::make_unique<rev::SparkMaxRelativeEncoder>(internalMotorSparkMax->GetEncoder());
               break;
          case TALON_FX:
               internalMotorTalonFX = std::make_unique<ctre::phoenix::motorcontrol::can::TalonFX>(id);
               internalMotorTalonFX->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);
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
     if(this->power == power) {
          return;
     }

     this->power = power;

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

void Motor::setDataForEncoderMovement(double gear_ratio, double wheel_circumference) {
     double sensor_units_per_rotation = 0;

     switch(motorType) {
          case TALON_FX:
               sensor_units_per_rotation = 4096;
               break;
          default:
               std::cout << "Motor encoder functions for motor with CAN id " << canID << " has not been properly implemented in the Motor class" << std::endl;
               break;
     }
     unitsPerMeter = (sensor_units_per_rotation / gear_ratio) * wheel_circumference;
}

void Motor::goTo(double displacement) {
     if(motorType != TALON_FX) {
          throw std::runtime_error("not useable for non talon fx motor function - Motor::goTo(double)");
     }

     encoderPositionQueue.push_back(encoderPositionQueue.at(encoderPositionQueue.size() - 1) + displacement * unitsPerMeter);
     
     if(internalMotorTalonFX->GetSelectedSensorPosition() == encoderPositionQueue.front()) {
          if(encoderPositionQueue.size() > 1) {
               encoderPositionQueue.erase(encoderPositionQueue.begin());
               internalMotorTalonFX->SetSelectedSensorPosition(encoderPositionQueue.front());
          }
     }
}

void Motor::setEncoderPosition(double pos_in_inches) {
     internalMotorTalonFX->SetSelectedSensorPosition(pos_in_inches);
}

double Motor::getPosition() {
     switch(motorType) {
          case REV_SPARK_MAX:
               return internalEncoderSparkMax->GetPosition();
          case TALON_FX:
               return internalMotorTalonFX->GetSelectedSensorPosition();
     }

     return 0;
}

double Motor::getMotorPower() {
     return power;
}