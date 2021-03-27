#include "subsystems/ActuatorSubsystem.h"

ActuatorSubsystem::ActuatorSubsystem(){
ActuatorMotor = new rev::CANSparkMax(m_MotorController, rev::CANSparkMax::MotorType::kBrushless);
}

void ActuatorSubsystem::Periodic() {}

void ActuatorSubsystem::Extend(){
 ActuatorMotor->SetVoltage(units::voltage::volt_t(8));
}

void ActuatorSubsystem::Retract(){
 ActuatorMotor->SetVoltage(units::voltage::volt_t(-8));
}

void ActuatorSubsystem::Neutral(){
 ActuatorMotor->SetVoltage(units::voltage::volt_t(0));
}


