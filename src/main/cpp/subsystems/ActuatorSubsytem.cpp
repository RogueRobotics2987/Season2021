#include "subsystems/ActuatorSubsystem.h"

ActuatorSubsystem::ActuatorSubsystem(){
ActuatorMotor = new rev::CANSparkMax(m_MotorController, rev::CANSparkMax::MotorType::kBrushless);
}

void ActuatorSubsystem::Periodic() {
    frc::SmartDashboard::PutBoolean("Forward Limit Switch", ActuatorMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get());
    frc::SmartDashboard::PutBoolean("Reverse Limit Switch", ActuatorMotor->GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get());
}

void ActuatorSubsystem::Extend(){
//  ActuatorMotor->SetVoltage(units::voltage::volt_t(2));
}

void ActuatorSubsystem::Retract(){
//  ActuatorMotor->SetVoltage(units::voltage::volt_t(-2));
}

void ActuatorSubsystem::Neutral(){
//  ActuatorMotor->SetVoltage(units::voltage::volt_t(0));
}


