#include "subsystems/ActuatorSubsystem.h"

ActuatorSubsystem::ActuatorSubsystem(){
ActuatorMotor = new rev::CANSparkMax(m_MotorController, rev::CANSparkMax::MotorType::kBrushless);
}

void ActuatorSubsystem::Periodic() {
    ForwardLimitSwitch = ActuatorMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get();
    frc::SmartDashboard::PutBoolean("Forward Limit Switch", ForwardLimitSwitch);
    // frc::SmartDashboard::PutBoolean("Reverse Limit Switch", ActuatorMotor->GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get());
    
}

void ActuatorSubsystem::Extend(){
    if(ForwardLimitSwitch == false){
 ActuatorMotor->SetVoltage(units::voltage::volt_t(2));
    }
    else{
        ActuatorMotor->SetVoltage(units::voltage::volt_t(0));
    }
}

void ActuatorSubsystem::Retract(){
//  ActuatorMotor->SetVoltage(units::voltage::volt_t(-2));
}

void ActuatorSubsystem::Neutral(){
    
 ActuatorMotor->SetVoltage(units::voltage::volt_t(0));
}


