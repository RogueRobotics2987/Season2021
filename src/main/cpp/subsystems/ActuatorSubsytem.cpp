#include "subsystems/ActuatorSubsystem.h"

ActuatorSubsystem::ActuatorSubsystem(){
ActuatorMotor = new rev::CANSparkMax(m_MotorController, rev::CANSparkMax::MotorType::kBrushless);
ActuatorMotor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
ActuatorMotor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, 5000); // TODO Change limit number
}

void ActuatorSubsystem::Periodic() {
    ForwardLimitSwitch = ActuatorMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get();
    frc::SmartDashboard::PutBoolean("Forward Limit Switch", ForwardLimitSwitch);
    // frc::SmartDashboard::PutBoolean("Reverse Limit Switch", );
    
}

void ActuatorSubsystem::Retract(){
    if(ForwardLimitSwitch == false){
 ActuatorMotor->SetVoltage(units::voltage::volt_t(2));
    }
    else{
        ActuatorMotor->SetVoltage(units::voltage::volt_t(0));
    }
}

void ActuatorSubsystem::Extend(){
 ActuatorMotor->SetVoltage(units::voltage::volt_t(-2));
}

void ActuatorSubsystem::Neutral(){
    
 ActuatorMotor->SetVoltage(units::voltage::volt_t(0));
}

bool ActuatorSubsystem::GetForwardLimitSwitch(){ // Gets the bottom limit switch state
    return ForwardLimitSwitch;
}
