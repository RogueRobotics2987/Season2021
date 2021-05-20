#include "subsystems/ActuatorSubsystem.h"

ActuatorSubsystem::ActuatorSubsystem(){
ActuatorMotor = new rev::CANSparkMax(m_MotorController, rev::CANSparkMax::MotorType::kBrushless);
// ActuatorMotor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
// ActuatorMotor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, );
}

void ActuatorSubsystem::Periodic() {
    ForwardLimitSwitch = ActuatorMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get();
    frc::SmartDashboard::PutBoolean("Forward Limit Switch", ForwardLimitSwitch);
    // frc::SmartDashboard::PutBoolean("Reverse Limit Switch", );
}

void ActuatorSubsystem::Retract(){
    ForwardLimitSwitch = ActuatorMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get();
    if(ForwardLimitSwitch == true){
 ActuatorMotor->SetVoltage(units::voltage::volt_t(2));
 std::cout << "I'm Trying to Retract!" << std::endl;
    }
    else{
        ActuatorMotor->SetVoltage(units::voltage::volt_t(0));
         std::cout << "I'm Trying to Retract, but the Limit Switch is on!" << std::endl;

    }
}

void ActuatorSubsystem::Extend(){    
ForwardLimitSwitch = ActuatorMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get();

 ActuatorMotor->SetVoltage(units::voltage::volt_t(-2));
  std::cout << "I'm Trying to Extend!" << std::endl;
}

void ActuatorSubsystem::Neutral(){
ForwardLimitSwitch = ActuatorMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get();   
  std::cout << "I'm Neutral!" << std::endl;
 ActuatorMotor->SetVoltage(units::voltage::volt_t(0));
}


