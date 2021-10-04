#include "subsystems/ActuatorSubsystem.h"

ActuatorSubsystem::ActuatorSubsystem(){
ActuatorMotor = new rev::CANSparkMax(m_MotorController, rev::CANSparkMax::MotorType::kBrushless);
ActuatorMotor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
// ActuatorMotor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, 5000); // TODO Change limit number
}

void ActuatorSubsystem::Periodic() {
    ForwardLimitSwitch = ActuatorMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get();
    frc::SmartDashboard::PutBoolean("Forward Limit Switch", ForwardLimitSwitch);
    // frc::SmartDashboard::PutBoolean("Reverse Limit Switch", );
}

void ActuatorSubsystem::Retract(){
 ActuatorMotor->SetVoltage(units::voltage::volt_t(2.0));
 std::cout << "I'm Trying to Retract!" << std::endl;
    
}

void ActuatorSubsystem::Extend(){    
ForwardLimitSwitch = ActuatorMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get();

 ActuatorMotor->SetVoltage(units::voltage::volt_t(-3.0));
  std::cout << "I'm Trying to Extend!" << std::endl;
}

void ActuatorSubsystem::Neutral(){
ForwardLimitSwitch = ActuatorMotor->GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get();   
  std::cout << "I'm Neutral!" << std::endl;
 ActuatorMotor->SetVoltage(units::voltage::volt_t(0));
}

bool ActuatorSubsystem::GetForwardLimitSwitch(){ // Gets the bottom limit switch state
    return ForwardLimitSwitch;
}

  rev::CANEncoder ActuatorSubsystem::GetEncoder(rev::CANEncoder::EncoderType sensorType, int counts_per_rev){
     return ActuatorMotor->GetEncoder(sensorType, counts_per_rev = counts_per_rev); 
  }