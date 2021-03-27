#include "subsystems/ActuatorSubsystem.h"

ActuatorSubsystem::ActuatorSubsystem(){
ActuatorMotor = new rev::CANSparkMax(m_MotorController, rev::CANSparkMax::MotorType::kBrushless);
}

void ActuatorSubsystem::Periodic() {}

void ActuatorSubsystem::Extend(){
 
}
void ActuatorSubsystem::Retract(){

}