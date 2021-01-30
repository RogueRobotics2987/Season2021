// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

ArmSubsystem::ArmSubsystem() {
  // Implementation of subsystem constructor goes here.
  
  Arm1 = new WPI_TalonSRX(23);
  Arm2 = new WPI_TalonSRX(18);
  

};

void ArmSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ArmSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
// Control drive train with joystick input
void ArmSubsystem::ArmControl(double arm1, double arm2) {
 Arm1->Set(arm1);
 Arm2->Set(arm2);
}