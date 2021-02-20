// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PickupBallAuto.h"

PickupBallAuto::PickupBallAuto(Intake& c_intake, bool c_enabled) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_intake = c_intake;
  m_enabled = c_enabled;
}

// Called when the command is initially scheduled.
void PickupBallAuto::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PickupBallAuto::Execute() {
  if(m_enabled) {
    m_intake.IntakeBall(0.65);
  } else {
    m_intake.IntakeBall(0.0);
  }

}

// Called once the command ends or is interrupted.
void PickupBallAuto::End(bool interrupted) {}

// Returns true when the command should end.
bool PickupBallAuto::IsFinished() {
  return true;
}
