// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShooterSafe.h"

ShooterSafe::ShooterSafe(ShooterSubsystem* l_Shooter) {
  m_Shooter = l_Shooter;
  AddRequirements(m_Shooter);

  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ShooterSafe::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShooterSafe::Execute() {
  m_Shooter->Open(1);
  m_Shooter->Open(2);
  m_Shooter->Open(3);
  m_Shooter->Open(4);
  m_Shooter->Open(5);
  
}

// Called once the command ends or is interrupted.
void ShooterSafe::End(bool interrupted) {}

// Returns true when the command should end.
bool ShooterSafe::IsFinished() {
  return false;
}
