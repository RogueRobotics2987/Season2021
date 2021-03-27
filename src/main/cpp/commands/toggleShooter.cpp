// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/toggleShooter.h"

toggleShooter::toggleShooter(Shooter* c_shooter) {
  m_shooter = c_shooter;

  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void toggleShooter::Initialize(){
  m_shooter -> toggleShoot();
}

// Called repeatedly when this Command is scheduled to run
void toggleShooter::Execute() {

}

// Called once the command ends or is interrupted.
void toggleShooter::End(bool interrupted) {

}

// Returns true when the command should end.
bool toggleShooter::IsFinished() {
  return true;
}
