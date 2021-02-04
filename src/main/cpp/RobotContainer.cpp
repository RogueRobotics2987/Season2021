// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc/Joystick.h>

RobotContainer::RobotContainer() {
  xbox = new frc::Joystick(1);
  stick = new frc::Joystick(0);
  // Initialize all of your commands and subsystems here
  // m_subsystem = new DifferentialDriveSubsystem();
  // DifferentialDriveCommand = new DifferentialDriveCommand(*m_subsystem)
  m_subsystem.SetDefaultCommand(DifferentialDriveCommand(&m_subsystem, stick));
  m_armsubsystem.SetDefaultCommand(ArmCommand(&m_armsubsystem, xbox));
  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

// frc2::Command* RobotContainer::GetAutonomousCommand() {
//   // An example command will be run in autonomous
//   return new DifferentialDriveCommand;
// }
