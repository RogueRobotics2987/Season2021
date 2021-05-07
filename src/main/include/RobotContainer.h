// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ActuatorSubsystem.h"
#include "commands/Actuator.h"
#include "commands/BeginCompressor.h"
#include "subsystems/Compressor.h"
#include "subsystems/ShooterSubsystem.h"
#include "commands/Shooter.h"
#include <iostream>
#include "commands/ShooterSafe.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  // frc2::Command* GetAutonomousCommand();

 private:
  // The driver's controller
  //frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  frc::Joystick m_driverController{OIConstants::kDriverControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;
  ShooterSubsystem m_Shooter;
  ActuatorSubsystem m_Actuator;
  CompressorObject m_Compressor;

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
};
