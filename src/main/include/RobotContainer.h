/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc2/command/button/JoystickButton.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc2/command/Command.h>
#include "ctre/Phoenix.h"
#include "commands/Autonomous.h"
#include "subsystems/DriveTrain.h"


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

  frc2::Command* GetAutonomousCommand();
  void PeriodicDebug(void);

 private:
  // The robot's subsystems and commands are defined here...
  frc::Joystick joyLeft{1}; 
  frc::Joystick joyRight{2}; 

  
  

  DriveTrain m_drivetrain;
   
  Autonomous m_autonomousCommand;

  std::shared_ptr<NetworkTable> limelightTablerri;
   double txi = 0.0;
   double tyi = 0.0;
   bool RedB = false;
   bool RedA = false;

  void ConfigureButtonBindings();
};
