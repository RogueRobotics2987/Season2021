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
#include "subsystems/ColorSensor.h"
#include "subsystems/Shooter.h"
#include "commands/ShootCmdCls.h"
#include "commands/TrimAngle.h"
#include "subsystems/ShooterActuator.h"
#include "subsystems/Intake.h"
#include "commands/PickupBall.h"
#include "subsystems/Compressor.h"
#include "commands/beginCompressor.h"
#include "subsystems/Climber.h"
#include "commands/Climb.h"
#include "commands/ThirdStageWheel.h" 
#include "commands/ballReset.h"
#include "commands/AutoPickup.h"
#include "commands/AutoTrimAngle2.h"
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include "commands/AutoShoot.h"
#include <frc/smartdashboard/SendableChooser.h>

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
  // void frc::SendableChooser();

 private:
  // The robot's subsystems and commands are defined here...
  frc::Joystick joyLeft{0}; 
  frc::Joystick joyRight{2}; 
  //frc2::JoystickButton* j1; 
  frc::Joystick xbox{1}; 
  ColorSensor cSensor; 
  ShooterActuator actuator;
  CompressorObject m_compressor;

  

  

  //Elevator* m_elevator;
  DriveTrain m_drivetrain;
  //rc2::JoystickButton j1{&joyLeft, 1}; 
   

  //Dannalyn's shooter code
  Shooter m_shooter; 
 //frc2::JoystickButton joyB2{&joyLeft, 2};
  //frc2::JoystickButton joyB3{&joyLeft, 3};

  //Sydneys Intake Code
  Intake m_intake;

  //Climb code
  Climber m_climber;

  Autonomous m_autonomousCommand;

  void ConfigureButtonBindings();
};
