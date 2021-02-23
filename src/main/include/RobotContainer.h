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
#include "commands/PickupBallAuto.h"
#include "subsystems/Compressor.h"
#include "commands/beginCompressor.h"
#include "subsystems/Climber.h"
#include "commands/Climb.h"
#include "commands/ThirdStageWheel.h" 
#include "commands/ballReset.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/CommandScheduler.h>
#include "commands/TankDrive.h"
#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <iostream>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RamseteCommand.h>
#include "Constants.h" 
#include "commands/SpinWheel.h"
#include "commands/PIDShoot.h"
#include "commands/IntakeOut.h" 
#include "commands/startConveyor.h" 
#include "commands/shooterBackwards.h" 
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>



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
  frc::Joystick joyLeft{0}; 
  frc::Joystick joyRight{2}; 
  //frc2::JoystickButton* j1; 
  //frc2::JoystickButton j1{&joyLeft, 1}; 
  frc::Joystick xbox{1}; 
  ColorSensor cSensor; 
  ShooterActuator actuator;
  CompressorObject m_compressor;
  DriveTrain m_drivetrain;
   
  //Dannalyn's shooter code
  Shooter m_shooter; 

  //Sydneys Intake Code
  Intake m_intake;

  //Climb code
  Climber m_climber;

  Autonomous m_autonomousCommand;

  void ConfigureButtonBindings();
};
