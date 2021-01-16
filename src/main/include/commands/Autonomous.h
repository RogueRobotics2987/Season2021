/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/DriveTrain.h"
#include <frc/Timer.h>
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"
#include "subsystems/ShooterActuator.h"

/**
 * The main autonomous command to pickup and deliver the soda to the box.
 */
class Autonomous
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, Autonomous> {
 public:
  Autonomous(DriveTrain* drivetrain, Shooter* shooter, ShooterActuator* shooteractuator, Intake* intake);

  void Initialize();
  void Execute();
  void End(bool interrupted);

  bool IsFinished() override;
 private:
  DriveTrain* m_driveTrain;
  Shooter* m_shooter;
  ShooterActuator* m_shooterActuator;
  Intake* m_intake;
  frc::Timer* m_timer;
  double state = 0;
  double driveTime = 0; 
  bool stop = false; 
  double shootTime = 0; 

};
