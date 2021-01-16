<<<<<<< HEAD
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>

#include "subsystems/DriveTrain.h"

/**
 * Have the robot drive tank style using the PS3 Joystick until interrupted.
 */
class TankDrive : public frc2::CommandHelper<frc2::CommandBase, TankDrive> {
 public:
  TankDrive(DriveTrain* drivetrain, frc::Joystick* stickRight, frc::Joystick* stickLeft);
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

 private:
  frc::Joystick* m_stickRight; 
  DriveTrain* m_drivetrain;
  frc::Joystick* m_stickLeft; 
};
=======
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>

#include "subsystems/DriveTrain.h"

/**
 * Have the robot drive tank style using the PS3 Joystick until interrupted.
 */
class TankDrive : public frc2::CommandHelper<frc2::CommandBase, TankDrive> {
 public:
  TankDrive(DriveTrain* drivetrain, frc::Joystick* stickRight, frc::Joystick* stickLeft);
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

 private:
  frc::Joystick* m_stickRight; 
  DriveTrain* m_drivetrain;
  frc::Joystick* m_stickLeft; 
};
>>>>>>> 119e8b76d8cbbb98afe2ba5b6ee254a4c9262844
