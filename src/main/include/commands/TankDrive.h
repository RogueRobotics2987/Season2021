#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>

#include "subsystems/DifferentialDriveSubsystem.h"


class TankDrive : public frc2::CommandHelper<frc2::CommandBase, TankDrive> {
 public:
  TankDrive(DifferentialDriveSubsystem* drivetrain, frc::Joystick* stickRight, frc::Joystick* stickLeft);
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

 private:
  frc::Joystick* m_stickRight; 
  DifferentialDriveSubsystem* m_drivetrain;
  frc::Joystick* m_stickLeft; 
};
