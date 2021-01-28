// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ctre/Phoenix.h"
#include "frc/SpeedControllerGroup.h"
#include <frc/drive/DifferentialDrive.h>

class DifferentialDriveSubsystem : public frc2::SubsystemBase {
 public:
  DifferentialDriveSubsystem();

  /**
   * The log method puts interesting information to the SmartDashboard.
   */

   void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  void Drive(double y, double z) { 
  // Control DriveTrain with joystick inut  
}

 private:
 // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
WPI_TalonSRX* FrontLeft;
WPI_TalonSRX* FrontRight;
WPI_TalonSRX* BackLeft;
WPI_TalonSRX* BackRight;

frc::SpeedControllerGroup* m_leftMoters;

frc::SpeedControllerGroup* m_rightMoters;

frc::DifferentialDrive* m_drive;

};
