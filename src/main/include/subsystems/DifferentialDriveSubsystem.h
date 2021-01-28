// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

<<<<<<< HEAD
#include <frc2/command/SubsystemBase.h>
#include "ctre/Phoenix.h"
#include "frc/SpeedControllerGroup.h"
#include <frc/drive/DifferentialDrive.h>
=======
#include "ctre/Phoenix.h"
#include <frc2/command/SubsystemBase.h>
#include <frc/drive/DifferentialDrive.h>
#include "frc/SpeedControllerGroup.h"

>>>>>>> dcd3e367e17464515bc9e8b8b0327aae4e8d57ef

class DifferentialDriveSubsystem : public frc2::SubsystemBase {
 public:
  DifferentialDriveSubsystem();

  /**
<<<<<<< HEAD
   * The log method puts interesting information to the SmartDashboard.
   */

   void Periodic() override;
=======
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
>>>>>>> dcd3e367e17464515bc9e8b8b0327aae4e8d57ef

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

<<<<<<< HEAD
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
=======
  // Control drive train with joystick input
  void Drive(double y, double z);
    


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  WPI_TalonSRX* FrontLeft;
  WPI_TalonSRX* FrontRight;
  WPI_TalonSRX* BackLeft;
  WPI_TalonSRX* BackRight;


  frc::SpeedControllerGroup* m_leftMotors;

  frc::SpeedControllerGroup* m_rightMotors;

  frc::DifferentialDrive* m_drive;
 };
>>>>>>> dcd3e367e17464515bc9e8b8b0327aae4e8d57ef
