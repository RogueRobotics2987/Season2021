// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DifferentialDriveSubsystem.h"

DifferentialDriveSubsystem::DifferentialDriveSubsystem(){
  // Implementation of sumbsystem constructor method goes here.

  FrontLeft = new WPI_TalonSRX(12);
  FrontRight = new WPI_TalonSRX(16);
  BackLeft = new WPI_TalonSRX(14);
  BackRight = new WPI_TalonSRX(15);
 
  m_leftMoters = new frc::SpeedControllerGroup(*FrontLeft, *BackLeft);

 m_rightMoters = new frc::SpeedControllerGroup(*BackLeft, *FrontLeft);

  m_drive = new frc::DifferentialDrive(*m_rightMoters, *m_leftMoters);
};

void DifferentialDriveSubsystem::Periodic(){
    // Implementation of sumbsystem periodic method goes here.
}

void DifferentialDriveSubsystem::SimulationPeriodic(){
    // Implementation of sumbsystem simulation periodic method goes here.
}
void DifferentialDriveSubsystem::Drive(double y, double z) { 
// Control DriveTrain with joystick inut
m_drive->ArcadeDrive(-y, z);

}