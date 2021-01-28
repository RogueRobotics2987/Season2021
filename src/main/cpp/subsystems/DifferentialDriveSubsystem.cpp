// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DifferentialDriveSubsystem.h"

<<<<<<< HEAD
DifferentialDriveSubsystem::DifferentialDriveSubsystem(){
  // Implementation of sumbsystem constructor method goes here.

=======
DifferentialDriveSubsystem::DifferentialDriveSubsystem() {
  // Implementation of subsystem constructor goes here.
  
>>>>>>> dcd3e367e17464515bc9e8b8b0327aae4e8d57ef
  FrontLeft = new WPI_TalonSRX(12);
  FrontRight = new WPI_TalonSRX(16);
  BackLeft = new WPI_TalonSRX(14);
  BackRight = new WPI_TalonSRX(15);
<<<<<<< HEAD
 
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

=======

   m_leftMotors = new frc::SpeedControllerGroup(*FrontLeft, *BackLeft);

   m_rightMotors = new frc::SpeedControllerGroup(*FrontRight, *BackRight);

   m_drive = new frc::DifferentialDrive(*m_leftMotors, *m_rightMotors);

};

void DifferentialDriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void DifferentialDriveSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
// Control drive train with joystick input
void DifferentialDriveSubsystem::Drive(double y, double z) {
  m_drive->ArcadeDrive(-y, z);
>>>>>>> dcd3e367e17464515bc9e8b8b0327aae4e8d57ef
}