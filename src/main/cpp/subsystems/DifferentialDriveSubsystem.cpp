// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DifferentialDriveSubsystem.h"
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "AHRS.h"
#include "Constants.h" 
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RamseteCommand.h>




  


// coreys code is now different than brandons!
DifferentialDriveSubsystem::DifferentialDriveSubsystem() {
// Encoders may measure differently in the real world and in
// simulation. In this example the robot moves 0.042 barleycorns
// per tick in the real world, but the simulated encoders
// simulate 360 tick encoders. This if statement allows for the
// real robot to handle this difference in devices.
#ifndef SIMULATION
  // m_leftEncoder.SetDistancePerPulse(0.042);
  // m_rightEncoder.SetDistancePerPulse(0.042);
#else
  // Circumference in ft = 4in/12(in/ft)*PI
  // m_leftEncoder.SetDistancePerPulse(static_cast<double>(4.0 / 12.0 * M_PI) /
  //                                   360.0);
  // m_rightEncoder.SetDistancePerPulse(static_cast<double>(4.0 / 12.0 * M_PI) /
  //                                    360.0);
#endif
  SetName("DifferentialDriveSubsystem");
  Motor = new WPI_TalonSRX(12);
  Motor2 = new WPI_TalonSRX(16);
  Motor3 = new WPI_TalonSRX(14);
  Motor4 = new WPI_TalonSRX(15);
 DriverFront = new frc::DifferentialDrive(*Motor2, *Motor);
 DriverFront = new frc::DifferentialDrive(*Motor3, *Motor4);
  
  // Let's show everything on the LiveWindow
  // AddChild("Front_Left Motor", &m_frontLeft);
  // AddChild("Rear Left Motor", &m_rearLeft);
  // AddChild("Front Right Motor", &m_frontRight);
  // AddChild("Rear Right Motor", &m_rearRight);
  // AddChild("Left Encoder", &m_leftEncoder);
  // AddChild("Right Encoder", &m_rightEncoder);
  // AddChild("Rangefinder", &m_rangefinder);
  // AddChild("Gyro", &m_gyro);
}

void DifferentialDriveSubsystem::Log() {
  


  // frc::SmartDashboard::PutNumber("Left Speed", m_leftEncoder.GetRate());
  // frc::SmartDashboard::PutNumber("Right Speed", m_rightEncoder.GetRate());
  // frc::SmartDashboard::PutNumber("Gyro", m_gyro.GetAngle());
}

void DifferentialDriveSubsystem::Drive(double y, double z) { 
  
  //leftFrontEncoder->SetDistancePerPulse();
//   frc::SmartDashboard::PutNumber("Right Voltage Output", RightFront->GetAppliedOutput());
//   frc::SmartDashboard::PutNumber("Left Voltage Output", LeftFront->GetAppliedOutput());
//   frc::SmartDashboard::PutNumber("Left Distance", leftEncoder->GetPosition());
//   frc::SmartDashboard::PutNumber("Right Distance", rightEncoder->GetPosition());
//  //frc::SmartDashboard::PutNumber("AHRS Heading", GetHeading()); 
  m_robotDrive->ArcadeDrive(-y, z);
  
}

void DifferentialDriveSubsystem::autonDrive(){
  m_robotDrive->ArcadeDrive(-.4, 0); 
}

void DifferentialDriveSubsystem::Periodic(){
    //m_odometry->Update(frc::Rotation2d(units::degree_t(GetHeading())), units::meter_t(leftEncoder->GetPosition()), units::meter_t(rightEncoder->GetPosition()));
}