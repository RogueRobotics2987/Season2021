/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveTrain.h"
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "AHRS.h"
#include "Constants.h" 
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RamseteCommand.h>




// coreys code is now different than brandons!
DriveTrain::DriveTrain() {
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
  SetName("DriveTrain");
  LeftBack = new rev::CANSparkMax(56, rev::CANSparkMax::MotorType::kBrushless);
  LeftFront = new rev::CANSparkMax(49, rev::CANSparkMax::MotorType::kBrushless);
  leftEncoder = new rev::CANEncoder(*LeftFront); 
  RightBack = new rev::CANSparkMax(50, rev::CANSparkMax::MotorType::kBrushless);
  RightFront = new rev::CANSparkMax(46, rev::CANSparkMax::MotorType::kBrushless);
  rightEncoder = new rev::CANEncoder(*RightFront); 
  //myAhrs = new AHRS(frc::SerialPort::kMXP); 
  m_robotDrive = new frc::DifferentialDrive(*LeftFront, *RightFront);
  m_odometry = new frc::DifferentialDriveOdometry{frc::Rotation2d(units::degree_t(GetHeading()))};
  // RightFront->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast); 
  // LeftFront->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast); 
  // RightBack->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast); 
  // LeftBack->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast); 

  // leftEncoder->SetPosition(0); 
  // rightEncoder->SetPosition(0); 
  // RightFront->SetSmartCurrentLimit(5); 
  // LeftFront->SetSmartCurrentLimit(5); 
  LeftBack->Follow(*LeftFront); 
  RightBack->Follow(*RightFront); 

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

void DriveTrain::Log() {
  


  // frc::SmartDashboard::PutNumber("Left Speed", m_leftEncoder.GetRate());
  // frc::SmartDashboard::PutNumber("Right Speed", m_rightEncoder.GetRate());
  // frc::SmartDashboard::PutNumber("Gyro", m_gyro.GetAngle());
}

void DriveTrain::Drive(double y, double z) { 
  
  //leftFrontEncoder->SetDistancePerPulse();
//   frc::SmartDashboard::PutNumber("Right Voltage Output", RightFront->GetAppliedOutput());
//   frc::SmartDashboard::PutNumber("Left Voltage Output", LeftFront->GetAppliedOutput());
//   frc::SmartDashboard::PutNumber("Left Distance", leftEncoder->GetPosition());
//   frc::SmartDashboard::PutNumber("Right Distance", rightEncoder->GetPosition());
//  //frc::SmartDashboard::PutNumber("AHRS Heading", GetHeading()); 
  m_robotDrive->ArcadeDrive(-y, z);
  
}

void DriveTrain::autonDrive(){
  m_robotDrive->ArcadeDrive(-.4, 0); 
}

void DriveTrain::Periodic(){
  frc::SmartDashboard::PutNumber("Get Heading (ahrs)", myAhrs->GetAngle());
  frc::SmartDashboard::PutNumber("Get Heading (converted)", double(GetHeading()));

    m_odometry->Update(
      frc::Rotation2d(GetHeading()), 
      units::meter_t(leftEncoder->GetPosition() * 0.044), 
      units::meter_t(rightEncoder->GetPosition() * 0.044)
      );
  
  frc::SmartDashboard::PutNumber("left Encoder Val", leftEncoder->GetPosition() * 0.044);
  frc::SmartDashboard::PutNumber("right Encoder Val", rightEncoder->GetPosition() * 0.044);

  m_field.SetRobotPose(m_odometry->GetPose());
  frc::SmartDashboard::PutData("Field", &m_field);




}
// void DriveTrain::TrajectoryInit(){ 
//   frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
//                                AutoConstants::kMaxAcceleration);
//  config.SetKinematics(DriveConstants::kDriveKinematics); 
//  //DriveConstants::kDriveKinematics; 
// }

// void DriveTrain::TankDriveVolts(units::volt_t left, units::volt_t right) {
//   // if(left > units::volt_t(.25)){ left = units::volt_t(.25); }
//   // if(right > units::volt_t(.25)){ right = units::volt_t(.25); }
//   frc::SmartDashboard::PutNumber("Right Voltage Output", RightFront->GetAppliedOutput());
//   frc::SmartDashboard::PutNumber("Left Voltage Output", LeftFront->GetAppliedOutput());
//   frc::SmartDashboard::PutNumber("Left Distance", leftEncoder->GetPosition());
//   frc::SmartDashboard::PutNumber("Right Distance", rightEncoder->GetPosition());
//   frc::SmartDashboard::PutNumber("AHRS Heading", GetHeading()); 
  
//   LeftFront->SetVoltage(left);
//   RightFront->SetVoltage(-right);
//   m_robotDrive->Feed();

// }
units::degree_t DriveTrain::GetHeading() { 
  return units::degree_t(myAhrs->GetAngle() * 360.0 / 2 / M_PI); // TODO: Fixed Units
}


void DriveTrain::Reset() {
  myAhrs->Reset();
  leftEncoder->SetPosition(0.0);
  rightEncoder->SetPosition(0.0);

} 

// void DriveTrain::Reset() {
//   // m_gyro.Reset();
//   // m_leftEncoder.Reset();
//   // m_rightEncoder.Reset();
// }

// double DriveTrain::GetDistance() {
//   // return (m_leftEncoder.GetDistance() + m_rightEncoder.GetDistance()) / 2.0;
//   return 0;
// }

// frc::DifferentialDriveWheelSpeeds DriveTrain::GetWheelSpeeds() {
// return {(-leftEncoder->GetVelocity() * 1_mps),(rightEncoder->GetVelocity() * 1_mps)};
// }



// double DriveTrain::GetDistanceToObstacle() {
//   // Really meters in simulation since it's a rangefinder...
//   // return m_rangefinder.GetAverageVoltage();
//   return 0;
// }

// double DriveTrain::GetTurnRate(){ 
//   return myAhrs->GetRate(); 
// }

// void DriveTrain::ResetOdometry(frc::Pose2d pose){ 
//   ResetEncoders(); 
//   m_odometry->ResetPosition(pose, frc::Rotation2d(units::degree_t(GetHeading()))); 
// }

// frc::Pose2d DriveTrain::GetPose(){
//   //return m_odometry->GetPose(); 
// }

// void DriveTrain::ResetEncoders(){ 
//   rightEncoder->SetPosition(0); 
//   leftEncoder->SetPosition(0); 
// }
