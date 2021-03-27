/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveTrain.h"

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

  myAhrs = new AHRS(frc::SerialPort::kMXP); 
  m_robotDrive = new frc::DifferentialDrive(*LeftFront, *RightFront);
  // RightFront->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast); 
  // LeftFront->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast); 
  // RightBack->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast); 
  // LeftBack->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast); 
  LeftBack->Follow(*LeftFront); 
  RightBack->Follow(*RightFront); 

  // LeftEncoder.SetPosition(0); 
  // RightEncoder.SetPosition(0); 

  // RightFront->SetSmartCurrentLimit(5); 
  // LeftFront->SetSmartCurrentLimit(5); 

  m_odometry = new frc::DifferentialDriveOdometry{frc::Rotation2d(units::degree_t(GetHeading()))};

  DriveTrain::Reset();

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
//   frc::SmartDashboard::PutNumber("Left Distance", LeftEncoder.GetPosition());
//   frc::SmartDashboard::PutNumber("Right Distance", RightEncoder.GetPosition());
//  //frc::SmartDashboard::PutNumber("AHRS Heading", GetHeading()); 
  m_robotDrive->ArcadeDrive(-y, z);
  
}

void DriveTrain::autonDrive(){
  m_robotDrive->ArcadeDrive(-.4, 0); 
}

void DriveTrain::Periodic(){

  frc::SmartDashboard::PutNumber("Get Heading (ahrs)", myAhrs->GetAngle());
  frc::SmartDashboard::PutNumber("Get Heading (converted)", double(GetHeading()));
  
  frc::SmartDashboard::PutNumber("Output Voltage Right BusVolatage", RightFront->GetBusVoltage());
  frc::SmartDashboard::PutNumber("Output Voltage Left BusVoltage", LeftFront->GetBusVoltage());
  frc::SmartDashboard::PutNumber("Output Voltage Right GetApplied", RightFront->GetAppliedOutput());
  frc::SmartDashboard::PutNumber("Output Voltage Left GetApplied", LeftFront->GetAppliedOutput());


    m_odometry->Update(
      frc::Rotation2d(GetHeading()), 
      units::meter_t(LeftEncoder.GetPosition() * 0.044), 
      units::meter_t(-1.0 * RightEncoder.GetPosition() * 0.044)
      );
  
  frc::SmartDashboard::PutNumber("left Encoder Val", LeftEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("right Encoder Val", -1.0 * RightEncoder.GetPosition());

  m_field.SetRobotPose(m_odometry->GetPose());
  frc::SmartDashboard::PutData("Field", &m_field);




}
// void DriveTrain::TrajectoryInit(){ 
//   frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
//                                AutoConstants::kMaxAcceleration);
//  config.SetKinematics(DriveConstants::kDriveKinematics); 
//  //DriveConstants::kDriveKinematics; 
// }

void DriveTrain::TankDriveVolts(units::volt_t left, units::volt_t right) {
//   // if(left > units::volt_t(.25)){ left = units::volt_t(.25); }
//   // if(right > units::volt_t(.25)){ right = units::volt_t(.25); }
  frc::SmartDashboard::PutNumber("Left Distance", LeftEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Right Distance", RightEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Tank Drive Volts Left", double(left));
  frc::SmartDashboard::PutNumber("Tank Drive Volts Right", double(right));
  // frc::SmartDashboard::PutNumber("AHRS Heading", GetHeading()); 
  
  LeftFront->SetVoltage(left);
  RightFront->SetVoltage(-right);
  m_robotDrive->Feed();

}
units::degree_t DriveTrain::GetHeading() { 
  return units::degree_t(-1.0 * myAhrs->GetAngle()); // TODO: Fixed Units
}


void DriveTrain::Reset() {
  myAhrs->Reset();
  LeftEncoder.SetPosition(0.0);
  RightEncoder.SetPosition(0.0);
} 

frc::DifferentialDriveWheelSpeeds DriveTrain::GetWheelSpeeds() {
  // units::meter_t(LeftEncoder.GetPosition() * 0.044), 
  // units::meter_t(-1.0 * RightEncoder.GetPosition() * 0.044)
  return {(LeftEncoder.GetVelocity() * 1_mps * 0.044 / 60),
      (-RightEncoder.GetVelocity() * 1_mps * 0.044 / 60)};
}

void DriveTrain::ResetOdometry(frc::Pose2d pose){ 
  Reset(); //reset encoders and ahrs  
  m_odometry->ResetPosition(pose, frc::Rotation2d(units::degree_t(GetHeading()))); 
}

frc::Pose2d DriveTrain::GetPose(){
  return m_odometry->GetPose();
}

void DriveTrain::ResetEncoders(){ 
  RightEncoder.SetPosition(0); 
  LeftEncoder.SetPosition(0); 
}
