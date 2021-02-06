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
  RightBack = new rev::CANSparkMax(50, rev::CANSparkMax::MotorType::kBrushless);
  RightFront = new rev::CANSparkMax(46, rev::CANSparkMax::MotorType::kBrushless);
  myAhrs = new AHRS(frc::SerialPort::kMXP); 
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


  //SYDNEYS TRAJECTORY VALUES
  leftEncoder = new rev::CANEncoder(*LeftFront); 
  rightEncoder = new rev::CANEncoder(*RightFront); 
  

  // Let's show everything on the LiveWindow
  // AddChild("Front_Left Motor", &m_frontLeft);
  // AddChild("Rear Left Motor", &m_rearLeft);
  // AddChild("Front Right Motor", &m_frontRight);
  // AddChild("Rear Right Motor", &m_rearRight);
  // AddChild("Left Encoder", &m_leftEncoder);
  // AddChild("Right Encoder", &m_rightEncoder);
  // AddChild("Rangefinder", &m_rangefinder);
  // AddChild("Gyro", &m_gyro);

  // frc::SmartDashboard::PutNumber("Get Heading", float(GetHeading()));  
  frc::SmartDashboard::PutNumber("Get Heading", myAhrs->GetAngle());
  frc::SmartDashboard::PutNumber("Set DriveP", kp);
  frc::SmartDashboard::PutNumber("Set DriveI", ki);
  frc::SmartDashboard::PutNumber("Set DriveD", kd);
  frc::SmartDashboard::PutNumber("Set DriveFF", kff);
  frc::SmartDashboard::PutNumber("Set Rotations", rotations);
  leftdrivePID = new rev::CANPIDController(*LeftFront);
  rightdrivePID = new rev::CANPIDController(*RightFront); 
  leftdrivePID->SetP(kp);
  leftdrivePID->SetI(ki);
  leftdrivePID->SetD(kd);
  leftdrivePID->SetFF(kff);
  rightdrivePID->SetP(kp);
  rightdrivePID->SetI(ki);
  rightdrivePID->SetD(kd);
  rightdrivePID->SetFF(kff);
  // leftdrivePID->SetOutputRange(-1, 1);
  // leftdrivePID->SetSmartMotionMaxVelocity(4000);
  // leftdrivePID->SetSmartMotionMinOutputVelocity(1500);
  // leftdrivePID->SetSmartMotionMaxAccel(1000.0/1.0);
  // leftdrivePID->SetSmartMotionAllowedClosedLoopError(0.0);
  // leftdrivePID->SetIZone(800);
  // leftdrivePID->SetFF(0.7/3500
  // rightdrivePID->SetOutputRange(-1, 1);
  // rightdrivePID->SetSmartMotionMaxVelocity(4000);
  // rightdrivePID->SetSmartMotionMinOutputVelocity(1500);
  // rightdrivePID->SetSmartMotionMaxAccel(1000.0/1.0);
  // rightdrivePID->SetSmartMotionAllowedClosedLoopError(0.0);
  // rightdrivePID->SetIZone(800);
  // rightdrivePID->SetFF(0.7/3500);






}

void DriveTrain::Log() {
  


  // frc::SmartDashboard::PutNumber("Left Speed", m_leftEncoder.GetRate());
  // frc::SmartDashboard::PutNumber("Right Speed", m_rightEncoder.GetRate());
  // frc::SmartDashboard::PutNumber("Gyro", m_gyro.GetAngle());
}


// JOYSTICK DRIVE FUNCTION
void DriveTrain::Drive(double y, double z) { 
  
// leftFrontEncoder->SetDistancePerPulse();
// frc::SmartDashboard::PutNumber("Right Voltage Output", RightFront->GetAppliedOutput());
// frc::SmartDashboard::PutNumber("Left Voltage Output", LeftFront->GetAppliedOutput());
frc::SmartDashboard::PutNumber("Left Distance", leftEncoder->GetPosition());
frc::SmartDashboard::PutNumber("Right Distance", rightEncoder->GetPosition());
// frc::SmartDashboard::PutNumber("AHRS Heading", GetHeading()); 
  m_robotDrive->ArcadeDrive(-y, z);
  
}



void DriveTrain::autonDrive(){
  m_robotDrive->ArcadeDrive(-.4, 0); 
}

void DriveTrain::AutoDrive() {
  double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);
  kp = frc::SmartDashboard::GetNumber("Set DriveP", kp);
  ki = frc::SmartDashboard::GetNumber("Set DriveI", ki);
  kd = frc::SmartDashboard::GetNumber("Set DriveD", kd);
  kff = frc::SmartDashboard::GetNumber("Set DriveFF", kff);

  if(lastkp != kp) {
    leftdrivePID->SetP(kp);
    rightdrivePID->SetP(kp);
    lastkp = kp; 
  } 
  if(lastki != ki) {
    leftdrivePID->SetI(ki);
    rightdrivePID->SetI(ki);
    lastki = ki; 
  }
  if(lastkd != kd) {
    leftdrivePID->SetD(kd);
    rightdrivePID->SetD(kd);
    lastkd = kd; 
  }
  if(lastkff != kff) {
    leftdrivePID->SetFF(kff);
    rightdrivePID->SetFF(kff);
    lastkff = kff; 
  }

  leftdrivePID->SetReference(rotations, rev::ControlType::kPosition);
  rightdrivePID->SetReference(rotations, rev::ControlType::kPosition);
}

void DriveTrain::Periodic(){
  frc::SmartDashboard::PutNumber("Get Heading (ahrs)", myAhrs->GetAngle());
  frc::SmartDashboard::PutNumber("Get Heading (converted)", double(GetHeading()));

    m_odometry->Update(
      frc::Rotation2d(GetHeading()), 
      units::meter_t(leftEncoder->GetPosition() * 0.044), 
      units::meter_t(-1.0 * rightEncoder->GetPosition() * 0.044)
      );
  
  frc::SmartDashboard::PutNumber("left Encoder Val", leftEncoder->GetPosition());
  frc::SmartDashboard::PutNumber("right Encoder Val", -1.0 * rightEncoder->GetPosition());

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

  frc::SmartDashboard::PutNumber("Volts Left", double(left));
  frc::SmartDashboard::PutNumber("Volts right", double(right));

//BRANDONS OLD CODE
//   // if(left > units::volt_t(.25)){ left = units::volt_t(.25); }
//   // if(right > units::volt_t(.25)){ right = units::volt_t(.25); }
//   frc::SmartDashboard::PutNumber("Right Voltage Output", RightFront->GetAppliedOutput());
//   frc::SmartDashboard::PutNumber("Left Voltage Output", LeftFront->GetAppliedOutput());
//   frc::SmartDashboard::PutNumber("Left Distance", leftEncoder->GetPosition());
//   frc::SmartDashboard::PutNumber("Right Distance", rightEncoder->GetPosition());
//   frc::SmartDashboard::PutNumber("AHRS Heading", GetHeading());  


  LeftFront->SetVoltage(left);
  RightFront->SetVoltage(-1.0 * right); // This is probably not right...
  m_robotDrive->Feed();

}
units::degree_t DriveTrain::GetHeading() { 
        return units::degree_t(-1.0 * myAhrs->GetAngle()); 
}

void DriveTrain::Reset() {
  myAhrs->Reset();
  leftEncoder->SetPosition(0.0);
  rightEncoder->SetPosition(0.0);
}

// double DriveTrain::GetDistance() {
//   // return (m_leftEncoder.GetDistance() + m_rightEncoder.GetDistance()) / 2.0;
//   return 0;
// }

frc::DifferentialDriveWheelSpeeds DriveTrain::GetWheelSpeeds() {
return {units::meters_per_second_t(-leftEncoder->GetVelocity()),
        units::meters_per_second_t(rightEncoder->GetVelocity())};
}



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

frc::Pose2d DriveTrain::GetPose(){
  // m_odometry->GetPose().X();
  return m_odometry->GetPose(); 
}

// void DriveTrain::ResetEncoders(){ 
//   rightEncoder->SetPosition(0); 
//   leftEncoder->SetPosition(0); 
// }
