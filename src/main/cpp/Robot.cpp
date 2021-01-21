/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <iostream>
#include "ctre/Phoenix.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"


using namespace frc;
using namespace std;



/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
class Robot : public frc::TimedRobot {
  
  WPI_TalonSRX FrontLeft = {12};
  WPI_TalonSRX FrontRight = {16};
  WPI_TalonSRX BackLeft = {14};
  WPI_TalonSRX BackRight = {15};
  WPI_TalonSRX Intake = {11};
  WPI_TalonSRX Arm1 = {23};
  WPI_TalonSRX Arm2 = {18};
  WPI_TalonSRX Reload = {13};
 
  frc::DifferentialDrive m_robotDrive{FrontLeft, FrontRight};
  frc::Joystick m_stick{0};
  bool BackForwards = false;
  
 // LimelightControl LimelightCam;

  void RobotInit() {
    BackLeft.Follow(FrontLeft);
    BackRight.Follow(FrontRight);
  }

  // frc::PWMVictorSPX m_leftMotor{0};
  // frc::PWMVictorSPX m_rightMotor{1};
  // frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};
  // frc::Joystick m_stick{0};

 public:
  void TeleopPeriodic() {
    if(m_stick.GetRawButton(1)){
    float Kp = 0.033 * 1.2;
    float Kpy = 0.033 * 1.5; 

    std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight-rr");
    // Compiler doesn't like the above line, but it works for
    //auto table = NetworkTableInstance::GetTable()
   // std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetTable("limelight-rr");
   // std::shared_ptr<NetworkTable> table = NetworkTableInstance::GetTable("bob");
    float tx = table->GetNumber("tx", 0.0);
    float ty = table->GetNumber("ty", 0.0);

    cout << tx << endl;
    cout << ty << endl;

    // Drive with arcade style
    //m_robotDrive.ArcadeDrive(-m_stick.GetY(), m_stick.GetZ());
    m_robotDrive.ArcadeDrive(Kpy * ty, Kp * tx);
    }
    
    else{
      if(m_stick.GetRawButton(2)) {
        if(BackForwards) {
          BackForwards = false;
        }
        else{
          BackForwards = true;
        }

      }
      if(BackForwards) {
        m_robotDrive.ArcadeDrive(-m_stick.GetY(), m_stick.GetZ());
      }
      else{
        m_robotDrive.ArcadeDrive(m_stick.GetY(), m_stick.GetZ());
      }
    if(m_stick.GetRawButton(3)) {
      Intake.Set(ControlMode::PercentOutput, 1);
  }
  else{
    Intake.Set(ControlMode::PercentOutput, 0);
  }
  
  if(m_stick.GetRawButton(7)) {
      Arm1.Set(ControlMode::PercentOutput, -0.5);
  }
  
  
  
  else if(m_stick.GetRawButton(9)) {
      Arm1.Set(ControlMode::PercentOutput, 0.5);
  }

  else{
  Arm1.Set(ControlMode::PercentOutput, 0);
  }
 
  
   if(m_stick.GetRawButton(8)) {
      Arm2.Set(ControlMode::PercentOutput, 0.5);
  }
  
  
  else if(m_stick.GetRawButton(10)) {
      Arm2.Set(ControlMode::PercentOutput, -0.5);
  }
  else{
    Arm2.Set(ControlMode::PercentOutput, 0);
     }
     if (m_stick.GetRawButton(4)) {
       Reload.Set(ControlMode::PercentOutput, 0.25);
     }
     else if(m_stick.GetRawButton(6)){
       Reload.Set(ControlMode::PercentOutput, -0.25);
     }
     else{
       Reload.Set(ControlMode::PercentOutput, 0);
     }
     if(m_stick.GetRawButton(5)){
       m_robotDrive.CurvatureDrive(m_stick.GetY(), m_stick.GetX(), false);
     }
    
  }
    }
    };

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); } 
#endif
