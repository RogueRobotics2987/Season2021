/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TankDrive.h"

#include "Robot.h"


TankDrive::TankDrive(DriveTrain* drivetrain, frc::Joystick* stickRight, frc::Joystick* stickLeft)
    {
  
  m_drivetrain = drivetrain; 
  m_stickLeft = stickLeft; 
  m_stickRight = stickRight; 
  SetName("TankDrive");
  AddRequirements({m_drivetrain});
  frc::SmartDashboard::PutBoolean("Acceleration Control", true); 
  
}

  void TankDrive::Initialize() {
    m_drivetrain->Reset();
  }


// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() { 
  //uncomment lines 33-62 to put acceleration control back in 
  static double lastLeft = 0.0; 
  static double lastRight = 0.0;

  double Left = m_stickLeft -> GetY(); //getting the Y value from the joystick
  double Right = m_stickRight -> GetX(); //comment
  double outLeft = 0;
  double outRight = 0;
  double maxChange = 0.04; //per second
 
 frc::SmartDashboard::PutNumber("lastLeft Value", lastLeft);
 frc::SmartDashboard::PutNumber("Left value", Left);
 frc::SmartDashboard::PutNumber("lastRight Value", lastRight);
 frc::SmartDashboard::PutNumber("Right value", Right);
 frc::SmartDashboard::GetNumber("maxChange", maxChange); 
 maxChange = frc::SmartDashboard::GetNumber("maxChange", maxChange); 

 bool accelCtrl = false; 
 accelCtrl = frc::SmartDashboard::GetBoolean("Acceleration Control", false); 

  if (abs(Left-lastLeft) >maxChange && accelCtrl) {
    outLeft = lastLeft + copysignf(1.0,Left - lastLeft)*maxChange;
    } else {
      outLeft = Left;
  }
  if (abs(Right-lastRight) >maxChange && accelCtrl) {
    outRight = lastRight + copysignf(1.0,Right - lastRight)*maxChange;
    } else {
      outRight = Right;
  }
  
  m_drivetrain -> Drive(outLeft, outRight);
   lastLeft = outLeft;
  lastRight = outRight; 

  //comment out lines 64-69 if you put acceleration back in
  /*if (m_stickLeft->GetRawButton(2)) {
    m_drivetrain->Drive(-m_stickLeft->GetY(), m_stickRight->GetX()); 
  } else {
    m_drivetrain->Drive(m_stickLeft->GetY(), m_stickRight->GetX()); 
  } */
  
  }

// Make this return true when this Command no longer needs to run execute()
bool TankDrive::IsFinished() { return false; }

// Called once after isFinished returns true
void TankDrive::End(bool) { m_drivetrain->Drive(0, 0);}
