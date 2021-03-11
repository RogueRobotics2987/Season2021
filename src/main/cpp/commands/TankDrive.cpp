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

}
  void TankDrive::Initialize() {
    m_drivetrain->Reset();
  }


// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() { 
  
  double Left = m_stickLeft ->GetY();   // Gets Y-position of joystick
  double Right = m_stickRight ->GetX(); // X-position of joystick
  static double lastleft = 0.0;
  static double lastright = 0.0;
  static double outleft = 0.0;
  static double outright = 0.0;
  double maxChange = 0.025;
  
  // gradually changes speed
  if (abs(Left-lastleft)>maxChange){
    outleft = lastleft + copysignf(1.0, Left-lastleft)*maxChange;
  } else{
      outleft = Left;
  }
  if (abs(Right-lastright)>maxChange){
    outright = lastright + copysignf(1.0, Right-lastright)*maxChange;
  } else{
      outright = Right;
  }

  m_drivetrain -> Drive(outleft, outright);

  lastleft = outleft;
  lastright = outright;

  /*if (m_stickLeft->GetRawButton(2)) { 
    m_drivetrain->Drive(-m_stickLeft ->GetY(), m_stickRight ->GetX());
  } else {
    m_drivetrain->Drive(m_stickLeft ->GetY(), m_stickRight ->GetX()); 
  } */
}

// Make this return true when this Command no longer needs to run execute()
bool TankDrive::IsFinished() { return false; }

// Called once after isFinished returns true
void TankDrive::End(bool) { m_drivetrain->Drive(0, 0);}
