/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ShootCmdCls.h"

ShootCmdCls::ShootCmdCls(Shooter* a_shooter /*, frc::Joystick* a_joystick*/) {
  m_shooter= a_shooter;
  AddRequirements(m_shooter);
  //m_joystick = a_joystick;
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ShootCmdCls::Initialize() {
 //   m_shooter->SetPercentV(0.15);
 cout<<"its running"<<endl;

}

// Called repeatedly when this Command is scheduled to run
void ShootCmdCls::Execute() {
// OLD
//  m_shooter->SetPercentV(0.15);
  m_shooter->startShooter();
}

// Called once the command ends or is interrupted.
void ShootCmdCls::End(bool interrupted) {
 // m_shooter->SetPercentV(0.0);
 m_shooter->stopShooter();
}

// Returns true when the command should end.
bool ShootCmdCls::IsFinished() {
//  if (m_joystick->GetRawButton(2)) {
//     return false; 
//  } else {
//    return true;
//  }
  return false;
 }
