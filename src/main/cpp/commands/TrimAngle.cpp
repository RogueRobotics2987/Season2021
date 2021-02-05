/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <frc/smartdashboard/SmartDashboard.h> 
#include "commands/TrimAngle.h"
#include <iostream>
TrimAngle::TrimAngle(frc::Joystick* cStick, ShooterActuator* a_actuator, frc::Joystick* cJoy) {
  m_actuator = a_actuator;
  mJoy = cJoy; 
  //Pass the Constructor value into the real value
  rStick = cStick;
  AddRequirements(m_actuator);
  SetName("TrimAngle");
  // Use addRequirements() here to declare subsystem dependencies.
  myTimer = new frc::Timer; 
  myTimer -> Reset();
  myTimer -> Start();

}

// Called when the command is initially scheduled.
void TrimAngle::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void TrimAngle::Execute() {
      double startTime = myTimer->Get();

      frc::SmartDashboard::PutNumber("Left X", rStick->GetRawAxis(0));
      frc::SmartDashboard::PutNumber("Left Y", rStick->GetRawAxis(1));
      frc::SmartDashboard::PutNumber("Right X", rStick->GetRawAxis(4));
      frc::SmartDashboard::PutNumber("Right Y", rStick->GetRawAxis(5)); 
      m_actuator->setAngleH(rStick->GetRawAxis(0)); // updated button
      m_actuator->setAngleV(rStick->GetRawAxis(1)); // updated button
      
      // switches pipeline using xbox control. (remove x's)
      if(rStick -> GetPOV(x)){
        m_actuator -> limeStream(1);
      } else if(rStick -> GetPOV(xx)){
        m_actuator -> limeStream(2);
      } else if(rStick -> GetPOV(xxx)){
        m_actuator -> limeStream(3);
      } else if(rStick -> GetPOV(xxxx)){
        m_actuator -> limeStream(4);
      } else {
        m_actuator -> limeStream(2);
      }

      m_actuator->SetAutoAim(rStick->GetRawButton(7)); // updated button
      if(mJoy->GetRawButton(2) && run){
        if(camVal){
          camVal = false; 
          run = false; 
        }
        else{
          camVal = true; 
          run = false; 
        }
        }
        else if(!mJoy->GetRawButton(2)){
          run = true; 
        }
      m_actuator->switchCam(camVal); 
      if(1) {
        std::cout << "TrimAngle Execute Time: " << myTimer->Get() - startTime << std::endl;
      }


      //frc::SmartDashboard::PutBoolean("xBox Button 5", rStick->GetRawButton(5));
      
  //frc::SmartDashboard::PutNumber("Set angle", rStick); 
}

// Called once the command ends or is interrupted.
void TrimAngle::End(bool interrupted) {
}

// Returns true when the command should end.
bool TrimAngle::IsFinished() { 
 return false;
}
