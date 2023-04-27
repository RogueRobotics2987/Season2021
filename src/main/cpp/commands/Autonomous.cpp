/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Autonomous.h"

#include <frc2/command/ParallelCommandGroup.h>

Autonomous::Autonomous( DriveTrain* drivetrain) {
  SetName("Autonomous");
  AddCommands();
  m_timer = new frc::Timer; 
  m_timer->Start();
  m_timer->Reset(); 
      // clang-format off     
  // clang-format on

  m_driveTrain = drivetrain;
  AddRequirements(m_driveTrain);

}

void Autonomous::Initialize(){
  state = 0;
  stop = false; 
  m_timer->Start();
  m_timer->Reset();
  //m_intake->resetBallOut(); 

}
void Autonomous::Execute(){

  }




void Autonomous::End(bool interrupted){
  m_driveTrain->Drive(0, 0);
}

bool Autonomous::IsFinished(){
  if(stop == true){
    return true;
  }
  else{
    return false; 
  }
}