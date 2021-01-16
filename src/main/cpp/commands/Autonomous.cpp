/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Autonomous.h"

#include <frc2/command/ParallelCommandGroup.h>

Autonomous::Autonomous( DriveTrain* drivetrain, Shooter* shooter, ShooterActuator* shooteractuator, Intake* intake) {
  SetName("Autonomous");
  AddCommands();
  m_timer = new frc::Timer; 
  m_timer->Start();
  m_timer->Reset(); 
      // clang-format off     
  // clang-format on

  m_driveTrain = drivetrain;
  m_intake = intake;
  m_shooter = shooter;
  m_shooterActuator = shooteractuator;
  AddRequirements(m_driveTrain);
  AddRequirements(m_intake);
  AddRequirements(m_shooterActuator);
  AddRequirements(m_shooter);

}

void Autonomous::Initialize(){
  state = 0;
  stop = false; 
  m_timer->Start();
  m_timer->Reset();
  //m_intake->resetBallOut(); 

}
void Autonomous::Execute(){
  if(m_shooter->getVelocity() < 500){
      m_shooter->setPercent(.5);
    }
    else{
      m_shooter->setShooter(3800); 
    }
    

  if(state == 0){
    shootTime = m_timer->Get(); 
    state++; 
  }

  if(state == 1){
    m_shooterActuator->SetAutoAim(true);
    m_shooterActuator->setAngleH(0); 
    m_shooterActuator->setAngleV(0); 
    if(m_shooter->getVelocity() >= .95 * 3800 && m_shooterActuator->GetTX() < 1 && m_shooterActuator->GetTX() > -1 && m_shooterActuator->GetTY() < 1 && m_shooterActuator->GetTY() > -1){
      m_intake->StartConveyor(.5);
    }

    if(m_timer->Get() - shootTime >= 6){
      state++; 
    }

  }

  if(state == 2){
    m_intake->setSolenoidTrue(); 
    driveTime = m_timer->Get(); 
    m_intake->ResetBallCount(); 
    state++; 
    m_intake->StartConveyor(0); 
    }

  if(state == 3){
    m_shooterActuator->SetAutoAim(true);
    m_shooterActuator->setAngleH(0); 
    m_shooterActuator->setAngleV(0);
    m_driveTrain->autonDrive(); 
    m_intake->IntakeBall(.65); 
    //m_intake->StartConveyor(.6); 
    //m_intake->PrepareBall(); 
    if(m_timer->Get() - driveTime > 4){
      state++; 
      driveTime = m_timer->Get(); 
    }
  }
  if(state == 4){
    m_shooterActuator->SetAutoAim(true);
    m_shooterActuator->setAngleH(0); 
    m_shooterActuator->setAngleV(0); 
    if(m_shooter->getVelocity() >= .95 * 3800 && m_shooterActuator->GetTX() < 2 && m_shooterActuator->GetTX() > -2 && m_shooterActuator->GetTY() < 2 && m_shooterActuator->GetTY() > -2){
      m_intake->StartConveyor(.5);
    }
  if(m_timer->Get() - driveTime < .5){
    m_driveTrain->Drive(0, .3); 
  }
  else{
  m_driveTrain->Drive(0, 0);}
  }

  
  }




void Autonomous::End(bool interrupted){
  m_intake->ResetBallCount(); 
  m_shooter->stopShooter(); 
  m_intake->StopMotors(); 
  m_driveTrain->Drive(0, 0);
  m_intake->StartConveyor(0); 
}

bool Autonomous::IsFinished(){
  if(stop == true){
    return true;
  }
  else{
    return false; 
  }
}