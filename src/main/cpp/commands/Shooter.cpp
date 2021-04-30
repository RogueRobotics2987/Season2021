/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Shooter.h"
#include <iostream>

Shooter::Shooter(ShooterSubsystem* m_Shooter):m_Shooter(m_Shooter) {
  // m_Shooter = m_Shooter;
  AddRequirements(m_Shooter);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void Shooter::Initialize() {
  
}



// Called repeatedly when this Command is scheduled to run
void Shooter::Execute() {
  if (!Shooting){
      m_Shooter->Open(TubeCounter);
    Shooting = true;

  } else {
      Timer = Timer + 1;
  }
  
}

// Called once the command ends or is interrupted.
void Shooter::End(bool interrupted) {
}

// Returns true when the command should end.
bool Shooter::IsFinished() {
  if (Timer > MaxTime){
      m_Shooter->Close(TubeCounter);
      if (TubeCounter >= 5){
            TubeCounter = 1;
        } else {
            TubeCounter = TubeCounter + 1;
        } 
        Shooting = false;
        Timer = 0;
        return true;
    
  } else {
      return false;
  }

    
    

 }