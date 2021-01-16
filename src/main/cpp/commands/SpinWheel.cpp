<<<<<<< HEAD
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SpinWheel.h"

SpinWheel::SpinWheel(ColorSensor* mSensor):cSensor(mSensor) {
  AddRequirements(cSensor);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SpinWheel::Initialize() {
  cSensor->ResetNums();
  cSensor->resetTimer(); 

}



// Called repeatedly when this Command is scheduled to run
void SpinWheel::Execute() {
  cSensor->SetMotor(.5); 

}

// Called once the command ends or is interrupted.
void SpinWheel::End(bool interrupted) {
  cSensor->StopMotor();
}

// Returns true when the command should end.
bool SpinWheel::IsFinished() { 
  //Color wheel using time 
    // if(cSensor->GetTime() > 4){
    //   return true; 
    // }
    // else{
    //   return false; 
    // }

    //Color wheel using color counts 
    // if(cSensor->SpinNum() < 7){
    // return false; 
    // }
    // else{
    // return true; 
    // }
    

=======
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SpinWheel.h"

SpinWheel::SpinWheel(ColorSensor* mSensor):cSensor(mSensor) {
  AddRequirements(cSensor);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void SpinWheel::Initialize() {
  cSensor->ResetNums();
  cSensor->resetTimer(); 

}



// Called repeatedly when this Command is scheduled to run
void SpinWheel::Execute() {
  cSensor->SetMotor(.5); 

}

// Called once the command ends or is interrupted.
void SpinWheel::End(bool interrupted) {
  cSensor->StopMotor();
}

// Returns true when the command should end.
bool SpinWheel::IsFinished() { 
  //Color wheel using time 
    // if(cSensor->GetTime() > 4){
    //   return true; 
    // }
    // else{
    //   return false; 
    // }

    //Color wheel using color counts 
    if(cSensor->SpinNum() < 7){
    return false; 
    }
    else{
    return true; 
    }
    

>>>>>>> 119e8b76d8cbbb98afe2ba5b6ee254a4c9262844
 }