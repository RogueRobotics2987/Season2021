/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ThirdStageWheel.h"

ThirdStageWheel::ThirdStageWheel(ColorSensor* c_colorSensor) {
  mColorSensor = c_colorSensor;
  AddRequirements(mColorSensor); 

}

// Called when the command is initially scheduled.
void ThirdStageWheel::Initialize() {
  mColorSensor->ResetNums(); 

}

// Called repeatedly when this Command is scheduled to run
void ThirdStageWheel::Execute() {
  mColorSensor->SetMotor(.3); 

}

// Called once the command ends or is interrupted.
void ThirdStageWheel::End(bool interrupted) {
  mColorSensor->StopMotor(); 

}

// Returns true when the command should end.
bool ThirdStageWheel::IsFinished() { 
  if(mColorSensor->thirdLevelSpin() > 10){
    return true;
  }
  else{
    return false; 
  }


 }
