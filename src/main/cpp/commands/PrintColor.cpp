<<<<<<< HEAD
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/PrintColor.h"

PrintColor::PrintColor(ColorSensor* colorsensor) {
  myColorSensor = colorsensor;
  AddRequirements(myColorSensor);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void PrintColor::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PrintColor::Execute() {
  myColorSensor->PrintColor(); 

}

// Called once the command ends or is interrupted.
void PrintColor::End(bool interrupted) {}

// Returns true when the command should end.
=======
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/PrintColor.h"

PrintColor::PrintColor(ColorSensor* colorsensor) {
  myColorSensor = colorsensor;
  AddRequirements(myColorSensor);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void PrintColor::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PrintColor::Execute() {
  myColorSensor->PrintColor(); 

}

// Called once the command ends or is interrupted.
void PrintColor::End(bool interrupted) {}

// Returns true when the command should end.
>>>>>>> 119e8b76d8cbbb98afe2ba5b6ee254a4c9262844
bool PrintColor::IsFinished() { return false; }