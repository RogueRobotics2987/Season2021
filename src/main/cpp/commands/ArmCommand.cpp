// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArmCommand.h"


ArmCommand::ArmCommand(ArmSubsystem* subsystem, frc::Joystick* xbox)
 {
     m_subsystem = subsystem;
     SetName("ArmCommand");
     AddRequirements({m_subsystem});
     xbox = xbox;
 }
//ArmCommand::ArmCommand(ArmSubsystem* subsystem, frc::Joystick* xbox)
//  : m_armsubsystem{subsystem} {}
void ArmCommand::Execute(){
    m_subsystem->ArmControl(m_xbox->GetRawAxis(0), m_xbox->GetRawAxis(1));
}
bool ArmCommand::IsFinished(){
    return false;
}
void ArmCommand::End(bool){
    m_subsystem->ArmControl(0, 0);
}
