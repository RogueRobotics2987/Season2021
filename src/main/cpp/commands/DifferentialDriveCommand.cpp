// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DifferentialDriveCommand.h"


DifferentialDriveCommand::DifferentialDriveCommand(DifferentialDriveSubsystem* subsystem, frc::Joystick* stick)
    : m_subsystem{subsystem} {}
//DifferentialDriveCommand::DifferentialDriveCommand(ArmSubsystem* subsystem, frc::Joystick* xbox)
//  : m_armsubsystem{subsystem} {}
void DifferentialDriveCommand::Execute(){
    m_subsystem->Drive(stick->GetRawAxis(1), stick->GetRawAxis(2));
}
bool DifferentialDriveCommand::IsFinished(){
    return false;
}
void DifferentialDriveCommand::End(bool){
    m_subsystem->Drive(0, 0);
}