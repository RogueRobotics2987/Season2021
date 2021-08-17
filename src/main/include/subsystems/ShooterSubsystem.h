// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


#include <frc2/command/SubsystemBase.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <iostream>

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  

  void Open(int SolenoidNum);
  void Close(int SolenoidNum);
  frc::DoubleSolenoid* ShooterSolenoid1 = nullptr;
  frc::DoubleSolenoid* ShooterSolenoid2 = nullptr;
  frc::DoubleSolenoid* ShooterSolenoid3 = nullptr;
  frc::DoubleSolenoid* ShooterSolenoid4 = nullptr;
  frc::DoubleSolenoid* ShooterSolenoid5 = nullptr;
};

