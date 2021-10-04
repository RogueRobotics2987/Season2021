// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>

class ActuatorSubsystem : public frc2::SubsystemBase {
 public:
  ActuatorSubsystem();
  bool ForwardLimitSwitch = false;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  

  void Extend();
  void Retract();
  void Neutral();
  bool GetForwardLimitSwitch();
  rev::CANSparkMax* ActuatorMotor = nullptr;
  int m_MotorController = 40;
  rev::CANEncoder GetEncoder(rev::CANEncoder::EncoderType sensorType, int counts_per_rev);

};