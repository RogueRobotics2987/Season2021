/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ctre/Phoenix.h"
#include <frc/Timer.h> 
#include <frc/Solenoid.h> 
#include <frc/DoubleSolenoid.h>

class Climber : public frc2::SubsystemBase {
 public:
  Climber();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  void TheClimb(double climbVal); //i can almost see it... that dream im dreamin cause
  void Balance(double rollieVal);
  double getMatchTime(); 
  bool getForwardLimitVal(); 
  bool getReverseLimitVal(); 
  void movePin(bool State); 

 private:
  WPI_TalonFX* p_climbyBoiFalco = nullptr; //57
  WPI_TalonSRX* p_rollieBoiTalo = nullptr; //
  frc::Timer* matchTimer = nullptr; 
  frc::DoubleSolenoid* climbingPiston = nullptr;

  bool pistonVal = false; 

  

 
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

};
