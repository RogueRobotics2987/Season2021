/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include <frc/Joystick.h>
#include "rev/CANPIDController.h"
#include <frc/Timer.h>

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();
  

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  void startShooter();
  void stopShooter();
  void setShooter();
  void setPercent(double percent); 
  double getVelocity(); 


 private:
  rev::CANSparkMax* shooterMotor = nullptr;
  rev::CANPIDController* shooterPID = nullptr;
  rev::CANEncoder* shooterEncoder = nullptr;
  double TargetRPM = 4000;
  double kp = 5E-4; 
  double ki = 0; 
  double kd = 0;
  double kff = 2.05E-4; //old number 0.7/3500
  const std::string firmwareVersion = "1.8.2"; 
  frc::Timer* myTimer = nullptr;
  double Lastkp=0, Lastki=0, Lastkd=0, Lastkff=0;
  double arbFF = 0; 

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
