/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h> 
#include <frc/Solenoid.h> 
#include <frc/DoubleSolenoid.h> 
#include <frc/Timer.h> 
class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  void StartConveyor(double percent); 
  void IntakeBall(double setVal);
  void PrepareBall();
  void StopMotors();
  void setSolenoidTrue(); 
  void setSolenoidFalse(); 
  void startTimer(); 
  void ResetBallCount(); 
  double conveyorVal; 
  void resetOutBalls(); 

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax* p_intakeMotor;
  rev::CANSparkMax* p_conveyorMotor;
  frc::DigitalInput* p_intakeSensor;
  frc::DigitalInput* p_topSensor; 
  frc::Timer* myTimer; 
  frc::Timer* myTimer2;
  bool timeGotten = false; 
  double conveyorTime = 0; 
  int ballCount = 0;
  bool sensorBool = false; 
  bool secondSensorBool = false; 
  bool firstTimeGotten = false; 
  int ballOut = 0; 
  bool intakeTime = 0; 
  double firstTime = 0; 
  frc::DoubleSolenoid* intakeSolenoid; 
  std::string firmwareVersion = "1.8.2"; 
  frc::Timer* intakeTimer;
  bool intakeGood = true; 
  bool sensorSafety = false; 
};
