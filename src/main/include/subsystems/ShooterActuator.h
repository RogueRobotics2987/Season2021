/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Joystick.h>
#include "rev/CANSparkMax.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <frc/Timer.h>


class ShooterActuator : public frc2::SubsystemBase {
 public:
  ShooterActuator();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  void setAngleH(double stickVal);
  void setAngleV(double stickVal);
  bool GetForwardLimitState();
  void SetAutoAim(bool AutoAimFlag);
  void safeSetH(double setVal);
  void safeSetV(double setVal);
  double GetTY(); 
  double GetTX(); 
  void switchCam(bool flag); 
  void limeStream(int num);


 private:
   rev::CANSparkMax* angleMotorH; 
   rev::CANSparkMax* angleMotorV; 
   std::shared_ptr<NetworkTable> limelightTable;
   int H_AimState = 0;
   int V_AimState = 0;
   float tx = 0;
   float ty = 0;
   float AimH_P = 0.32;
   float AimV_P = 0.32;
   float PositionH;
   float PositionV;
   frc::Timer* myTimer;

   bool AutoAimMode = false;
   double safeStick(double stickVal, double pos);
   std::string firmwareVersion = "1.8.2"; 



   // Components (e.g. motor controllers and sensors) should generally be
   // declared private and exposed only through public methods.
};
