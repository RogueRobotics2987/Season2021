<<<<<<< HEAD
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

//#include <frc/commands/Subsystem.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/util/Color.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "rev/CANSparkMax.h" 
#include <frc/Timer.h> 


class ColorSensor : public frc2::SubsystemBase {

  
 public:
  ColorSensor();
  // Made i2cPort and declared it to port type kOnboard
  std::string GetColor();
  void Periodic() override; 
  void PrintColor();
  std::string GetGameData();
  void SetMotor(double motorVal); 
  double SpinNum(); 
  void ResetNums(); 
  void GameDataSpin(); 
  void StopMotor(); 
  double newNumSpins(); 
  double GetTime(); 
  void setDirection(); 
  int thirdLevelSpin(); 
  bool moveRight = true; 
  int thirdStageCount = 0; 

  std::string newColor = ""; 
  int numOfColor = 0; 
  int colorThreshold = 10;
  void resetTimer(); 
  double timeSinceLast = 0; 
  std::string lastColor = ""; 
  bool isGood = false; 
  std::string targetColor = "T"; 
  bool Run = false; 
  
  rev::CANSparkMax* spinner; //color wheel spinner
  std::string getColor2(); 
  private:
  frc::Timer* myTimer; 
  frc::Timer* myTimer2; 
  std::string ogColor = ""; 
  bool colorChange = false; 
  int colorCount = 0; 
  // Made a pointer called ColorSensor
    rev::ColorSensorV3* colorSensor;
    static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
    frc::Color curColor;
    std::string gameData;
    rev::ColorMatch* m_colorMatch; 
    static constexpr frc::Color kBlueTarget = frc::Color(0.175, 0.47, 0.356); 
    static constexpr frc::Color kGreenTarget = frc::Color(0.21, 0.55, 0.235); 
    static constexpr frc::Color kRedTarget = frc::Color(0.4, 0.42, 0.17); 
    static constexpr frc::Color kYellowTarget = frc::Color(0.305, 0.545, 0.15); 
    std::string firmwareVersion = "1.8.2"; 
    
=======
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

//#include <frc/commands/Subsystem.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/util/Color.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "rev/CANSparkMax.h" 
#include <frc/Timer.h> 


class ColorSensor : public frc2::SubsystemBase {

  
 public:
  ColorSensor();
  // Made i2cPort and declared it to port type kOnboard
  std::string GetColor();
  void Periodic() override; 
  void PrintColor();
  std::string GetGameData();
  void SetMotor(double motorVal); 
  double SpinNum(); 
  void ResetNums(); 
  void GameDataSpin(); 
  void StopMotor(); 
  double newNumSpins(); 
  double GetTime(); 
  void setDirection(); 
  int thirdLevelSpin(); 
  bool moveRight = true; 
  int thirdStageCount = 0; 

  std::string newColor = ""; 
  int numOfColor = 0; 
  int colorThreshold = 10;
  void resetTimer(); 
  double timeSinceLast = 0; 
  std::string lastColor = ""; 
  bool isGood = false; 
  std::string targetColor = "T"; 
  bool Run = false; 
  
  rev::CANSparkMax* spinner; //color wheel spinner
  std::string getColor2(); 
  private:
  frc::Timer* myTimer; 
  frc::Timer* myTimer2; 
  std::string ogColor = ""; 
  bool colorChange = false; 
  int colorCount = 0; 
  // Made a pointer called ColorSensor
    rev::ColorSensorV3* colorSensor;
    static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
    frc::Color curColor;
    std::string gameData;
    rev::ColorMatch* m_colorMatch; 
    static constexpr frc::Color kBlueTarget = frc::Color(0.175, 0.47, 0.356); 
    static constexpr frc::Color kGreenTarget = frc::Color(0.21, 0.55, 0.235); 
    static constexpr frc::Color kRedTarget = frc::Color(0.4, 0.42, 0.17); 
    static constexpr frc::Color kYellowTarget = frc::Color(0.305, 0.545, 0.15); 
    std::string firmwareVersion = "1.8.2"; 
    
>>>>>>> 119e8b76d8cbbb98afe2ba5b6ee254a4c9262844
 };