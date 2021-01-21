/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ColorSensor.h"
#include "rev/CANPIDController.h"

ColorSensor::ColorSensor() {
  myTimer = new frc::Timer; 
  myTimer2 = new frc::Timer; 
  myTimer2 -> Reset();
  myTimer2 -> Start();
  // Asigned the colorSensor pointer to a new i2cPort
  colorSensor = new rev::ColorSensorV3(i2cPort);
  curColor = colorSensor->GetColor();

  spinner = new rev::CANSparkMax(61, rev::CANSparkMax::MotorType::kBrushless); //a random motor was assigned, don't know what motor will be used on the actual robot
  spinner->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake); 
  m_colorMatch = new rev::ColorMatch; 
  m_colorMatch->AddColorMatch(kBlueTarget);
  m_colorMatch->AddColorMatch(kGreenTarget);
  m_colorMatch->AddColorMatch(kRedTarget);
  m_colorMatch->AddColorMatch(kYellowTarget);
}
std::string ColorSensor::getColor2(){ 
  frc::Color detectedColor = colorSensor->GetColor(); 
  double confidence = 0; 
  frc::Color matchedColor = m_colorMatch->MatchClosestColor(detectedColor, confidence); 

  if(matchedColor == kBlueTarget){
    return "R";
  }if(matchedColor == kRedTarget){
    return "B"; 
  }if(matchedColor == kYellowTarget){
    return "G"; 
  }if(matchedColor == kGreenTarget){
    return "Y"; 
  }
  else{
    return ""; 
  }
}
void ColorSensor::resetTimer(){
  myTimer->Reset(); 
  myTimer->Start(); 
}
double ColorSensor::GetTime(){
  return myTimer->Get(); 
}
std::string ColorSensor::GetColor(){
  curColor = colorSensor->GetColor(); 
  if ((curColor.blue>=0.35) && 
       (curColor.green>curColor.blue) &&
       (curColor.blue>curColor.red))
        return "R"; //color sensor reads blue, shield generator reads red

    else if ((curColor.green>curColor.blue) &&
            (curColor.blue>curColor.red) &&
            (abs(curColor.red-curColor.blue)<=.07))
            return "Y"; //color sensor reads green, shield generator reads yellow

    else if  ((curColor.green>curColor.red) &&
             (curColor.red>curColor.blue)
             )
             return "G"; //color sensor reads yellow, shield generator reads green

    else if  (((curColor.red>curColor.blue) &&
             (curColor.blue>curColor.green))
             || (abs(curColor.red-curColor.green)<=.1)
             )
             return "B";//color sensor reads red, shield generator reads blue

    else{
    return "";
    }
}

std::string ColorSensor::GetGameData(){
   gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
   return gameData; 
  
 
}

void ColorSensor::StopMotor(){
  spinner->Set(0); 
}

void ColorSensor::setDirection(){
  std::string givenColor = GetGameData(); 
  std::string ogColor = GetColor(); 
  int givenColorValue = 0; 
  int ogColorValue = 0; 
  if(givenColor == "R"){
    givenColorValue = 1; 
  }
  else if(givenColor == "G"){
    givenColorValue = 2; 
  }
  else if(givenColor == "B")
  {
    givenColorValue = 3; 
  }
  else if(givenColor == "Y"){
    givenColorValue = 4; 
  }
  
  if(ogColor == "R"){
    ogColorValue = 1; 
  }
  else if(ogColor == "G"){
    ogColorValue = 2; 
  }
  else if(ogColor == "B"){
    ogColorValue = 3; 
  }
  else if(ogColor == "Y"){
    ogColorValue = 4; 
  }

  if(ogColorValue - givenColorValue <= 0){
    moveRight = false; 
  }
  else if(ogColorValue - givenColorValue <= 2){
    moveRight = true; 
  }
  else{
    moveRight = false; 
  }

  if(givenColor == "Y" && ogColor == "R"){
    moveRight = true; 
  }
  else if(givenColor == "R" && ogColor == "Y"){
    moveRight = false; 
  }
  

}

int ColorSensor::thirdLevelSpin(){
  if(ogColor == GetColor()){
    thirdStageCount++; 
  }
  return thirdStageCount; 

}



void ColorSensor::SetMotor(double motorVal){
  spinner->Set(motorVal);
}
double ColorSensor::newNumSpins(){
    if(ogColor == ""){
      return 30; 
    }

      if(newColor == "R"){
        if(GetColor() == "G"){
          if(ogColor == "G"){
            colorCount++; 
          }
          newColor = "G";
        }
        
      }

      else if(newColor == "G"){
        if(GetColor() == "B" ){
          if(ogColor == "B"){
            colorCount++; 
          }
          newColor = "B"; 
        }

      }
      else if(newColor == "B"){
        if(GetColor() == "Y"){
          if(ogColor == "Y"){
            colorCount++; 
          }
          newColor = "Y"; 
        }

      }
      else if(newColor == "Y"){
        if(GetColor() == "R"){
          if(ogColor == "R"){
            colorCount++; 
          }
          newColor = "R"; 
        }
      }
      return colorCount; 
}
double ColorSensor::SpinNum(){
  if(colorChange && ogColor == getColor2()){
    colorCount++;
    colorChange = false; 
  }

  else if(ogColor != getColor2()){
    colorChange = true; 
  }

  //To get number of rotations of table use formula
  // (colorCount - 1)/2
  frc::SmartDashboard::PutNumber("Color Count", colorCount); 
  return colorCount; 

}

void ColorSensor::ResetNums(){
  ogColor = GetColor();
  colorCount = 0; 
  newColor = ogColor; 


}

void ColorSensor::Periodic(){
  double startTime = myTimer2->Get();

  bool colorSensorWorks = true; 
  if(firmwareVersion != spinner->GetFirmwareString()){
    colorSensorWorks = false; 
  }
  frc::SmartDashboard::PutBoolean("Color Sensor Works", colorSensorWorks); 
  curColor = colorSensor->GetColor(); 
  frc::SmartDashboard::PutString("Color2:", getColor2()); 
  frc::SmartDashboard::PutNumber("Green Value", curColor.green); 
  frc::SmartDashboard::PutNumber("Red Value", curColor.red); 
  frc::SmartDashboard::PutNumber("Blue Value", curColor.blue); 
  frc::SmartDashboard::PutString("Color:", GetColor()); 
  frc::SmartDashboard::PutNumber("Color Count", newNumSpins()); 
  frc::SmartDashboard::PutString("Original Color", ogColor); 
  frc::SmartDashboard::PutNumber("Color Motor Current Output", spinner->GetAppliedOutput()); 

    if(0) {
        std::cout << "ColorSensor Periodic Time: " << myTimer2->Get() - startTime << std::endl;
    }

}

void ColorSensor::GameDataSpin(){

}

void ColorSensor::PrintColor(){
  frc::SmartDashboard::PutNumber("Red Value", curColor.red); 
  frc::SmartDashboard::PutNumber("Green Value", curColor.green); 
  frc::SmartDashboard::PutNumber("Blue Value", curColor.blue); 

  frc::SmartDashboard::PutString("Shield Generator Color", GetColor());
  //frc::SmartDashboard::PutBoolean("Game Data Match", GetGameData());
  frc::SmartDashboard::PutString("Game Data", gameData);
  
}