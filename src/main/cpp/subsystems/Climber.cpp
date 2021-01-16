/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Climber.h"
#include <frc/smartdashboard/SmartDashboard.h> 

Climber::Climber() {
    p_climbyBoiFalco = new WPI_TalonFX(57);
    p_rollieBoiTalo = new WPI_TalonSRX(30);
    matchTimer = new frc::Timer; 
    climbingPiston = new frc::DoubleSolenoid(6,7);//DoubleSolenoid(7); 
   // p_climbyBoiFalco->SetSelectedSensorPosition(0); 
   p_climbyBoiFalco->OverrideLimitSwitchesEnable(true);
   p_climbyBoiFalco->ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen); 
   p_climbyBoiFalco->ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyClosed); 
   


}

// This method will be called once per scheduler run
void Climber::Periodic() {
    frc::SmartDashboard::PutBoolean("Falcon Forward Limit", p_climbyBoiFalco->IsFwdLimitSwitchClosed()); 
    frc::SmartDashboard::PutBoolean("Falcon Reverse", p_climbyBoiFalco->IsRevLimitSwitchClosed()); 
    frc::SmartDashboard::PutNumber("Falcon Encoder Value", p_climbyBoiFalco->GetSelectedSensorPosition());
    //frc::SmartDashboard::PutNumber("Climber Encoder Value", p_climbyBoiFalco->Get);
    frc::SmartDashboard::PutNumber("Match Time", getMatchTime()); 
    
}

bool Climber::getForwardLimitVal(){
    return p_climbyBoiFalco->IsFwdLimitSwitchClosed(); 
}

bool Climber::getReverseLimitVal(){
    return p_climbyBoiFalco->IsRevLimitSwitchClosed(); 

}

void Climber::TheClimb(double climbVal) {

    
    if(fabs(climbVal) < .08){
    climbVal = 0; 
    }
    // if(!getForwardLimitVal() && climbVal > 0){
    //     climbVal = 0; 
    // }
    // if(getReverseLimitVal() && climbVal < 0){
    //     climbVal = 0; 
    // }
    p_climbyBoiFalco->Set(ControlMode::PercentOutput ,climbVal);
}

void Climber::Balance(double rollieVal) {
    if(fabs(rollieVal) < .08){ 
    rollieVal = 0; 
    } 
    p_rollieBoiTalo->Set(ControlMode::PercentOutput ,rollieVal);
}

double Climber::getMatchTime(){
    return matchTimer->GetMatchTime(); 
}

void Climber::movePin(bool State){ 
    if(State) {
        climbingPiston->Set(frc::DoubleSolenoid::Value::kForward);
    } else {
        climbingPiston->Set(frc::DoubleSolenoid::Value::kReverse);
    }
}