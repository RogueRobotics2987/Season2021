/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ShooterActuator.h"

ShooterActuator::ShooterActuator() {
    //55 is the number of the actual motor controller, 49 is the motor controller used for testing
    angleMotorH = new rev::CANSparkMax(54, rev::CANSparkMax::MotorType::kBrushless); 
    angleMotorH->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    angleMotorV = new rev::CANSparkMax(59, rev::CANSparkMax::MotorType::kBrushless); 
    angleMotorV->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    //angleMotor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse,-100);
    frc::SmartDashboard::PutNumber("Shooter Aim - Horz. - P", AimH_P);
    myTimer = new frc::Timer();
    myTimer->Reset();
    myTimer->Start();

}

double ShooterActuator::GetTX(){
    return tx; 
}
double ShooterActuator::GetTY(){
    return ty; 
}
void ShooterActuator::switchCam(bool flag){
    if(flag){
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-rr")->PutNumber("stream", 1); 
    }
    else{
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-rr")->PutNumber("stream", 2); 
    }
}

void ShooterActuator:: limeStream(int num){
    limelightTable -> PutNumber("pipeline", num);
}

// This method will be called once per scheduler run
void ShooterActuator::Periodic() {
    double startTime = myTimer->Get();
    

    bool shooterActuatorWorking = true; 
    if(angleMotorH->GetFirmwareString() != firmwareVersion || angleMotorV->GetFirmwareString() != firmwareVersion){
         shooterActuatorWorking = false; 
    }
    
    frc::SmartDashboard::PutBoolean("Shooter Actuator Working", shooterActuatorWorking); 
    limelightTable = NetworkTable::GetTable("limelight-rr");

    limelightTable -> PutNumber("pipeline", limeStream);

    //must be moved to command 
    tx = limelightTable->GetNumber("tx", 0.0); 
    ty = limelightTable->GetNumber("ty", 0.0); 


    // Control Loop Vals
    AimH_P = frc::SmartDashboard::GetNumber("Shooter Aim - Horz. - P", 0.0);

    //stays in periodic
    frc::SmartDashboard::PutNumber("Limelight X", tx);
    frc::SmartDashboard::PutNumber("Limelight Y", ty);

    //Mode 1: Startup
    //  Set Motor speed to inwards
    //  Until LimitSwitch triggers
    bool LimitSwitchStateF = angleMotorH->GetForwardLimitSwitch(
        rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get();
    frc::SmartDashboard::PutBoolean("F Limit Switch State", LimitSwitchStateF);
    
    PositionH = angleMotorH->GetEncoder().GetPosition();
    frc::SmartDashboard::PutNumber("Shooter Position - Horz.", PositionH);
    PositionV = angleMotorV->GetEncoder().GetPosition();
    frc::SmartDashboard::PutNumber("Shooter Position - Vert.", PositionV);

    bool SoftLimitSwitchR = angleMotorH->IsSoftLimitEnabled(rev::CANSparkMax::SoftLimitDirection::kReverse);
    bool SoftLimitSwitchF = angleMotorH->IsSoftLimitEnabled(rev::CANSparkMax::SoftLimitDirection::kForward);
    frc::SmartDashboard::PutBoolean("softF Limit Switch State", SoftLimitSwitchF);
    frc::SmartDashboard::PutBoolean("softR Limit Switch State", SoftLimitSwitchR);

    if(0) {
        std::cout << "ShooterActuator Periodic Time: " << myTimer->Get() - startTime << std::endl;
    }


}

bool ShooterActuator::GetForwardLimitState() {
    bool LimitSwitchStateF = angleMotorH->GetForwardLimitSwitch(
    rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get();
    return LimitSwitchStateF;
}

void ShooterActuator::SetAutoAim(bool AutoAimFlag) {
    AutoAimMode = AutoAimFlag;
}

void ShooterActuator::setAngleH(double stickVal){
    frc::SmartDashboard::PutNumber("Aim State - Horz.", H_AimState);
    bool LimitSwitchStateF = angleMotorH->GetForwardLimitSwitch(
    rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get();
    //float PositionH = angleMotorH->GetEncoder().GetPosition();

    if(H_AimState == 0) {  //Startup State
        safeSetH(1.0);
        if(LimitSwitchStateF) {
            H_AimState = 9;
            angleMotorH->GetEncoder().SetPosition(0);
        }
    } else if (H_AimState == 3) { // Auto Aim Mode
        //ledMode 
        // 3 on 
        // 0 off 
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-rr")->PutNumber("ledMode", 3); 

        float H_ControlVal = AimH_P*tx;
        frc::SmartDashboard::PutNumber("Temp Aiming Horz. Motor Output", H_ControlVal);
        safeSetH(H_ControlVal); // For now do nothing...
        //safeSetV(0.0); // For now do nothing...

        if(!AutoAimMode) {
            H_AimState = 9;
        }

        
    } else if (H_AimState == 9) { 
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-rr")->PutNumber("ledMode", 0); 

        // Manual Aim Mode
        // stickVal = safeStick(stickVal, PositionH);
        // angleMotorH->Set(stickVal);
        safeSetH(stickVal); // For now do nothing...


        if(AutoAimMode) {
            H_AimState = 3;
        }

    } else {  // We should NEVER get here!
      std::cout << "WARNING SHOOT IN BAD STATE: " << H_AimState << std::endl;
    }

}

void ShooterActuator::setAngleV(double stickVal){
    bool LimitSwitchStateF = angleMotorV->GetForwardLimitSwitch(
    rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get();
    //float PositionV = angleMotorV->GetEncoder().GetPosition();

    if(V_AimState == 0) {  //Startup State
        safeSetV(1.0);
        if(LimitSwitchStateF) {
            V_AimState = 9;
            angleMotorV->GetEncoder().SetPosition(0);
        }
        
    } else if (V_AimState == 3) { // Auto Aim State
        float V_ControlVal = AimV_P*ty;
        frc::SmartDashboard::PutNumber("Temp Aiming Vert. Motor Output", V_ControlVal);
        safeSetV(V_ControlVal); // For now do nothing...

        if(!AutoAimMode) {
            V_AimState = 9;
        }


    } else if (V_AimState == 9) { // Operating State
        // if(fabs(stickVal) < .08){
        //     stickVal = 0; 
        // }
        // if(stickVal>0.4) { // Limit to 0.4 max power
        //     stickVal = 0.4;
        // } else if(stickVal<-0.4) {
        //     stickVal = -0.4;
        // }
        // angleMotorV->Set(-stickVal);
        safeSetV(-stickVal);

        if(AutoAimMode) {
            V_AimState = 3;
        }

    } else {  // We should NEVER get here!
      std::cout << "WARNING SHOOT IN BAD STATE: " << V_AimState << std::endl;
    }

}

// void ShooterActuator::turnOnVision(){ 

// }
double ShooterActuator::safeStick(double stickVal, double pos) {
    double curMax = 0;
    double curMin =0;
    if(fabs(stickVal) < .08) { stickVal = 0.0;}
    // if(stickVal>0.4) { // Limit to 0.4 max power
    //     stickVal = 0.4;
    // } else if(stickVal<-0.4) {
    //     stickVal = -0.4;
    // } 

    if(pos < -100){
        curMax = 0.9;
    } else if (-100 < pos && pos <= 0) {
        curMax = (1.0 - 0.4)/(-100.0 -0.0)* pos + 0.4; 
    } else {
        curMax = 0.4;
    }

        
    if(-400 < pos){
        curMin = -0.9;
    } else if (-500 <= pos && pos <= -400) {
        curMin = (-0.4 - - 1.0)/(-500 - -400)* pos + -3.4; 
    } else {
        curMin = -0.4;
    }

    stickVal = std::min(curMax,stickVal);
    stickVal = std::max(curMin,stickVal);

    return stickVal; 

}
    void ShooterActuator::safeSetH(double setVal){
        //double aimResetSpeedH = 1.0;

        // stickVal = safeStick(stickVal, PositionH);
        // angleMotorH->Set(stickVal);

        setVal = safeStick(setVal, PositionH);
        angleMotorH->Set(setVal);

    }

    void ShooterActuator::safeSetV(double setVal){
        //double aimResetSpeedV = 1.0;
        setVal = safeStick(setVal, PositionV);
        angleMotorV->Set(setVal);

    }
