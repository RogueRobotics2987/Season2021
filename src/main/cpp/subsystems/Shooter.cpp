/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Shooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

Shooter::Shooter() {
    //42 test motor on Ollie 
    //56 ws on Robot on a board
    //frc::SmartDashboard::PutNumber("Set RPM", 3800); 
    frc::SmartDashboard::PutNumber("Set P", kp); 
    frc::SmartDashboard::PutNumber("Set I", ki); 
    frc::SmartDashboard::PutNumber("Set D", kd); 
    frc::SmartDashboard::PutNumber("Set FF", kff);
    frc::SmartDashboard::PutNumber("Set Arb FF", 0); 

    shooterMotor = new rev::CANSparkMax(47, rev::CANSparkMax::MotorType::kBrushless);
    shooterPID = new rev::CANPIDController(*shooterMotor);
    shooterEncoder = new rev::CANEncoder(*shooterMotor);
    //shooterPID->SetP(0.0003*4);
    shooterPID->SetP(kp); 
    shooterPID->SetI(ki);
    shooterPID->SetD(kd);
    //shooterPID->SetP(.0016/2); //Ollie motor only
    shooterPID->SetOutputRange(-1, 1);
    shooterPID->SetSmartMotionMaxVelocity(4000);
    shooterPID->SetSmartMotionMinOutputVelocity(1500);
    shooterPID->SetSmartMotionMaxAccel(1000.0/1.0);
    shooterPID->SetSmartMotionAllowedClosedLoopError(0.0);
    shooterPID->SetIZone(800);
    shooterPID->SetFF(0.7/3500);

    myTimer = new frc::Timer();
    myTimer -> Reset();
    myTimer -> Start();

}



// This method will be called once per scheduler run
void Shooter::Periodic() {
    bool shooterWorks = true; 

    double startTime = myTimer->Get();

    if(activeShoot){
        //shooterPID->SetReference(TargetRPM, rev::ControlType::kVelocity, arbFF);
        
        if(getVelocity() < 500){
            setPercent(0.5);
        } else {
            setShooter();
        }
        
    } else {
        stopShooter();
    };

    if(shooterMotor->GetFirmwareString() != firmwareVersion){
        shooterWorks = false;
    }
    frc::SmartDashboard::PutBoolean("Shooter Works", shooterWorks); 
    arbFF = frc::SmartDashboard::GetNumber("ArbFF", 0); 
    kp = frc::SmartDashboard::GetNumber("Set P", kp); 
    ki = frc::SmartDashboard::GetNumber("Set I", ki);
    kd = frc::SmartDashboard::GetNumber("Set D", kd);
    kff = frc::SmartDashboard::GetNumber("Set FF", kff);

    TargetRPM = frc::SmartDashboard::GetNumber("Set RPM", 3800); 
    frc::SmartDashboard::PutNumber("Shooter speed", shooterEncoder->GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter App Out", shooterMotor->GetAppliedOutput());
    frc::SmartDashboard::PutNumber("Shooter Applied Current", shooterMotor->GetOutputCurrent());
    if(Lastkp != kp)   {shooterPID->SetP(kp);   Lastkp = kp;}
    if(Lastki != ki)   {shooterPID->SetI(ki);   Lastki = ki;}
    if(Lastkd != kd)   {shooterPID->SetD(kd);   Lastkd = kd;}
    if(Lastkff != kff) {shooterPID->SetFF(kff); Lastkff = kff;}
    // shooterPID->SetP(5E-4); 
    // shooterPID->SetFF(2.05E-4);
    // shooterPID->SetD(1E-4);

    if(0) {
        std::cout << "Shooter Periodic Time: " << myTimer->Get() - startTime << std::endl;
    }
    

}

double Shooter::getVelocity(){
    return shooterEncoder->GetVelocity(); 

}
void Shooter::setPercent(double percent){
    shooterMotor->Set(percent); 
}


void Shooter::startShooter() {
    shooterMotor->Set(0.2);
}

void Shooter::stopShooter() {
    shooterMotor->Set(0);
}

void Shooter::setShooter() {
   // shooterPID->SetReference(maxRPM, rev::ControlType::kVelocity);
    shooterPID->SetReference(TargetRPM, rev::ControlType::kVelocity, arbFF);
}

void Shooter::toggleShoot() {
   activeShoot = !activeShoot;

}