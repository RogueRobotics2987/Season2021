#include "subsystems/ShooterSubsystem.h"


ShooterSubsystem::ShooterSubsystem(){
ShooterSolenoid1 = new frc::Solenoid(0,0); 
ShooterSolenoid2 = new frc::Solenoid(0,1); 
ShooterSolenoid3 = new frc::Solenoid(0,2); 
ShooterSolenoid4 = new frc::Solenoid(0,3); 
ShooterSolenoid5 = new frc::Solenoid(0,4); 
}



void ShooterSubsystem::Periodic() {}

void ShooterSubsystem::Open(int SolenoidNum){
    if(SolenoidNum==1){
        ShooterSolenoid1->Set(true);
    } else if((SolenoidNum==2){
        ShooterSolenoid1->Set(true);
    } else if((SolenoidNum==3){
        ShooterSolenoid1->Set(true);
    } else if((SolenoidNum==4){
        ShooterSolenoid1->Set(true);
    } else if((SolenoidNum==5){
        ShooterSolenoid1->Set(true);
    }
} 

void ShooterSubsystem::Close(int SolenoidNum){
    if(SolenoidNum==1){
        ShooterSolenoid1->Set(false);
    } else if((SolenoidNum==2){
        ShooterSolenoid1->Set(false);
    } else if((SolenoidNum==3){
        ShooterSolenoid1->Set(false);
    } else if((SolenoidNum==4){
        ShooterSolenoid1->Set(false);
    } else if((SolenoidNum==5){
        ShooterSolenoid1->Set(false);
    }
}
