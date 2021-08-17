#include "subsystems/ShooterSubsystem.h"


ShooterSubsystem::ShooterSubsystem(){
ShooterSolenoid1 = new frc::DoubleSolenoid(0, 1); 
ShooterSolenoid2 = new frc::DoubleSolenoid(2, 3); 
ShooterSolenoid3 = new frc::DoubleSolenoid(4, 5); 
ShooterSolenoid4 = new frc::DoubleSolenoid(6, 7); 
ShooterSolenoid5 = new frc::DoubleSolenoid(7, 8); 
}

void ShooterSubsystem::Periodic() {}
void ShooterSubsystem::Close(int SolenoidNum){
    std::cout << ShooterSolenoid1->Get() << std::endl;
    std::cout << ShooterSolenoid2->Get() << std::endl;
    std::cout << ShooterSolenoid3->Get() << std::endl;
    std::cout << ShooterSolenoid4->Get() << std::endl;
    std::cout << ShooterSolenoid5->Get() << std::endl;

    if(SolenoidNum==1){
        ShooterSolenoid1->Set(frc::DoubleSolenoid::kForward);
        std::cout << "Solenoid 1 close" << std::endl;
    } else if(SolenoidNum==2){
        ShooterSolenoid2->Set(frc::DoubleSolenoid::kForward);
        std::cout << "Solenoid 2 close" << std::endl;
    } else if(SolenoidNum==3){
        ShooterSolenoid3->Set(frc::DoubleSolenoid::kForward);
        std::cout << "Solenoid 3 close" << std::endl;
    } else if(SolenoidNum==4){
        ShooterSolenoid4->Set(frc::DoubleSolenoid::kForward);
        std::cout << "Solenoid 4 close" << std::endl;
    } else if(SolenoidNum==5){
        ShooterSolenoid5->Set(frc::DoubleSolenoid::kForward);
        std::cout << "Solenoid 5 close" << std::endl;
    }
}

void ShooterSubsystem::Open(int SolenoidNum){ 

    if(SolenoidNum==1){
        ShooterSolenoid1->Set(frc::DoubleSolenoid::kReverse);
        std::cout << ShooterSolenoid1->Get() << std::endl;
        std::cout << "Solenoid 1 open" << std::endl;
    } else if(SolenoidNum==2){
        ShooterSolenoid2->Set(frc::DoubleSolenoid::kReverse);
        std::cout << "Solenoid 2 open" << std::endl;
    } else if(SolenoidNum==3){
        ShooterSolenoid3->Set(frc::DoubleSolenoid::kReverse);
        std::cout << "Solenoid 3 open" << std::endl;
    } else if(SolenoidNum==4){
        ShooterSolenoid4->Set(frc::DoubleSolenoid::kReverse);
        std::cout << "Solenoid 4 open" << std::endl;
    } else if(SolenoidNum==5){
        ShooterSolenoid5->Set(frc::DoubleSolenoid::kReverse);
        std::cout << "Solenoid 5 open" << std::endl;
    }
} 

