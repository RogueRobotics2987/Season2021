#pragma once 
#include "subsystems/Elevator.h" 
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>


class setHeight : public frc2::CommandHelper<frc2::CommandBase, setHeight>{ 
    public: 
        setHeight(double setPoint, Elevator* mElevator); 
        void Initialize(); 
        void Execute(); 
        bool isFinished(); 
        void End(); 
        void initialize(); 



    private: 
        Elevator* elevator; 
        double heightVal; 


};

