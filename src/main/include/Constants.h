#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <units/units.h>
#include <wpi/math>

#pragma once 

namespace DriveConstants{ 


//BRANDONS OLD CONSTANTS
    // constexpr auto ks = 0.131_V; 
    // constexpr auto kv = 2.73 * 1_V * 1_s / 1_m; 
    // constexpr auto ka = .516 * 1_V * 1_s * 1_s / 1_m; 
    // constexpr double kPDriveVel = 16.9*4; 
    // constexpr auto trackWidth = 0.784_m; 
    // extern const frc::DifferentialDriveKinematics kDriveKinematics;
    // constexpr int kEncoderCPR = 1024; 


//OUR NEW CONSTANTS (JAN 2021)
    constexpr auto ks = 0.175_V; 
    constexpr auto kv = 2.76 * 1_V * 1_s / 1_m; 
    constexpr auto ka = .44 * 1_V * 1_s * 1_s / 1_m; 
    constexpr double kPDriveVel = 2.15 / 100; 
    constexpr auto trackWidth = 13.3_m; 
    extern const frc::DifferentialDriveKinematics kDriveKinematics;
    constexpr int kEncoderCPR = 1024; 



}

namespace AutoConstants{ 
     constexpr auto kMaxSpeed = 1_mps; 
     constexpr auto kMaxAcceleration = 1_mps_sq * 0.5;
     constexpr double  kRamseteB = 2; 
     constexpr double kRamseteZeta = .7;


}

