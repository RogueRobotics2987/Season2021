#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <units/units.h>
#include <wpi/math>

#pragma once 

namespace DriveConstants{ 

    constexpr auto ks = 0.131_V; 
    constexpr auto kv = 2.73 * 1_V * 1_s / 1_m; 
    constexpr auto ka = .516 * 1_V * 1_s * 1_s / 1_m; 
    // constexpr double kPDriveVel = 1.69; 
    constexpr double kPDriveVel = 0; 
    constexpr auto trackWidth = 0.784_m; 
    extern const frc::DifferentialDriveKinematics kDriveKinematics;
    constexpr int kEncoderCPR = 1024; 


}

namespace AutoConstants{ 
     constexpr auto kMaxSpeed = 1_mps; 
     constexpr auto kMaxAcceleration = 1_mps_sq;
     constexpr double  kRamseteB = 2; 
     constexpr double kRamseteZeta = .7;


}

