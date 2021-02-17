/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "AHRS.h"
#include "rev/CANSparkMax.h"

#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>

//#include <units/units.h>
#include <units/angle.h>

#include "Constants.h" 

/**
 * The DriveTrain subsystem incorporates the sensors and actuators attached to
 * the robots chassis. These include four drive motors, a left and right encoder
 * and a gyro.
 */
class DriveTrain : public frc2::SubsystemBase {
  public:
    DriveTrain();

    /**
     * The log method puts interesting information to the SmartDashboard.
     */
    void Log();
    void Periodic(); 
    void autonDrive(); 
    /**
     * Tank style driving for the DriveTrain.
     * @param left Speed in range [-1,1]
     * @param right Speed in range [-1,1]
     */
    void Drive(double y, double z);

    /**
     * @return The robots heading in degrees.
     */
    units::degree_t GetHeading();
    void ResetOdometry(frc::Pose2d pose); 

    /**
     * Reset the robots sensors to the zero states.
     */
    void Reset();

    frc::Pose2d GetPose(); 
    void ResetEncoders(); 
    void TrajectoryInit(); 
    frc::DifferentialDriveWheelSpeeds GetWheelSpeeds(); 
    void TankDriveVolts(units::volt_t left, units::volt_t right); 


  private:
    rev::CANSparkMax* LeftBack = new rev::CANSparkMax(56, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANSparkMax* LeftFront = new rev::CANSparkMax(49, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANEncoder LeftEncoder = LeftFront->GetEncoder(); 
    rev::CANSparkMax* RightBack = new rev::CANSparkMax(50, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANSparkMax* RightFront = new rev::CANSparkMax(46, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANEncoder RightEncoder = RightFront->GetEncoder();

    frc::DifferentialDrive* m_robotDrive = nullptr;
    AHRS* myAhrs = nullptr; 
    frc::DifferentialDriveOdometry* m_odometry = nullptr; 

    frc::Field2d m_field;
};
