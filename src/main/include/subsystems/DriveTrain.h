/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "AHRS.h"
#include <frc/AnalogGyro.h>
#include <frc/AnalogInput.h>
#include <frc/Encoder.h>
#include <frc/PWMVictorSPX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/trajectory/Trajectory.h> 
#include <frc/trajectory/TrajectoryGenerator.h>
#include <units/units.h>
#include <frc/smartdashboard/Field2d.h>
//#include "rev/CANPIDController.h"





namespace frc {
class Joystick;
}  // namespace frc

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
  double GetHeading();
  void ResetOdometry(frc::Pose2d pose); 

  /**
   * Reset the robots sensors to the zero states.
   */
  void Reset();

  /**
   * @return The distance driven (average of left and right encoders).
   */
  double GetDistance();

  /**
   * @return The distance to the obstacle detected by the rangefinder.
   */
  double GetDistanceToObstacle();
  double GetTurnRate(); 

frc::Pose2d GetPose(); 
void ResetEncoders(); 
void TrajectoryInit(); 
frc::DifferentialDriveWheelSpeeds GetWheelSpeeds(); 
void TankDriveVolts(units::volt_t left, units::volt_t right); 

  


 private:

  rev::CANSparkMax* LeftBack = nullptr;
  rev::CANSparkMax* LeftFront = nullptr;
  rev::CANSparkMax* RightBack = nullptr;
  rev::CANSparkMax* RightFront = nullptr;
  rev::CANEncoder* leftEncoder = nullptr;
  rev::CANEncoder* rightEncoder = nullptr; 
  frc::DifferentialDrive* m_robotDrive = nullptr;
  AHRS* myAhrs = nullptr; 
  frc::DifferentialDriveOdometry* m_odometry = nullptr; 

  frc::Field2d m_field; 


  // frc::PWMVictorSPX m_frontLeft{1};
  // frc::PWMVictorSPX m_rearLeft{2};
  // frc::SpeedControllerGroup m_left{LeftFront, LeftBack};

  // frc::PWMVictorSPX m_frontRight{3};
  // frc::PWMVictorSPX m_rearRight{4};
  // frc::SpeedControllerGroup m_right{RightFront, RightBack};

  // frc::DifferentialDrive m_robotDrive{m_left, m_right};

  // frc::Encoder m_leftEncoder{1, 2};
  // frc::Encoder m_rightEncoder{3, 4};
  // frc::AnalogInput m_rangefinder{6};
  // frc::AnalogGyro m_gyro{1};
};
