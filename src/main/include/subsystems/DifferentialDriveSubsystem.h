// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
#include "ctre/Phoenix.h"





namespace frc {
class Joystick;
}  // namespace frc

/**
 * The DriveTrain subsystem incorporates the sensors and actuators attached to
 * the robots chassis. These include four drive motors, a left and right encoder
 * and a gyro.
 */
class DifferentialDriveSubsystem : public frc2::SubsystemBase {
 public:
  DifferentialDriveSubsystem();

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

  WPI_TalonSRX* Motor;
  WPI_TalonSRX* Motor2;
  WPI_TalonSRX* Motor3;
  WPI_TalonSRX* Motor4;
  frc::DifferentialDrive* DriverFront;
  frc::DifferentialDrive* DriverBack;
  frc::DifferentialDrive* m_robotDrive;
  AHRS* myAhrs; 
  frc::DifferentialDriveOdometry* m_odometry;


  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
