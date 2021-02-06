// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <frc/Spark.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <wpi/math>
#include "rev/CANSparkMax.h"
#include "rev/CANEncoder.h"

#include "Constants.h"

class SwerveModule {
  using radians_per_second_squared_t =
      units::compound_unit<units::radians,
                           units::inverse<units::squared<units::second>>>;

 public:
//   SwerveModule(int driveMotorChannel, int turningMotorChannel,
//                const int driveEncoderPorts[2], const int turningEncoderPorts[2],
//                bool driveEncoderReversed, bool turningEncoderReversed);
    SwerveModule(int m_MotorController, rev::CANEncoder::EncoderType m_EncoderType, int m_counts_per_rev, 
    int m_MotorControllerTurning, rev::CANEncoder::EncoderType m_EncoderTypeTurning, int m_counts_per_revTurning,
    bool driveEncoderReversed,
    bool turningEncoderReversed);
    ~SwerveModule();
  frc::SwerveModuleState GetState();

  void SetDesiredState(frc::SwerveModuleState& state);

//   void ResetEncoders();

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  static constexpr units::radians_per_second_t kModuleMaxAngularVelocity =
      units::radians_per_second_t(wpi::math::pi);  // radians per second
  static constexpr units::unit_t<radians_per_second_squared_t>
      kModuleMaxAngularAcceleration =
          units::unit_t<radians_per_second_squared_t>(
              wpi::math::pi * 2.0);  // radians per second squared

//   frc::Spark m_driveMotor;
//   frc::Spark m_turningMotor;
  rev::CANSparkMax* samDriveMotor;
  rev::CANSparkMax* samTurningMotor;

//   frc::Encoder m_driveEncoder;
//   frc::Encoder m_turningEncoder;
  rev::CANEncoder* samDriveEncoder;
  rev::CANEncoder* samTurningEncoder;

  bool m_reverseDriveEncoder;
  bool m_reverseTurningEncoder;

  frc2::PIDController m_drivePIDController{
      ModuleConstants::kPModuleDriveController, 0, 0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      ModuleConstants::kPModuleTurningController,
      0.0,
      0.0,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};
};
