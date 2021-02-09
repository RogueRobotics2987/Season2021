// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/math>

#include "Constants.h"
#include <string>

// SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
//                            const int driveEncoderPorts[],
//                            const int turningEncoderPorts[],
//                            bool driveEncoderReversed,
//                            bool turningEncoderReversed)
//     : m_driveMotor(driveMotorChannel),
//       m_turningMotor(turningMotorChannel),
//       m_driveEncoder(driveEncoderPorts[0], driveEncoderPorts[1]),
//       m_turningEncoder(turningEncoderPorts[0], turningEncoderPorts[1]),
//       m_reverseDriveEncoder(driveEncoderReversed),
//       m_reverseTurningEncoder(turningEncoderReversed) {
//   // Set the distance per pulse for the drive encoder. We can simply use the
//   // distance traveled for one rotation of the wheel divided by the encoder
//   // resolution.
//   m_driveEncoder.SetDistancePerPulse(
//       ModuleConstants::kDriveEncoderDistancePerPulse);

//   // Set the distance (in this case, angle) per pulse for the turning encoder.
//   // This is the the angle through an entire rotation (2 * wpi::math::pi)
//   // divided by the encoder resolution.
//   m_turningEncoder.SetDistancePerPulse(
//       ModuleConstants::kTurningEncoderDistancePerPulse);

//   // Limit the PID Controller's input range between -pi and pi and set the input
//   // to be continuous.
//   m_turningPIDController.EnableContinuousInput(units::radian_t(-wpi::math::pi),
//                                                units::radian_t(wpi::math::pi));
// }

SwerveModule::SwerveModule(int m_MotorController, rev::CANEncoder::EncoderType m_EncoderType, int m_counts_per_rev, 
int m_MotorControllerTurning, rev::CANEncoder::EncoderType m_EncoderTypeTurning, int m_counts_per_revTurning,
 bool driveEncoderReversed,
 bool turningEncoderReversed)
    // : //m_driveMotor(driveMotorChannel),
    //   // m_turningMotor(turningMotorChannel),
    //   //m_driveEncoder(driveEncoderPorts[0], driveEncoderPorts[1]),
    //   samDriveMotor{m_MotorController},
    //   samTurningMotor{m_MotorControllerTurning},
    //   samDriveEncoder(samDriveMotor, m_EncoderType, m_counts_per_rev),
    //   // m_turningEncoder(turningEncoderPorts[0], turningEncoderPorts[1]),
    //   samTurningEncoder(samTurningMotor, m_EncoderTypeTurning, m_counts_per_revTurning),
    //   m_reverseDriveEncoder(driveEncoderReversed),
    //   m_reverseTurningEncoder(turningEncoderReversed)
       {
         samDriveMotor = new rev::CANSparkMax(m_MotorController, rev::CANSparkMax::MotorType::kBrushless);
         samTurningMotor = new rev::CANSparkMax(m_MotorControllerTurning, rev::CANSparkMax::MotorType::kBrushless);
         samDriveEncoder = new rev::CANEncoder(*samDriveMotor, m_EncoderType, m_counts_per_rev);
         samTurningEncoder = new rev::CANEncoder(*samTurningMotor, m_EncoderTypeTurning, m_counts_per_revTurning);
         m_reverseDriveEncoder = driveEncoderReversed;
         m_reverseTurningEncoder = turningEncoderReversed;
        
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  samDriveEncoder->SetPositionConversionFactor(
      ModuleConstants::kDriveEncoderDistancePerPulse);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * wpi::math::pi)
  // divided by the encoder resolution.
  samTurningEncoder->SetPositionConversionFactor(
      ModuleConstants::kTurningEncoderDistancePerPulse);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(units::radian_t(-wpi::math::pi),
                                               units::radian_t(wpi::math::pi));
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{samDriveEncoder->GetVelocity()},
          frc::Rotation2d(units::radian_t(samTurningEncoder->GetPosition()))};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState& state) {
  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      samDriveEncoder->GetVelocity(), state.speed.to<double>());

  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t(samTurningEncoder->GetPosition()), state.angle.Radians());
  frc::SmartDashboard::PutNumber(std::to_string(samDriveMotor->GetDeviceId()), driveOutput);
  frc::SmartDashboard::PutNumber(std::to_string(samTurningMotor->GetDeviceId()), turnOutput);


  // Set the motor outputs.
  samDriveMotor->Set(driveOutput);
  samTurningMotor->Set(turnOutput);
}

// void SwerveModule::ResetEncoders() {
//   samDriveEncoder.Reset();
//   samTurningEncoder.Reset();
// }
SwerveModule::~SwerveModule(){
  delete samDriveMotor;
  delete samTurningMotor;
  delete samDriveEncoder;
  delete samTurningEncoder;
}
