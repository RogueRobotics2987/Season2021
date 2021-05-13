// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include "commands/ResetHeading.h"

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
      std::cout << "cout in robot container" << std::endl;

  // Configure the button bindings
  ConfigureButtonBindings();
  m_drive.ZeroHeading();

  // Set up default drive command
  m_Actuator.SetDefaultCommand(frc2::RunCommand(
  [this]{
    m_driverController.GetPOV();
  if (m_driverController.GetPOV()==0){
    m_Actuator.Extend();
  } else if (m_driverController.GetPOV()==180){
    m_Actuator.Retract();
  } else if (m_driverController.GetPOV()==-1){
    m_Actuator.Neutral();
  }
  },
  {&m_Actuator}));
  m_Compressor.SetDefaultCommand(beginCompressor(&m_Compressor));
  m_Shooter.SetDefaultCommand(ShooterSafe(&m_Shooter));
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        //   std::cout << "sea out in robot container" << std::endl;
          frc::SmartDashboard::PutNumber("Left Hand Y", m_driverController.GetX(frc::GenericHID::kLeftHand));
          frc::SmartDashboard::PutNumber("Right Hand Y", m_driverController.GetY(frc::GenericHID::kRightHand));
          frc::SmartDashboard::PutNumber("Left Hand X", m_driverController.GetZ());
        
        
        
        double safeX = m_driverController.GetX(frc::GenericHID::kLeftHand);
        if(fabs(safeX)<.225) {
            safeX=0;}
        double safeY =  m_driverController.GetY(frc::GenericHID::kRightHand);
        if(fabs(safeY)<.225) { 
            safeY=0;}
        double safeRot = m_driverController.GetZ();
        if(fabs(safeRot)<.24) {
            safeRot=0;}
        
        // std::cout << "Sam Debug" << safeX << "," << safeY << "," << safeRot << std::endl;
        
        m_drive.Drive(units::meters_per_second_t(
                         -safeY),
                      units::meters_per_second_t(
                         -safeX),
                      units::radians_per_second_t(
                         -safeRot),
                      true);
        // m_drive.Drive(units::meters_per_second_t(0),
        // units::meters_per_second_t(1),
        // units::radians_per_second_t(0),
        // false);
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
    frc2::JoystickButton(&m_driverController, 2).WhenPressed(ResetHeading(&m_drive));
    frc2::JoystickButton(&m_driverController, 1).WhenPressed(Shooter(&m_Shooter));
//       [this] {
//         m_drive.ZeroHeading();
    //   },
//       {&m_drive}));
      
}

// frc2::Command* RobotContainer::GetAutonomousCommand() {
//   // Set up config for trajectory
//   frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
//                                AutoConstants::kMaxAcceleration);
//   // Add kinematics to ensure max speed is actually obeyed
//   config.SetKinematics(m_drive.kDriveKinematics);

//   // An example trajectory to follow.  All units in meters.
//   auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
//       // Start at the origin facing the +X direction
//       frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
//       // Pass through these two interior waypoints, making an 's' curve path
//       {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
//       // End 3 meters straight ahead of where we started, facing forward
//       frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
//       // Pass the config
//       config);

//   frc::ProfiledPIDController<units::radians> thetaController{
//       AutoConstants::kPThetaController, 0, 0,
//       AutoConstants::kThetaControllerConstraints};

//   thetaController.EnableContinuousInput(units::radian_t(-wpi::math::pi),
//                                         units::radian_t(wpi::math::pi));

//   frc2::SwerveControllerCommand<4> swerveControllerCommand(
//       exampleTrajectory, [this]() { return m_drive.GetPose(); },

//       m_drive.kDriveKinematics,

//       frc2::PIDController(AutoConstants::kPXController, 0, 0),
//       frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController,

//       [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

//       {&m_drive});

//   // Reset odometry to the starting pose of the trajectory.
//   m_drive.ResetOdometry(exampleTrajectory.InitialPose());

//   // no auto
//   return new frc2::SequentialCommandGroup(
//       std::move(swerveControllerCommand), std::move(swerveControllerCommand),
//       frc2::InstantCommand(
//           [this]() {
//             m_drive.Drive(units::meters_per_second_t(0),
//                           units::meters_per_second_t(0),
//                           units::radians_per_second_t(0), false);
//           },
//           {}));
// }
