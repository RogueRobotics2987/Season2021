/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/CommandScheduler.h>
#include "commands/TankDrive.h"
#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <iostream>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "Constants.h" 
#include "commands/SpinWheel.h"
#include "commands/PIDShoot.h"
#include "commands/SpinWheel.h" 
#include "commands/IntakeOut.h" 
#include "commands/startConveyor.h" 
#include "commands/shooterBackwards.h" 
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>





RobotContainer::RobotContainer()
    : m_autonomousCommand(&m_drivetrain, &m_shooter, &actuator, &m_intake) 
    {
    //frc::SmartDashboard::PutData(&m_drivetrain);






    m_drivetrain.Log(); 

    //Dannalyn's shooter code
   // m_shooter.SetDefaultCommand(ShootCmdCls(&m_shooter/*, &m_joy*/)); 
    actuator.SetDefaultCommand(TrimAngle(&xbox, &actuator, &joyRight)); // updated button
    m_compressor.SetDefaultCommand(beginCompressor(&m_compressor));
    m_intake.SetDefaultCommand(PickupBall(&m_intake, &xbox, &joyLeft)); // updated button
    m_climber.SetDefaultCommand(Climb(&m_climber, &xbox, &joyLeft));

  // m_drivetrain.Log();
  ConfigureButtonBindings();
  m_drivetrain.SetDefaultCommand(TankDrive(&m_drivetrain, &joyRight, &joyLeft));
  //std::cout << "Configure buttons" << std::endl;
  // Configure the button bindings
  //std::cout << "Constructor" << std::endl; 
   //j1.WhenPressed(new setHeight(10, &m_elevator));

}




void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  //std::cout << "Configure button bindings" << std::endl;

 //j1.WhenPressed(new setHeight(10.0, &m_elevator)); 
 //j1.WhenPressed(SpinWheel(&cSensor)); 
 
 
 //Sydneys Intake Code
 //frc2::JoystickButton(&xbox,1).WhenHeld(PickupBall(&m_intake));

frc2::JoystickButton(&xbox,3).WhenHeld(PIDShoot(&m_shooter, &m_intake)); // updated button
 
 
 //frc2::JoystickButton(&xbox, 1).WhenHeld(startConveyor(&m_intake, .3)); //shoot, updated button
 //frc2::JoystickButton(&joyLeft, 11).WhenHeld(startConveyor(&m_intake, -.3)); //backwards conveyor, updated button

 frc2::JoystickButton(&joyLeft,10).WhenPressed(SpinWheel(&cSensor)); // updated button

 frc2::JoystickButton(&xbox, 5).WhenPressed(IntakeOut(&m_intake, true)); // updated button
 frc2::JoystickButton(&xbox, 6).WhenPressed(IntakeOut(&m_intake, false)); // updated button
 frc2::JoystickButton(&xbox, 10).WhenPressed(ThirdStageWheel(&cSensor)); // updated button
 frc2::JoystickButton(&joyLeft, 11).WhenHeld(shooterBackwards(&m_shooter));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {

//PATHWEAVER JSON ATTEMPT
    wpi::SmallString<64> toA3;
    frc::filesystem::GetDeployDirectory(toA3);
    wpi::sys::path::append(toA3, "paths/toA3.wpilib.json");
    frc::Trajectory toA3trajectory = frc::TrajectoryUtil::FromPathweaverJson(toA3);


    wpi::SmallString<64> toA6;
    frc::filesystem::GetDeployDirectory(toA6);
    wpi::sys::path::append(toA6, "paths/toA6.wpilib.json");
    frc::Trajectory toA6trajectory = frc::TrajectoryUtil::FromPathweaverJson(toA6);


    wpi::SmallString<64> toA9;
    frc::filesystem::GetDeployDirectory(toA9);
    wpi::sys::path::append(toA9, "paths/toA9.wpilib.json");
    frc::Trajectory toA9trajectory = frc::TrajectoryUtil::FromPathweaverJson(toA9);



  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
      frc::SimpleMotorFeedforward<units::meters>(
      DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics, 10_V);

  frc::TrajectoryConfig config{AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration}; 
  config.SetKinematics(DriveConstants::kDriveKinematics);
  config.AddConstraint(autoVoltageConstraint);


//COMMENT IN/OUT WHEN DOING AUTO RUN / GALACTIC SEARCH
  // config.SetReversed(true);

  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      //just go straight forward
      // {frc::Translation2d(1_m, 0_m)},


//SYDNEYS FIRST ATTEMPT AT BARREL
     {frc::Translation2d(2.76_m, -0.01_m), 
      frc::Translation2d(3.11_m, -1.17_m), 
      frc::Translation2d(1.72_m, -1.39_m),
      frc::Translation2d(1.67_m, -0.27_m),
      frc::Translation2d(5.41_m, 0.27_m),
      frc::Translation2d(5.19_m, 1.5_m),
      frc::Translation2d(4.01_m, 1.42_m),
      frc::Translation2d(3.7_m, 0.4_m),
      frc::Translation2d(5.87_m, -1.47_m),
      frc::Translation2d(7_m, -0.95_m),
      frc::Translation2d(6.52_m, -0.08_m),
      frc::Translation2d(1.34_m, -0.1_m)
     },


      //  // Pass through these two interior waypoints, making an 's' curve path
      //  {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
      //  {frc::Translation2d(1_m, 1_m)}, 

      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass the config                                                                      
      config);


//COMMENT IN/OUT FOR AUTO STUFF/GALACTIC SEARCH
  //m_intake.IntakeBall(1.0);


  m_drivetrain.ResetOdometry(toA3trajectory.InitialPose()); 


  frc2::RamseteCommand ramseteCommandA3(
      toA3trajectory, [this]() { return m_drivetrain.GetPose(); },
      frc::RamseteController(AutoConstants::kRamseteB,
                             AutoConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics,
      [this] { return m_drivetrain.GetWheelSpeeds(); },
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      [this](auto left, auto right) { m_drivetrain.TankDriveVolts(left, right); },
      {&m_drivetrain});

  frc2::RamseteCommand ramseteCommandA6(
      toA6trajectory, [this]() { return m_drivetrain.GetPose(); },
      frc::RamseteController(AutoConstants::kRamseteB,
                             AutoConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics,
      [this] { return m_drivetrain.GetWheelSpeeds(); },
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      [this](auto left, auto right) { m_drivetrain.TankDriveVolts(left, right); },
      {&m_drivetrain});

  frc2::RamseteCommand ramseteCommandA9(
      toA9trajectory, [this]() { return m_drivetrain.GetPose(); },
      frc::RamseteController(AutoConstants::kRamseteB,
                             AutoConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics,
      [this] { return m_drivetrain.GetWheelSpeeds(); },
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      [this](auto left, auto right) { m_drivetrain.TankDriveVolts(left, right); },
      {&m_drivetrain});



      return new frc2::SequentialCommandGroup(
      std::move(ramseteCommandA3),
      std::move(ramseteCommandA6),
      std::move(ramseteCommandA9),
      frc2::InstantCommand([this] { m_drivetrain.TankDriveVolts(0_V, 0_V); }, {}));



  // return new Autonomous(&m_drivetrain, &m_shooter, &actuator, &m_intake);
}


  void RobotContainer::PeriodicDebug(void) {
    frc::SmartDashboard::PutBoolean("xBox Check2 Button 5", xbox.GetRawButton(5));
  }
