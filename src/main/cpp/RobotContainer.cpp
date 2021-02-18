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





RobotContainer::RobotContainer()
    : m_autonomousCommand(&m_drivetrain, &m_shooter, &actuator, &m_intake) 
    {
    //frc::SmartDashboard::PutData(&m_drivetrain);

  
    m_drivetrain.Log(); 

    //Dannalyn's shooter code
   // m_shooter.SetDefaultCommand(ShootCmdCls(&m_shooter/*, &m_joy*/)); //don't need in 2021
    actuator.SetDefaultCommand(TrimAngle(&xbox, &actuator, &joyRight)); // updated button
    m_compressor.SetDefaultCommand(beginCompressor(&m_compressor));
    m_intake.SetDefaultCommand(PickupBall(&m_intake, &xbox, &joyLeft)); // updated button
    m_climber.SetDefaultCommand(Climb(&m_climber, &xbox));

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

 frc2::JoystickButton(&xbox,9).WhenPressed(SpinWheel(&cSensor)); // updated button

 frc2::JoystickButton(&xbox, 5).WhenPressed(IntakeOut(&m_intake, true)); // updated button
 frc2::JoystickButton(&xbox, 6).WhenPressed(IntakeOut(&m_intake, false)); // updated button
 frc2::JoystickButton(&xbox, 10).WhenPressed(ThirdStageWheel(&cSensor)); // updated button
 frc2::JoystickButton(&joyLeft, 11).WhenHeld(shooterBackwards(&m_shooter));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
      frc::SimpleMotorFeedforward<units::meters>(
      DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics, 10_V);

  frc::TrajectoryConfig config{AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration}; 
  config.SetKinematics(DriveConstants::kDriveKinematics);
  config.AddConstraint(autoVoltageConstraint);

  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      //just go straight forward
      // {frc::Translation2d(1_m, 0_m)},


      //  // Pass through these two interior waypoints, making an 's' curve path
       {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
      //  {frc::Translation2d(1_m, 1_m)}, 

      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass the config                                                                      
      config);

      m_drivetrain.ResetOdometry(exampleTrajectory.InitialPose()); 

  frc2::RamseteCommand ramseteCommand(
      exampleTrajectory, [this]() { return m_drivetrain.GetPose(); },
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
      std::move(ramseteCommand),
      frc2::InstantCommand([this] { m_drivetrain.TankDriveVolts(0_V, 0_V); }, {}));



  // return new Autonomous(&m_drivetrain, &m_shooter, &actuator, &m_intake);
}


  void RobotContainer::PeriodicDebug(void) {
    frc::SmartDashboard::PutBoolean("xBox Check2 Button 5", xbox.GetRawButton(5));
  }
