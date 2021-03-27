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
  std::cout<<"***DJO*** I'm running GetAutonomousCommand()" << std::endl;


  limelightTablerri = NetworkTable::GetTable("limelight-rri"); 
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-rri") -> PutNumber("pipeline", 1);
  std::cout<<"***DJO*** I'm running pipeline update to pipeline 1" << std::endl;
  txi = nt::NetworkTableInstance::GetDefault().GetTable("limelight-rri")->GetNumber("tx", -20.0); 
  tyi = limelightTablerri->GetNumber("ty", 0.0); 
  frc::SmartDashboard::PutNumber("Galactic X", txi);
  frc::SmartDashboard::PutNumber("Galactic Y", tyi);
  // if (txi > 3.0 && tyi < -9.0) {
  //   RedA = true;
  // } else  { // if (txi < 3)
  //   RedA = false;
  // }

  if (txi > -10.0) { 
      RedA = true;
  } else  { 
      RedA = false;
  }
  frc::SmartDashboard::PutBoolean("RedA", RedA);

//JSON FILES AND PATHS
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

    wpi::SmallString<64> toFinish;
    frc::filesystem::GetDeployDirectory(toFinish);
    wpi::sys::path::append(toFinish, "paths/toFinish.wpilib.json");
    frc::Trajectory toFinishTrajectory = frc::TrajectoryUtil::FromPathweaverJson(toFinish);





    wpi::SmallString<64> barrelRegFile;
    frc::filesystem::GetDeployDirectory(barrelRegFile);
    wpi::sys::path::append(barrelRegFile, "paths/barrel_longBlue_fewPoints.wpilib.json");
    frc::Trajectory barrelRegTrajectory = frc::TrajectoryUtil::FromPathweaverJson(barrelRegFile);






    wpi::SmallString<64> fancyBarrelStartA;
    frc::filesystem::GetDeployDirectory(fancyBarrelStartA);
    wpi::sys::path::append(fancyBarrelStartA, "paths/fancyBarrelStartA.wpilib.json");
    frc::Trajectory fancyBarrelStartATrajectory = frc::TrajectoryUtil::FromPathweaverJson(fancyBarrelStartA);

    wpi::SmallString<64> fancyBarrelEndA;
    frc::filesystem::GetDeployDirectory(fancyBarrelEndA);
    wpi::sys::path::append(fancyBarrelEndA, "paths/fancyBarrelEndA.wpilib.json");
    frc::Trajectory fancyBarrelEndATrajectory = frc::TrajectoryUtil::FromPathweaverJson(fancyBarrelEndA);

    wpi::SmallString<64> fancyBarrelStartB;
    frc::filesystem::GetDeployDirectory(fancyBarrelStartB);
    wpi::sys::path::append(fancyBarrelStartB, "paths/fancyBarrelStartB.wpilib.json");
    frc::Trajectory fancyBarrelStartBTrajectory = frc::TrajectoryUtil::FromPathweaverJson(fancyBarrelStartB);

    wpi::SmallString<64> fancyBarrelMidB;
    frc::filesystem::GetDeployDirectory(fancyBarrelMidB);
    wpi::sys::path::append(fancyBarrelMidB, "paths/fancyBarrelMidB.wpilib.json");
    frc::Trajectory fancyBarrelMidBTrajectory = frc::TrajectoryUtil::FromPathweaverJson(fancyBarrelMidB);

    wpi::SmallString<64> fancyBarrelEndB;
    frc::filesystem::GetDeployDirectory(fancyBarrelEndB);
    wpi::sys::path::append(fancyBarrelEndB, "paths/fancyBarrelEndB.wpilib.json");
    frc::Trajectory fancyBarrelEndBTrajectory = frc::TrajectoryUtil::FromPathweaverJson(fancyBarrelEndB);





    wpi::SmallString<64> gSearchBlueA;
    frc::filesystem::GetDeployDirectory(gSearchBlueA);
    wpi::sys::path::append(gSearchBlueA, "paths/galacticSearchBlueA.wpilib.json");
    frc::Trajectory gSearchBlueATrajectory = frc::TrajectoryUtil::FromPathweaverJson(gSearchBlueA);

    wpi::SmallString<64> gSearchRedA;
    frc::filesystem::GetDeployDirectory(gSearchRedA);
    wpi::sys::path::append(gSearchRedA, "paths/galacticSearchRedA.wpilib.json");
    frc::Trajectory gSearchRedATrajectory = frc::TrajectoryUtil::FromPathweaverJson(gSearchRedA);






    wpi::SmallString<64> extraBarrelStart;
    frc::filesystem::GetDeployDirectory(extraBarrelStart);
    wpi::sys::path::append(extraBarrelStart, "paths/barrelStart.wpilib.json");
    frc::Trajectory extraBarrelStartTrajectory = frc::TrajectoryUtil::FromPathweaverJson(extraBarrelStart);

    wpi::SmallString<64> extraBarrelEnd;
    frc::filesystem::GetDeployDirectory(extraBarrelEnd);
    wpi::sys::path::append(extraBarrelEnd, "paths/barrelEndExtra.wpilib.json");
    frc::Trajectory extraBarrelEndTrajectory = frc::TrajectoryUtil::FromPathweaverJson(extraBarrelEnd);




    wpi::SmallString<64> GSARedStartFile;
    frc::filesystem::GetDeployDirectory(GSARedStartFile);
    wpi::sys::path::append(GSARedStartFile, "paths/GSARedStart.wpilib.json");
    frc::Trajectory GSARedStartTraj = frc::TrajectoryUtil::FromPathweaverJson(GSARedStartFile);

    wpi::SmallString<64> GSARedEndFile;
    frc::filesystem::GetDeployDirectory(GSARedEndFile);
    wpi::sys::path::append(GSARedEndFile, "paths/GSARedEnd.wpilib.json");
    frc::Trajectory GSARedEndTraj = frc::TrajectoryUtil::FromPathweaverJson(GSARedEndFile);



    wpi::SmallString<64> GSABlueStartFile;
    frc::filesystem::GetDeployDirectory(GSABlueStartFile);
    wpi::sys::path::append(GSABlueStartFile, "paths/GSABlueStart.wpilib.json");
    frc::Trajectory GSABlueStartTraj = frc::TrajectoryUtil::FromPathweaverJson(GSABlueStartFile);

    wpi::SmallString<64> GSABlueMidFile;
    frc::filesystem::GetDeployDirectory(GSABlueMidFile);
    wpi::sys::path::append(GSABlueMidFile, "paths/GSABlueMid.wpilib.json");
    frc::Trajectory GSABlueMidTraj = frc::TrajectoryUtil::FromPathweaverJson(GSABlueMidFile);

    wpi::SmallString<64> GSABlueEndFile;
    frc::filesystem::GetDeployDirectory(GSABlueEndFile);
    wpi::sys::path::append(GSABlueEndFile, "paths/GSABlueEnd.wpilib.json");
    frc::Trajectory GSABlueEndTraj = frc::TrajectoryUtil::FromPathweaverJson(GSABlueEndFile);




    wpi::SmallString<64> slalomFile;
    frc::filesystem::GetDeployDirectory(slalomFile);
    wpi::sys::path::append(slalomFile, "paths/Slalem_v001.wpilib.json");
    frc::Trajectory slalomTraj = frc::TrajectoryUtil::FromPathweaverJson(slalomFile);






  frc::CentripetalAccelerationConstraint autoCentripConstraint = frc::CentripetalAccelerationConstraint(
      2.0_mps_sq
  );


  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
      frc::SimpleMotorFeedforward<units::meters>(
      DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics, 10_V);

  frc::TrajectoryConfig config{AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration}; 
  config.SetKinematics(DriveConstants::kDriveKinematics);
  config.AddConstraint(autoVoltageConstraint);
  config.SetReversed(false);
  config.AddConstraint(autoCentripConstraint);
  

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
      frc::Translation2d(6.52_m, -0.1_m),
      frc::Translation2d(1.34_m, -0.1_m)
     },
      //  // Pass through these two interior waypoints, making an 's' curve path
      //  {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
      //  {frc::Translation2d(1_m, 1_m)}, 
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(180_deg)),
      // Pass the config                                                                      
      config);


//POINT BY POINT TEST
  auto pointTraj = frc::TrajectoryGenerator::GenerateTrajectory(
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(-26.56505118_deg + 180.0_deg)),
     {
      frc::Translation2d(3.583_m, -1.618_m),
      frc::Translation2d(3.686_m, 0.427_m)
     },
      frc::Pose2d(8.352_m, 0.427_m, frc::Rotation2d(-7.917293025_deg + 180.0_deg)),
      config
  );

  auto pointDebug = frc::TrajectoryGenerator::GenerateTrajectory(
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(180.0_deg)),
     {
      frc::Translation2d(2.5_m, 0_m),
      frc::Translation2d(5_m, 0_m),
      frc::Translation2d(5_m, -1.5_m)
     },
      frc::Pose2d(5_m, -3_m, frc::Rotation2d(90.0_deg)),
      config
  );

  frc2::RamseteCommand ramseteCommandExample(
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

  frc2::RamseteCommand ramseteCommandBounceFinish(
      toFinishTrajectory, [this]() { return m_drivetrain.GetPose(); },
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

  frc2::RamseteCommand ramseteCommandBarrelReg(
      barrelRegTrajectory, [this]() { return m_drivetrain.GetPose(); },
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

  frc2::RamseteCommand ramseteCmdFancyStartA(
      fancyBarrelStartATrajectory, [this]() { return m_drivetrain.GetPose(); },
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

  frc2::RamseteCommand ramseteCmdFancyEndA(
      fancyBarrelEndATrajectory, [this]() { return m_drivetrain.GetPose(); },
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

  frc2::RamseteCommand ramseteCmdFancyStartB(
      fancyBarrelStartBTrajectory, [this]() { return m_drivetrain.GetPose(); },
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

  frc2::RamseteCommand ramseteCmdFancyMidB(
      fancyBarrelMidBTrajectory, [this]() { return m_drivetrain.GetPose(); },
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

  frc2::RamseteCommand ramseteCmdFancyEndB(
      fancyBarrelEndBTrajectory, [this]() { return m_drivetrain.GetPose(); },
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

  frc2::RamseteCommand ramseteCmdGalacticSearchBlueA(
      gSearchBlueATrajectory, [this]() { return m_drivetrain.GetPose(); },
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

  frc2::RamseteCommand ramseteCmdGalacticSearchRedA(
      gSearchRedATrajectory, [this]() { return m_drivetrain.GetPose(); },
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

  frc2::RamseteCommand ramseteCmdBarrelExtraEnd(
      extraBarrelEndTrajectory, [this]() { return m_drivetrain.GetPose(); },
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

  frc2::RamseteCommand ramseteCmdBarrelExtraStart(
      extraBarrelStartTrajectory, [this]() { return m_drivetrain.GetPose(); },
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







  frc2::RamseteCommand ramCmdGSARedStart(
      GSARedStartTraj, [this]() { return m_drivetrain.GetPose(); },
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

  frc2::RamseteCommand ramCmdGSARedEnd(
      GSARedEndTraj, [this]() { return m_drivetrain.GetPose(); },
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





  frc2::RamseteCommand ramCmdGSABlueStart(
      GSABlueStartTraj, [this]() { return m_drivetrain.GetPose(); },
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

  frc2::RamseteCommand ramCmdGSABlueMid(
      GSABlueMidTraj, [this]() { return m_drivetrain.GetPose(); },
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

  frc2::RamseteCommand ramCmdGSABlueEnd(
      GSABlueEndTraj, [this]() { return m_drivetrain.GetPose(); },
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




  frc2::RamseteCommand ramCmdPointTest(
      pointTraj, [this]() { return m_drivetrain.GetPose(); },
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

  frc2::RamseteCommand ramCmdPointDebug(
      pointDebug, [this]() { return m_drivetrain.GetPose(); },
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








  frc2::RamseteCommand ramCmdSlalom(
      slalomTraj, [this]() { return m_drivetrain.GetPose(); },
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




   limelightTablerri = NetworkTable::GetTable("limelight-rri"); 
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-rri") -> PutNumber("pipeline", 1);
    txi = limelightTablerri->GetNumber("tx", 0.0); 
    tyi = limelightTablerri->GetNumber("ty", 0.0); 
    frc::SmartDashboard::PutNumber("Galactic X", txi);
    frc::SmartDashboard::PutNumber("Galactic Y", tyi);

    if (txi < -1.5) { 
        RedA = true;
    } else  { 
        RedA = false;
    }
    frc::SmartDashboard::PutBoolean("RedA", RedA);







    //  m_drivetrain.ResetOdometry(toA3trajectory.InitialPose()); 
     m_drivetrain.ResetOdometry(barrelRegTrajectory.InitialPose());
        //  m_drivetrain.ResetOdometry(fancyBarrelStartATrajectory.InitialPose());
        //  m_drivetrain.ResetOdometry(fancyBarrelStartBTrajectory.InitialPose());
        //  m_drivetrain.ResetOdometry(toFinishTrajectory.InitialPose());
    //  m_drivetrain.ResetOdometry(gSearchBlueATrajectory.InitialPose());
    //  m_drivetrain.ResetOdometry(gSearchRedATrajectory.InitialPose());
        //  m_drivetrain.ResetOdometry(extraBarrelStartTrajectory.InitialPose());
        //  m_drivetrain.ResetOdometry(GSARedStartTraj.InitialPose());
        //  m_drivetrain.ResetOdometry(GSABlueStartTraj.InitialPose());
    //  m_drivetrain.ResetOdometry(pointTraj.InitialPose());
    //  m_drivetrain.ResetOdometry(pointDebug.InitialPose());
        //  m_drivetrain.ResetOdometry(slalomTraj.InitialPose()); 




  frc2::SequentialCommandGroup* basicBounceGroup = new frc2::SequentialCommandGroup(
      std::move(ramseteCommandA3),
      std::move(ramseteCommandA6),
      std::move(ramseteCommandA9),
      std::move(ramseteCommandBounceFinish),
      frc2::InstantCommand([this] { m_drivetrain.TankDriveVolts(0_V, 0_V); }, {})
      );




  frc2::SequentialCommandGroup* regularBarrelGroup = new frc2::SequentialCommandGroup(
      std::move(ramseteCommandBarrelReg),
      frc2::InstantCommand([this] { m_drivetrain.TankDriveVolts(0_V, 0_V); }, {})
      );




  frc2::SequentialCommandGroup* pointsGroup = new frc2::SequentialCommandGroup(
      std::move(ramseteCommandExample),
      frc2::InstantCommand([this] { m_drivetrain.TankDriveVolts(0_V, 0_V); }, {})
      );




  frc2::SequentialCommandGroup* fancyBarrelGroupA = new frc2::SequentialCommandGroup(
    IntakeOut(&m_intake, true),
    frc2::ParallelRaceGroup(
        AutoPickup(&m_intake, true, 20.0),
        std::move(ramseteCmdFancyStartA)
        
    ),
    frc2::ParallelCommandGroup(
        AutoTrimAngle(&actuator, true),
        AutoShoot(&m_shooter, &actuator, &m_intake, 0.0, 9.0)
    ),
    std::move(ramseteCmdFancyEndA)
  );






  frc2::SequentialCommandGroup* fancyBarrelGroupB = new frc2::SequentialCommandGroup(
    IntakeOut(&m_intake, true),
    frc2::ParallelRaceGroup(
      std::move(ramseteCmdFancyStartB),
      AutoPickup(&m_intake, true, 20.0)
    ),
    std::move(ramseteCmdFancyMidB),
    frc2::ParallelCommandGroup(
        AutoTrimAngle(&actuator, true),
        AutoShoot(&m_shooter, &actuator, &m_intake, 0.0, 10.0)
    ),
    std::move(ramseteCmdFancyEndB),
    frc2::InstantCommand([this] { m_drivetrain.TankDriveVolts(0_V, 0_V); }, {})
  );





//   frc2::SequentialCommandGroup* doNothing(
//       frc::Translation2d(0_m, 0_m),
//       frc::Translation2d(0_m, 0_m)
//   );

  frc2::ParallelRaceGroup* gSearchBlueAGroup = new frc2::ParallelRaceGroup(
      std::move(ramseteCmdGalacticSearchBlueA),
      frc2::SequentialCommandGroup(
          IntakeOut(&m_intake, true),
          AutoPickup(&m_intake, true, 30)
      )
  );
    




  frc2::ParallelRaceGroup* gSearchRedAGroup = new frc2::ParallelRaceGroup(
      std::move(ramseteCmdGalacticSearchRedA),
      frc2::SequentialCommandGroup(
          IntakeOut(&m_intake, true),
          AutoPickup(&m_intake, true, 30)
      )
  );


  frc2::SequentialCommandGroup* extraBarrel = new frc2::SequentialCommandGroup(
      std::move(ramseteCmdBarrelExtraStart),
      std::move(ramseteCmdBarrelExtraEnd)
  );



  frc2::ParallelRaceGroup* GSARedGroup = new frc2::ParallelRaceGroup(
    frc2::SequentialCommandGroup(
        std::move(ramCmdGSARedStart),
        std::move(ramCmdGSARedEnd)
    ),
    frc2::SequentialCommandGroup(
        IntakeOut(&m_intake, true),
        AutoPickup(&m_intake, true, 30)
    )
  );

  frc2::ParallelRaceGroup* GSABlueGroup = new frc2::ParallelRaceGroup(
    frc2::SequentialCommandGroup(
        std::move(ramCmdGSABlueStart),
        std::move(ramCmdGSABlueMid),
        std::move(ramCmdGSABlueEnd)
    ),
    frc2::SequentialCommandGroup(
        IntakeOut(&m_intake, true),
        AutoPickup(&m_intake, true, 30)
    )
  );



  frc2::ParallelRaceGroup* pointTestGroup = new frc2::ParallelRaceGroup(
      std::move(ramCmdPointTest),
      frc2::SequentialCommandGroup(
          IntakeOut(&m_intake, true),
          AutoPickup(&m_intake, true, 30)
      )
  );

  frc2::ParallelRaceGroup* pointDebugGroup = new frc2::ParallelRaceGroup(
      std::move(ramCmdPointDebug),
      frc2::SequentialCommandGroup(
          IntakeOut(&m_intake, true),
          AutoPickup(&m_intake, true, 30)
      )
  );



  frc2::SequentialCommandGroup* slalomGroup = new frc2::SequentialCommandGroup(
      std::move(ramCmdSlalom),
      frc2::InstantCommand([this] { m_drivetrain.TankDriveVolts(0_V, 0_V); }, {})
 );





//   autoChooser.SetDefaultOption("Board A", gSearchBlueAGroup);
//   autoChooser.AddOption("Board B", gSearchRedAGroup);
//   frc2::SmartDashboard::PutData(autoChooser);
    // if (RedA == true){
    //     return gSearchRedAGroup;
    // }else{
    //     return gSearchBlueAGroup;
    // };




    // if(RedA == true){
    //     return gSearchRedAGroup;
    // } else {        // Blue map
    //     return gSearchBlueAGroup;
    // }
    
    return regularBarrelGroup;
    


    // if(RedA == true){
    //     return gSearchRedAGroup;
    // } else {        // Blue map
    //     return gSearchBlueAGroup;
    // }




}


  void RobotContainer::PeriodicDebug(void) {
    frc::SmartDashboard::PutBoolean("xBox Check2 Button 5", xbox.GetRawButton(5));
  }
