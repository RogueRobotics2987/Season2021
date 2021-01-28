// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

<<<<<<< HEAD
#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include "RobotContainer.h"
=======
#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
>>>>>>> dcd3e367e17464515bc9e8b8b0327aae4e8d57ef

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
<<<<<<< HEAD
  void DisabledInit() override;
  void DisabledPeriodic() override;
=======
>>>>>>> dcd3e367e17464515bc9e8b8b0327aae4e8d57ef
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
<<<<<<< HEAD
  void TestPeriodic() override;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* m_autonomousCommand = nullptr;

  RobotContainer m_container;
=======
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
>>>>>>> dcd3e367e17464515bc9e8b8b0327aae4e8d57ef
};
