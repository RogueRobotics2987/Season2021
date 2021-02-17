/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
// this is a whole bunch of new code I added to my personal branch that I wanna copy over eventually
void Robot::RobotPeriodic() { 
  frc2::CommandScheduler::GetInstance().Run();
  if(0)
    {std::cout << "Not active" << std::endl;}
  m_container.PeriodicDebug();
  }
//test
//test
/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  // NON CMD-BASED Approach:
  //drive robot 
    //joystick input
      // 2 axis
        // x and y
    //send joystick input to drivetrain
  
  // get the 2 joystick objects (joyLeft, joyRight)
  // get drivetrain object (DiffDrivetrain)
  // DiffDrivetrain.Drive(joyLeft.getYAxis(), joyRight.getYAxis())




}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
