/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/beginCompressor.h"

beginCompressor::beginCompressor(CompressorObject* c_compressor) {
  m_compressor = c_compressor;
  AddRequirements(m_compressor);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void beginCompressor::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void beginCompressor::Execute() {
  m_compressor->startCompressor();
}

// Called once the command ends or is interrupted.
void beginCompressor::End(bool interrupted) {}

// Returns true when the command should end.
bool beginCompressor::IsFinished() { return false; }
