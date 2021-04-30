#include "commands/BeginCompressor.h"

beginCompressor::beginCompressor(CompressorObject* c_compressor):m_compressor(c_compressor) {
//   m_compressor = c_compressor;
  AddRequirements(m_compressor);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void beginCompressor::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void beginCompressor::Execute() {
    // std::cout << "Runing The Compressor" << std::endl;
  m_compressor->startCompressor();
}

// Called once the command ends or is interrupted.
void beginCompressor::End(bool interrupted) {}

// Returns true when the command should end.
bool beginCompressor::IsFinished() { return false; }