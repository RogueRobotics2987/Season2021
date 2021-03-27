#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Compressor.h>

class CompressorObject : public frc2::SubsystemBase {
 public:
  CompressorObject();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  void startCompressor();

 private:
  frc::Compressor* m_compressor = nullptr;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};