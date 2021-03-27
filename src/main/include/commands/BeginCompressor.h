#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Compressor.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class beginCompressor
    : public frc2::CommandHelper<frc2::CommandBase, beginCompressor> {
 public:
  beginCompressor(CompressorObject* c_compressor);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  CompressorObject* m_compressor = nullptr;
};

