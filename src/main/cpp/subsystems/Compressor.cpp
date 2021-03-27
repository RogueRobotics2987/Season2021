#include "subsystems/Compressor.h"

CompressorObject::CompressorObject() {
    m_compressor = new frc::Compressor;
}

// This method will be called once per scheduler run
void CompressorObject::Periodic() {}

void CompressorObject::startCompressor() {
    m_compressor->SetClosedLoopControl(true);
}