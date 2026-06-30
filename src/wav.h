// wav.h
//
// Pure, hardware-free WAV (RIFF/PCM) header writer. No Arduino includes, so it
// is unit-tested on the host (test/test_wav). audio_capture.cpp is the shell
// that supplies the buffer and the fixed 16 kHz/mono/16-bit format.

#pragma once

#include <stdint.h>
#include <string.h>

namespace wav {

// 44-byte canonical PCM WAV header.
static const uint32_t HEADER_BYTES = 44;

// Write a 44-byte PCM WAV header into p for pcmBytes of sample data.
// p must have room for HEADER_BYTES.
inline void writeHeader(uint8_t *p, uint32_t pcmBytes, uint32_t sampleRate,
                        uint16_t channels, uint16_t bitsPerSample) {
  uint16_t blockAlign = (uint16_t)(channels * (bitsPerSample / 8));
  uint32_t byteRate = sampleRate * blockAlign;
  uint32_t riffLen = 36 + pcmBytes;
  uint32_t fmtLen = 16;
  uint16_t fmt = 1; // PCM
  memcpy(p, "RIFF", 4);
  memcpy(p + 4, &riffLen, 4);
  memcpy(p + 8, "WAVEfmt ", 8);
  memcpy(p + 16, &fmtLen, 4);
  memcpy(p + 20, &fmt, 2);
  memcpy(p + 22, &channels, 2);
  memcpy(p + 24, &sampleRate, 4);
  memcpy(p + 28, &byteRate, 4);
  memcpy(p + 32, &blockAlign, 2);
  memcpy(p + 34, &bitsPerSample, 2);
  memcpy(p + 36, "data", 4);
  memcpy(p + 40, &pcmBytes, 4);
}

} // namespace wav
