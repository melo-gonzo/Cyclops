// p4_audio.h — ESP32-P4-NANO microphone backend (ES8311 codec).
//
// The NANO's analog mic feeds an ES8311 codec: I2S on MCLK=13 SCLK=12 LRCK=11
// ASDOUT(codec->P4)=9 DSDIN(P4->codec)=10, control on the SAME I2C bus as the
// camera SCCB (GPIO7/8 — camera_p4 owns the bus, see p4I2cBus()). This module
// brings the codec + an I2S STD RX channel up at the fleet's audio format
// (16 kHz, 16-bit, mono) and exposes a blocking read with the same contract as
// the XIAO's i2s_read: fill the buffer completely.
//
// audio_capture.cpp's capture task calls p4AudioRead() where the XIAO calls
// i2s_read — the only board-specific seam in the audio pipeline.
#pragma once
#if defined(CAMERA_MODEL_ESP32P4)

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Bring up the codec (ADC path) + I2S channel. Camera must be initialized
// first (it creates the shared I2C bus). Returns false if the codec doesn't
// probe/configure — audio features degrade exactly like a failed PDM init.
bool p4AudioInit();

// Blocking read of 16-bit mono samples @16kHz. Fills `len` bytes completely
// (or returns false on driver error). Mirrors i2s_read(portMAX_DELAY).
bool p4AudioRead(void *dst, size_t len, size_t *bytesRead);

// Input gain in dB (maps the fleet's mic-gain slider onto the codec's analog
// gain instead of software multiplication).
void p4AudioSetGainDb(float db);

#endif // CAMERA_MODEL_ESP32P4
