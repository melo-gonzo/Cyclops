// board_stubs.cpp
//
// On boards without the PDM mic / SD layer (the whole audio_capture.cpp module
// is compiled out), the video recorder + motion detector still run: motion
// detection works off the PSRAM frame ring and needs no SD. video_record.cpp
// links against a handful of SD and audio-cross-trigger symbols that normally
// live in audio_capture.cpp; provide honest no-op implementations here so motion
// detection works while SD-backed clip saving and audio cross-triggering simply
// do nothing. Compiled only where the audio module is absent.

// (The ESP32-P4 provides real SD symbols in p4_sd.cpp — SDMMC card — so these
// stubs compile only where there is genuinely no SD/audio hardware.)
#if !defined(CAMERA_MODEL_XIAO_ESP32S3) && !defined(CAMERA_MODEL_ESP32P4)

#include "audio_capture.h"

bool sdIsAvailable() { return false; }      // no SD card wired on this board
void sdRequestBootSkip() {}                 // no SD phase to skip
SemaphoreHandle_t sdGetMutex() { return NULL; }
bool sdLowSpace() { return false; }
void sdNotifyWriteFailed() {}
uint32_t audioTriggerFromVideo() { return 0; } // no audio to cross-trigger

#endif // !CAMERA_MODEL_XIAO_ESP32S3 && !CAMERA_MODEL_ESP32P4
