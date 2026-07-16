// camera_p4.h — ESP32-P4 MIPI-CSI camera backend (OV5647 via esp_video + the P4
// hardware JPEG encoder). Drop-in for the OV2640 (esp_camera/DVP) interface that
// main.cpp's camCB + streaming use, so the board-agnostic capture/stream/motion
// path stays identical to the S3/CAM builds. Selected in place of OV2640 when
// CAMERA_MODEL_ESP32P4 is defined. Only compiled on the P4.
#pragma once
#if defined(CAMERA_MODEL_ESP32P4)

#include <stdint.h>
#include <stddef.h>
#include "driver/i2c_master.h"

// The shared control I2C bus (port 0, SCL=8/SDA=7): the camera's SCCB and the
// ES8311 audio codec both live on it. Created once by CameraP4::begin();
// NULL until then (p4_audio checks).
i2c_master_bus_handle_t p4I2cBus();

class CameraP4 {
public:
  // Init the CSI + OV5647 over SCCB, open /dev/video0, start streaming, and
  // create the HW JPEG encoder. Returns true on success.
  bool begin();
  // Tear down and re-create after a wedge (mirrors OV2640::reinit()).
  bool reinit();

  // Grab the next frame (blocks on the sensor) and HW-JPEG-encode it in place.
  void run();
  size_t   getSize() { return _jlen; }   // last encoded JPEG length
  uint8_t *getfb()   { return _jbuf; }   // last encoded JPEG buffer
  int getWidth()     { return (int)_w; }
  int getHeight()    { return (int)_h; }

private:
  bool startPipeline();
  void stopPipeline();

  int      _fd   = -1;
  uint32_t _w    = 0, _h = 0;
  uint8_t *_buf[3] = {nullptr, nullptr, nullptr};
  void    *_jenc = nullptr;   // jpeg_encoder_handle_t
  uint8_t *_jbuf = nullptr;   // encoder output buffer
  size_t   _jcap = 0;
  size_t   _jlen = 0;
};

#endif // CAMERA_MODEL_ESP32P4
