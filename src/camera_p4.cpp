// camera_p4.cpp — see camera_p4.h. OV5647 MIPI-CSI capture via esp_video, frames
// HW-JPEG-encoded so the rest of Cyclops (ring, streaming, motion) sees the same
// JPEG frames it does from the DVP OV2640 on the S3/CAM.
#if defined(CAMERA_MODEL_ESP32P4)

#include "camera_p4.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cerrno>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "linux/videodev2.h"
#include "driver/jpeg_encode.h"

static const char *TAG = "cam_p4";

// ESP32-P4 EV-board SCCB (camera-control I2C) profile — matches the NANO.
#define CAM_SCCB_SCL_PIN   ((gpio_num_t)8)
#define CAM_SCCB_SDA_PIN   ((gpio_num_t)7)
#define CAM_BUF_COUNT      3
#define JPEG_QUALITY       80

// Shared control I2C bus: camera SCCB + the ES8311 audio codec both hang off
// port 0 (SCL=8/SDA=7) on the NANO. We create the bus ourselves (instead of
// letting esp_video do it) so p4_audio can attach the codec to the same handle
// — the i2c_master driver refuses a second bus on a claimed port.
static i2c_master_bus_handle_t s_i2c_bus = NULL;
i2c_master_bus_handle_t p4I2cBus() { return s_i2c_bus; }

static esp_video_init_csi_config_t s_csi_cfg = {
    .sccb_config = {
        .init_sccb = false,     // we own the bus; esp_video uses i2c_handle
        .i2c_handle = NULL,     // filled in startPipeline()
        .freq = 100000,
    },
    .reset_pin = GPIO_NUM_NC,
    .pwdn_pin  = GPIO_NUM_NC,
};
static const esp_video_init_config_t s_video_cfg = { .csi = &s_csi_cfg };
static bool s_video_inited = false;

bool CameraP4::startPipeline() {
  if (!s_i2c_bus) {
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = 0;
    bus_cfg.scl_io_num = CAM_SCCB_SCL_PIN;
    bus_cfg.sda_io_num = CAM_SCCB_SDA_PIN;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;
    if (i2c_new_master_bus(&bus_cfg, &s_i2c_bus) != ESP_OK) {
      ESP_LOGE(TAG, "i2c bus create failed");
      return false;
    }
  }
  if (!s_video_inited) {
    s_csi_cfg.sccb_config.i2c_handle = s_i2c_bus;
    if (esp_video_init(&s_video_cfg) != ESP_OK) { ESP_LOGE(TAG, "esp_video_init failed"); return false; }
    s_video_inited = true;
  }
  _fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDWR);
  if (_fd < 0) { ESP_LOGE(TAG, "open %s failed: %d", ESP_VIDEO_MIPI_CSI_DEVICE_NAME, errno); return false; }

  const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  struct v4l2_format fmt = {}; fmt.type = type;
  ioctl(_fd, VIDIOC_G_FMT, &fmt);
  _w = fmt.fmt.pix.width; _h = fmt.fmt.pix.height;

  struct v4l2_requestbuffers req = {}; req.count = CAM_BUF_COUNT; req.type = type; req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(_fd, VIDIOC_REQBUFS, &req) != 0) { ESP_LOGE(TAG, "REQBUFS failed"); return false; }
  for (int i = 0; i < CAM_BUF_COUNT; i++) {
    struct v4l2_buffer b = {}; b.type = type; b.memory = V4L2_MEMORY_MMAP; b.index = i;
    if (ioctl(_fd, VIDIOC_QUERYBUF, &b) != 0) return false;
    _buf[i] = (uint8_t *)mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, b.m.offset);
    if (_buf[i] == MAP_FAILED) { _buf[i] = nullptr; return false; }
    if (ioctl(_fd, VIDIOC_QBUF, &b) != 0) return false;
  }
  if (ioctl(_fd, VIDIOC_STREAMON, &type) != 0) { ESP_LOGE(TAG, "STREAMON failed"); return false; }

  if (!_jenc) {
    jpeg_encode_engine_cfg_t eng = { .intr_priority = 0, .timeout_ms = 100 };
    jpeg_encoder_handle_t enc = nullptr;
    if (jpeg_new_encoder_engine(&eng, &enc) != ESP_OK) { ESP_LOGE(TAG, "jpeg engine failed"); return false; }
    _jenc = enc;
    jpeg_encode_memory_alloc_cfg_t mcfg = { .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER };
    _jbuf = (uint8_t *)jpeg_alloc_encoder_mem(_w * _h, &mcfg, &_jcap);
    if (!_jbuf) { ESP_LOGE(TAG, "jpeg out alloc failed"); return false; }
  }
  ESP_LOGI(TAG, "OV5647 pipeline up: %ux%u, jpeg cap=%u", (unsigned)_w, (unsigned)_h, (unsigned)_jcap);
  return true;
}

void CameraP4::stopPipeline() {
  if (_fd >= 0) {
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(_fd, VIDIOC_STREAMOFF, &type);
    close(_fd);
    _fd = -1;
  }
}

bool CameraP4::begin() { return startPipeline(); }

bool CameraP4::reinit() {
  stopPipeline();
  return startPipeline();
}

void CameraP4::run() {
  _jlen = 0;
  if (_fd < 0) return;
  const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  struct v4l2_buffer b = {}; b.type = type; b.memory = V4L2_MEMORY_MMAP;
  if (ioctl(_fd, VIDIOC_DQBUF, &b) != 0) return;
  if ((b.flags & V4L2_BUF_FLAG_DONE) && b.index < CAM_BUF_COUNT) {
    jpeg_encode_cfg_t cfg = { .height = _h, .width = _w,
                              .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
                              .sub_sample = JPEG_DOWN_SAMPLING_YUV420,
                              .image_quality = JPEG_QUALITY };
    uint32_t out = 0;
    if (jpeg_encoder_process((jpeg_encoder_handle_t)_jenc, &cfg, _buf[b.index], b.bytesused,
                             _jbuf, _jcap, &out) == ESP_OK)
      _jlen = out;
  }
  ioctl(_fd, VIDIOC_QBUF, &b);
}

#endif // CAMERA_MODEL_ESP32P4
