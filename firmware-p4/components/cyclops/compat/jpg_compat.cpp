// P4 implementations of the two esp32-camera conversion entry points the shared
// Cyclops code uses (see esp_jpg_decode.h / img_converters.h in this dir).
//
// esp_jpg_decode(): same tjpgd decoder family as the fleet's S3 build, via the
// esp_jpeg component (buffer-in → RGB888 buffer-out at the requested scale),
// delivered to the caller through the old block-callback protocol: a
// data==NULL dims announcement, one full-frame RGB888 block, a data==NULL end
// marker. video_record.cpp's callbacks accept arbitrary block geometry, so one
// full-frame block is valid.
//
// fmt2jpg(): thumbnail-sized RGB888 → JPEG via the vendored jpge encoder
// (software; the P4 HW encoder needs 16-px-aligned dims, thumbnails are 100x100).
#include "esp_jpg_decode.h"
#include "img_converters.h"

#include <string.h>
#include <stdlib.h>
#include "esp_heap_caps.h"
#include "jpeg_decoder.h"
#include "jpge.h"

static void *psAlloc(size_t n) {
  void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
  if (!p) p = malloc(n);
  return p;
}

extern "C" esp_err_t esp_jpg_decode(size_t len, jpg_scale_t scale,
                                    jpg_reader_cb reader, jpg_writer_cb writer, void *arg) {
  if (!len || !reader || !writer) return ESP_ERR_INVALID_ARG;

  // Pull the whole source through the reader (the callers stream from a
  // contiguous ring snapshot; one copy keeps the old streaming API intact).
  uint8_t *src = (uint8_t *)psAlloc(len);
  if (!src) return ESP_ERR_NO_MEM;
  size_t got = 0;
  while (got < len) {
    size_t n = reader(arg, got, src + got, len - got);
    if (n == 0) break;
    got += n;
  }
  if (got < len) { free(src); return ESP_FAIL; }

  esp_jpeg_image_cfg_t cfg = {};
  cfg.indata = src;
  cfg.indata_size = len;
  cfg.out_format = JPEG_IMAGE_FORMAT_RGB888;
  cfg.out_scale = (esp_jpeg_image_scale_t)scale;  // enums align: NONE/2X/4X/8X
  esp_jpeg_image_output_t info = {};
  if (esp_jpeg_get_image_info(&cfg, &info) != ESP_OK) { free(src); return ESP_FAIL; }

  uint16_t outW = info.width, outH = info.height;
  // Dims announcement — the caller aborts here on a resolution mismatch.
  if (!writer(arg, 0, 0, outW, outH, NULL)) { free(src); return ESP_FAIL; }

  size_t rgbLen = (size_t)outW * outH * 3;
  uint8_t *rgb = (uint8_t *)psAlloc(rgbLen);
  if (!rgb) { free(src); return ESP_ERR_NO_MEM; }
  cfg.outbuf = rgb;
  cfg.outbuf_size = rgbLen;
  esp_jpeg_image_output_t out = {};
  esp_err_t err = esp_jpeg_decode(&cfg, &out);
  bool ok = (err == ESP_OK);
  if (ok) ok = writer(arg, 0, 0, outW, outH, rgb);   // one full-frame block
  if (ok) writer(arg, 0, 0, 0, 0, NULL);             // end marker
  free(rgb);
  free(src);
  return ok ? ESP_OK : ESP_FAIL;
}

// Growable in-memory sink for jpge (mirrors esp32-camera's to_jpg.cpp one).
namespace {
class MemStream : public jpge::output_stream {
public:
  uint8_t *buf = nullptr;
  size_t   len = 0, cap = 0;
  ~MemStream() override { free(buf); }
  bool put_buf(const void *p, int n) override {
    if (len + (size_t)n > cap) {
      size_t ncap = cap ? cap * 2 : 8192;
      while (ncap < len + (size_t)n) ncap *= 2;
      uint8_t *nb = (uint8_t *)realloc(buf, ncap);
      if (!nb) return false;
      buf = nb; cap = ncap;
    }
    memcpy(buf + len, p, n);
    len += n;
    return true;
  }
  jpge::uint get_size() const override { return (jpge::uint)len; }
  uint8_t *release() { uint8_t *b = buf; buf = nullptr; cap = len = 0; return b; }
};
}  // namespace

extern "C" bool fmt2jpg(uint8_t *src, size_t src_len, uint16_t width, uint16_t height,
                        pixformat_t format, uint8_t quality, uint8_t **out, size_t *out_len) {
  if (!src || !out || !out_len || format != PIXFORMAT_RGB888) return false;
  if (src_len < (size_t)width * height * 3) return false;

  MemStream stream;
  jpge::params p;
  p.m_quality = quality;
  p.m_subsampling = jpge::H2V2;
  jpge::jpeg_encoder enc;
  if (!enc.init(&stream, width, height, 3, p)) return false;
  for (uint16_t y = 0; y < height; y++) {
    if (!enc.process_scanline(src + (size_t)y * width * 3)) { enc.deinit(); return false; }
  }
  if (!enc.process_scanline(NULL)) { enc.deinit(); return false; }
  enc.deinit();

  *out_len = stream.len;
  *out = stream.release();
  return *out != NULL;
}
