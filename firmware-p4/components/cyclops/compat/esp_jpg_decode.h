// P4 compat header — same API as esp32-camera's esp_jpg_decode.h (the fleet's
// arduino-2.x framework bundles it; nothing provides it on the P4). The shared
// video_record.cpp motion/thumbnail path uses this exact callback protocol:
// reader streams the source JPEG, writer receives RGB888 blocks, with a
// data==NULL call announcing the scaled dims first and a final data==NULL end
// marker. Implemented in jpg_compat.cpp over the esp_jpeg (tjpgd) component.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    JPG_SCALE_NONE,
    JPG_SCALE_2X,
    JPG_SCALE_4X,
    JPG_SCALE_8X,
    JPG_SCALE_MAX = JPG_SCALE_8X
} jpg_scale_t;

typedef size_t (*jpg_reader_cb)(void *arg, size_t index, uint8_t *buf, size_t len);
typedef bool (*jpg_writer_cb)(void *arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data);

esp_err_t esp_jpg_decode(size_t len, jpg_scale_t scale, jpg_reader_cb reader, jpg_writer_cb writer, void *arg);

#ifdef __cplusplus
}
#endif
