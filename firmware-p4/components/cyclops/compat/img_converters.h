// P4 compat header — the slice of esp32-camera's img_converters.h/sensor.h the
// shared Cyclops code uses (fmt2jpg for clip thumbnails + the pixformat enum).
// esp32-camera registers nothing on the esp32p4 target, so this stands in.
// fmt2jpg is implemented in jpg_compat.cpp over the vendored jpge encoder.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Matches esp32-camera's sensor.h ordering (only RGB888 is used on the P4).
typedef enum {
    PIXFORMAT_RGB565 = 0,
    PIXFORMAT_YUV422,
    PIXFORMAT_YUV420,
    PIXFORMAT_GRAYSCALE,
    PIXFORMAT_JPEG,
    PIXFORMAT_RGB888,
    PIXFORMAT_RAW,
    PIXFORMAT_RGB444,
    PIXFORMAT_RGB555,
} pixformat_t;

// Encode a raw frame to JPEG. Output buffer is malloc'd (PSRAM-preferred) and
// owned by the caller (free()). Only PIXFORMAT_RGB888 input is supported here.
bool fmt2jpg(uint8_t *src, size_t src_len, uint16_t width, uint16_t height,
             pixformat_t format, uint8_t quality, uint8_t **out, size_t *out_len);

#ifdef __cplusplus
}
#endif
