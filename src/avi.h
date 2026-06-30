// avi.h
//
// Pure, hardware-free MJPEG-AVI byte framing. No Arduino includes -> unit-tested
// on the host (test/test_avi). The fixed-offset RIFF/AVI header is unforgiving
// (one wrong offset = a file VLC won't open), so the layout lives in one tested
// place. video_record.cpp is the shell that writes these buffers to SD and
// patches the size/count fields at close.
//
// Layout: RIFF @0, LIST hdrl @12 (avih @24, LIST strl @88: strh @100, strf
// @164), LIST movi @212, frames from @224, idx1 appended at end.

#pragma once

#include <stdint.h>
#include <string.h>

namespace avi {

static const uint32_t HEADER_BYTES = 224;
static const uint32_t MOVI_FOURCC_POS = 220; // frame offsets are relative to here
static const uint32_t IDX_KEYFRAME = 0x10;   // AVIIF_KEYFRAME

// Little-endian writers (AVI is LE; explicit so host tests == device output).
inline void w32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
inline void w16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}

// Fill the fixed 224-byte header stub. dwTotalFrames (@48), dwLength (@140),
// RIFF size (@4) and movi size (@216) are left zero and patched at close.
inline void buildHeader(uint8_t *h, uint16_t w, uint16_t ht, uint16_t fps) {
  uint32_t f = fps ? fps : 1; // guard div-by-zero (fps clamped upstream)
  memset(h, 0, HEADER_BYTES);
  memcpy(h, "RIFF", 4);
  memcpy(h + 8, "AVI ", 4);
  memcpy(h + 12, "LIST", 4);
  w32(h + 16, 192);
  memcpy(h + 20, "hdrl", 4);
  memcpy(h + 24, "avih", 4);
  w32(h + 28, 56);
  w32(h + 32, 1000000UL / f);     // dwMicroSecPerFrame
  w32(h + 36, 512UL * 1024);      // dwMaxBytesPerSec
  w32(h + 44, 0x10);              // AVIF_HASINDEX
  w32(h + 56, 1);                 // dwStreams
  w32(h + 60, 256UL * 1024);      // dwSuggestedBufferSize
  w32(h + 64, w);
  w32(h + 68, ht);
  memcpy(h + 88, "LIST", 4);
  w32(h + 92, 116);
  memcpy(h + 96, "strl", 4);
  memcpy(h + 100, "strh", 4);
  w32(h + 104, 56);
  memcpy(h + 108, "vids", 4);
  memcpy(h + 112, "MJPG", 4);
  w32(h + 128, 1);               // dwScale
  w32(h + 132, f);              // dwRate
  w32(h + 144, 256UL * 1024);   // dwSuggestedBufferSize
  w32(h + 148, 0xFFFFFFFFu);    // dwQuality
  w16(h + 160, w);              // rcFrame
  w16(h + 162, ht);
  memcpy(h + 164, "strf", 4);
  w32(h + 168, 40);
  w32(h + 172, 40);             // biSize
  w32(h + 176, w);
  w32(h + 180, ht);
  w16(h + 184, 1);             // planes
  w16(h + 186, 24);           // bits
  memcpy(h + 188, "MJPG", 4);
  w32(h + 192, (uint32_t)w * ht * 3); // biSizeImage
  memcpy(h + 212, "LIST", 4);
  memcpy(h + 220, "movi", 4);
}

// 8-byte '00dc' chunk header preceding a JPEG frame of len bytes.
inline void frameChunkHeader(uint8_t *hdr, uint32_t len) {
  memcpy(hdr, "00dc", 4);
  w32(hdr + 4, len);
}

// 16-byte idx1 entry: '00dc', flags, offset (relative to movi fourcc), length.
inline void indexEntry(uint8_t *e, uint32_t off, uint32_t len) {
  memcpy(e, "00dc", 4);
  w32(e + 4, IDX_KEYFRAME);
  w32(e + 8, off);
  w32(e + 12, len);
}

// 8-byte 'idx1' chunk header preceding the frames*16-byte index, appended at the
// movi end (after the last good frame).
inline void idx1Header(uint8_t *hdr, uint32_t frames) {
  memcpy(hdr, "idx1", 4);
  w32(hdr + 4, frames * 16);
}

// Field offsets in the header that are zero-stubbed by buildHeader and patched at
// close once the frame count and final extents are known.
static const uint32_t RIFF_SIZE_POS    = 4;   // dwRIFFsize
static const uint32_t TOTAL_FRAMES_POS = 48;  // avih dwTotalFrames
static const uint32_t STRH_LENGTH_POS  = 140; // strh dwLength (frame count)
static const uint32_t MOVI_SIZE_POS    = 216; // movi LIST size

struct Patch { uint32_t off, val; };

// The four little-endian size/count values written into the header at close.
// fileEnd is the total file size; moviEnd is the position just past the last
// FULLY-written frame (where idx1 begins). Pure so the offsets+derivation are
// host-tested - one wrong value yields a file a compliant player won't open.
inline void finalizePatches(uint32_t frames, uint32_t moviEnd, uint32_t fileEnd,
                            Patch out[4]) {
  out[0] = {RIFF_SIZE_POS,    fileEnd - 8};
  out[1] = {TOTAL_FRAMES_POS, frames};
  out[2] = {STRH_LENGTH_POS,  frames};
  out[3] = {MOVI_SIZE_POS,    moviEnd - MOVI_FOURCC_POS};
}

} // namespace avi
