// Host-native unit tests for the pure MJPEG-AVI framing. pio test -e native

#include <unity.h>
#include <string.h>
#include "avi.h"

static uint8_t h[avi::HEADER_BYTES];

static uint32_t rd32(const uint8_t *p) {
  return (uint32_t)p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}
static bool tag(const uint8_t *p, const char *t) { return memcmp(p, t, 4) == 0; }

void test_header_is_224_bytes(void) {
  TEST_ASSERT_EQUAL_UINT32(224, avi::HEADER_BYTES);
}

void test_header_fourccs_at_offsets(void) {
  avi::buildHeader(h, 1024, 768, 5);
  TEST_ASSERT_TRUE(tag(h + 0, "RIFF"));
  TEST_ASSERT_TRUE(tag(h + 8, "AVI "));
  TEST_ASSERT_TRUE(tag(h + 12, "LIST"));
  TEST_ASSERT_TRUE(tag(h + 20, "hdrl"));
  TEST_ASSERT_TRUE(tag(h + 24, "avih"));
  TEST_ASSERT_TRUE(tag(h + 88, "LIST"));
  TEST_ASSERT_TRUE(tag(h + 96, "strl"));
  TEST_ASSERT_TRUE(tag(h + 100, "strh"));
  TEST_ASSERT_TRUE(tag(h + 108, "vids"));
  TEST_ASSERT_TRUE(tag(h + 112, "MJPG"));
  TEST_ASSERT_TRUE(tag(h + 164, "strf"));
  TEST_ASSERT_TRUE(tag(h + 188, "MJPG"));
  TEST_ASSERT_TRUE(tag(h + 212, "LIST"));
  TEST_ASSERT_TRUE(tag(h + 220, "movi"));
}

void test_header_dimensions_and_fps(void) {
  avi::buildHeader(h, 1024, 768, 5);
  TEST_ASSERT_EQUAL_UINT32(200000, rd32(h + 32)); // 1e6/5 us per frame
  TEST_ASSERT_EQUAL_UINT32(1024, rd32(h + 64));   // avih width
  TEST_ASSERT_EQUAL_UINT32(768, rd32(h + 68));    // avih height
  TEST_ASSERT_EQUAL_UINT32(5, rd32(h + 132));     // dwRate (fps)
  TEST_ASSERT_EQUAL_UINT32(1024, rd32(h + 176));  // biWidth
  TEST_ASSERT_EQUAL_UINT32(768, rd32(h + 180));   // biHeight
  TEST_ASSERT_EQUAL_UINT32(1024u * 768 * 3, rd32(h + 192)); // biSizeImage
}

void test_header_patched_fields_start_zero(void) {
  avi::buildHeader(h, 800, 600, 10);
  TEST_ASSERT_EQUAL_UINT32(0, rd32(h + 4));   // RIFF size (patched at close)
  TEST_ASSERT_EQUAL_UINT32(0, rd32(h + 48));  // dwTotalFrames
  TEST_ASSERT_EQUAL_UINT32(0, rd32(h + 140)); // dwLength
  TEST_ASSERT_EQUAL_UINT32(0, rd32(h + 216)); // movi size
}

void test_header_fps_zero_guarded(void) {
  avi::buildHeader(h, 640, 480, 0); // must not divide by zero
  TEST_ASSERT_EQUAL_UINT32(1000000, rd32(h + 32)); // treated as 1 fps
}

void test_frame_chunk_header(void) {
  uint8_t c[8];
  avi::frameChunkHeader(c, 0x12345);
  TEST_ASSERT_TRUE(tag(c, "00dc"));
  TEST_ASSERT_EQUAL_UINT32(0x12345, rd32(c + 4));
}

void test_index_entry(void) {
  uint8_t e[16];
  avi::indexEntry(e, 0xAABB, 0x100);
  TEST_ASSERT_TRUE(tag(e, "00dc"));
  TEST_ASSERT_EQUAL_UINT32(avi::IDX_KEYFRAME, rd32(e + 4));
  TEST_ASSERT_EQUAL_UINT32(0xAABB, rd32(e + 8));  // offset
  TEST_ASSERT_EQUAL_UINT32(0x100, rd32(e + 12));  // length
}

void test_little_endian_writers(void) {
  uint8_t b[4];
  avi::w32(b, 0x04030201);
  TEST_ASSERT_EQUAL_UINT8(0x01, b[0]);
  TEST_ASSERT_EQUAL_UINT8(0x02, b[1]);
  TEST_ASSERT_EQUAL_UINT8(0x03, b[2]);
  TEST_ASSERT_EQUAL_UINT8(0x04, b[3]);
}

void test_idx1_header(void) {
  uint8_t hdr[8];
  avi::idx1Header(hdr, 5);
  TEST_ASSERT_TRUE(tag(hdr, "idx1"));
  TEST_ASSERT_EQUAL_UINT32(80, rd32(hdr + 4)); // 5 entries * 16 bytes
}

void test_finalize_patch_offsets_and_values(void) {
  avi::Patch p[4];
  avi::finalizePatches(/*frames*/ 7, /*moviEnd*/ 1000, /*fileEnd*/ 1234, p);
  TEST_ASSERT_EQUAL_UINT32(4, p[0].off);   TEST_ASSERT_EQUAL_UINT32(1234 - 8, p[0].val);
  TEST_ASSERT_EQUAL_UINT32(48, p[1].off);  TEST_ASSERT_EQUAL_UINT32(7, p[1].val);
  TEST_ASSERT_EQUAL_UINT32(140, p[2].off); TEST_ASSERT_EQUAL_UINT32(7, p[2].val);
  TEST_ASSERT_EQUAL_UINT32(216, p[3].off);
  TEST_ASSERT_EQUAL_UINT32(1000 - avi::MOVI_FOURCC_POS, p[3].val);
}

// Assemble a whole clip the way video_record.cpp does (header, 3 frames with a
// word-align pad on the odd-length one, idx1, then the close-time patches) and
// verify the result is internally consistent - the "VLC won't open it" guard.
void test_full_clip_assembly_roundtrip(void) {
  static uint8_t buf[1024];
  avi::buildHeader(buf, 320, 240, 5);
  uint32_t pos = avi::HEADER_BYTES;
  const uint32_t lens[3] = {10, 7, 20}; // 7 is odd -> exercises the pad byte
  uint32_t offs[3];
  for (int i = 0; i < 3; i++) {
    offs[i] = pos - avi::MOVI_FOURCC_POS; // frame offset is relative to 'movi' fourcc
    uint8_t ch[8];
    avi::frameChunkHeader(ch, lens[i]);
    memcpy(buf + pos, ch, 8); pos += 8;
    memset(buf + pos, 0xAB, lens[i]); pos += lens[i];
    if (lens[i] & 1) buf[pos++] = 0; // RIFF chunks are word-aligned
  }
  uint32_t moviEnd = pos;
  uint8_t ih[8];
  avi::idx1Header(ih, 3);
  memcpy(buf + pos, ih, 8); pos += 8;
  for (int i = 0; i < 3; i++) {
    uint8_t e[16];
    avi::indexEntry(e, offs[i], lens[i]);
    memcpy(buf + pos, e, 16); pos += 16;
  }
  uint32_t fileEnd = pos;
  avi::Patch p[4];
  avi::finalizePatches(3, moviEnd, fileEnd, p);
  for (int i = 0; i < 4; i++) avi::w32(buf + p[i].off, p[i].val);

  // Header size/count fields now reflect the assembled clip.
  TEST_ASSERT_TRUE(tag(buf, "RIFF"));
  TEST_ASSERT_EQUAL_UINT32(fileEnd - 8, rd32(buf + 4));      // RIFF size
  TEST_ASSERT_EQUAL_UINT32(3, rd32(buf + 48));               // dwTotalFrames
  TEST_ASSERT_EQUAL_UINT32(3, rd32(buf + 140));              // strh dwLength
  TEST_ASSERT_EQUAL_UINT32(moviEnd - avi::MOVI_FOURCC_POS, rd32(buf + 216)); // movi size
  // First frame chunk sits right after the 224-byte header; offset relative to
  // the 'movi' fourcc @220 is therefore 4.
  TEST_ASSERT_TRUE(tag(buf + avi::HEADER_BYTES, "00dc"));
  TEST_ASSERT_EQUAL_UINT32(4, offs[0]);
  // idx1 begins at moviEnd and the first index entry points back at frame 0.
  TEST_ASSERT_TRUE(tag(buf + moviEnd, "idx1"));
  TEST_ASSERT_EQUAL_UINT32(48, rd32(buf + moviEnd + 4));     // 3 * 16
  TEST_ASSERT_TRUE(tag(buf + moviEnd + 8, "00dc"));          // entry 0 tag
  TEST_ASSERT_EQUAL_UINT32(offs[0], rd32(buf + moviEnd + 8 + 8));  // entry 0 offset
  TEST_ASSERT_EQUAL_UINT32(lens[0], rd32(buf + moviEnd + 8 + 12)); // entry 0 length
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_header_is_224_bytes);
  RUN_TEST(test_header_fourccs_at_offsets);
  RUN_TEST(test_header_dimensions_and_fps);
  RUN_TEST(test_header_patched_fields_start_zero);
  RUN_TEST(test_header_fps_zero_guarded);
  RUN_TEST(test_frame_chunk_header);
  RUN_TEST(test_index_entry);
  RUN_TEST(test_little_endian_writers);
  RUN_TEST(test_idx1_header);
  RUN_TEST(test_finalize_patch_offsets_and_values);
  RUN_TEST(test_full_clip_assembly_roundtrip);
  return UNITY_END();
}
