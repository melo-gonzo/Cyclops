// path_safe.h
//
// Pure, hardware-free validation of client-supplied clip file names. No Arduino
// includes -> unit-tested on the host (test/test_path_safe). This is the path-
// traversal guard for /sd/file and /sd/delete, so it is security-relevant and
// belongs in one tested place. audio_capture.cpp is the shell (wraps Arduino
// String -> const char*).

#pragma once

#include <stddef.h>
#include <string.h>

namespace pathsafe {

inline bool endsWith(const char *s, const char *suffix) {
  size_t ls = strlen(s), lf = strlen(suffix);
  return ls >= lf && strcmp(s + ls - lf, suffix) == 0;
}

// Accept ONLY a bare clip_*.wav / clip_*.jpg, or vclip_*.avi / vclip_*.mot /
// vclip_*.jpg sidecar. The .jpg forms are the per-clip key-frame thumbnails
// (audio clips carry the camera frame at trigger; video clips the trigger frame).
// Rejects anything with a path separator or ".." (traversal), and bounds length.
inline bool validClipName(const char *name) {
  if (!name) return false;
  size_t n = strlen(name);
  if (n < 5 || n > 32) return false;
  if (strchr(name, '/')) return false;
  if (strstr(name, "..")) return false;
  bool clip = strncmp(name, "clip_", 5) == 0 &&
              (endsWith(name, ".wav") || endsWith(name, ".jpg"));
  bool vclip = strncmp(name, "vclip_", 6) == 0 &&
               (endsWith(name, ".avi") || endsWith(name, ".mot") ||
                endsWith(name, ".jpg"));
  return clip || vclip;
}

} // namespace pathsafe
