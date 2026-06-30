// clip_index.h
//
// In-RAM index of the /clips directory so the hot paths NEVER scan the card.
// A full FAT directory scan holds the single SD mutex and can take many seconds
// on a slow/worn card, which starved every other SD op (the field "SD busy" /
// slow-list / failed-download cascade). Four paths used to scan per poll or per
// write: the audio + video LIST endpoints and the audio + video cap evictions.
// They now read this index (its own short mutex, never the SD bus) instead.
//
// The index is populated by the one scan we keep (at mount/remount) and updated
// incrementally on every write / delete / clear. It holds only the clip files
// the lists + caps care about: clip_*.wav (audio) and vclip_*.avi (video).
// Sidecars (.mot), continuous segments (cont_*) and history.bin are not indexed.

#pragma once

// Pure in-RAM clip index (no hardware): available on every board so the video
// recorder/list compiles even where SD isn't present.

#include <Arduino.h>
#include <stdint.h>

enum ClipKind { CLIP_AUDIO = 0, CLIP_VIDEO = 1 };

// Clip file numbers wrap at this value (clip_00000 .. clip_09999), keeping names
// to 5 tidy digits forever instead of an ever-growing counter. The caps keep far
// fewer than this on disk, so the live set is always a small contiguous run in the
// 0..CLIP_NUM_WRAP-1 ring; oldest/next are computed wrap-aware (largest-gap), so
// eviction stays correct across the wrap. (Practically the wrap is ~a century out
// at any sane capture rate, but doing it right costs nothing and is future-proof.)
#define CLIP_NUM_WRAP 10000u

// Allocate the index (PSRAM) and its mutex. Call once at boot before any add.
void clipIndexInit();

// Drop all entries of a kind (CLIP_AUDIO/CLIP_VIDEO), or every entry when
// kind < 0. Used by mount-scan repopulation and the clear-all endpoints.
void clipIndexClear(int kind);

// Add/refresh an entry. `name` is the bare filename (e.g. "clip_00042_a.wav");
// the numeric index is parsed from it for oldest-first eviction. A name already
// present is updated in place (idempotent re-scan).
void clipIndexAdd(int kind, const char *name, uint32_t size, uint32_t mtime);

// Remove an entry by exact filename (any kind). No-op if absent.
void clipIndexRemove(const char *name);

// Live count of indexed files of a kind.
uint32_t clipIndexCount(int kind);

// Copy the name of the OLDEST entry of a kind into out[cap]. Returns false if
// none. Wrap-aware (largest-gap in the circular numbering), so the cap eviction
// keeps deleting the true oldest even after the file number wraps past 0.
bool clipIndexOldest(int kind, char *out, size_t cap);

// The next file number to assign for a kind: (newest existing number + 1) mod
// CLIP_NUM_WRAP, or 0 if none. Wrap-aware; used to seed the writer's counter at
// boot/remount so it resumes correctly even when the numbering has wrapped.
uint32_t clipIndexNextNum(int kind);

// Append the [{"name":..,"size":..,"mtime":..}, ...] JSON for a kind (up to maxN
// entries) to `out` - the exact shape the /sd/list and /video/list endpoints
// returned from their scans, now served from RAM with no SD access.
void clipIndexListJson(int kind, uint32_t maxN, String &out);
