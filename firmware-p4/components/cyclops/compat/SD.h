// P4 compat header — shadows arduino's SPI <SD.h> for the P4 build only.
//
// The shared code (video_record, cont_record, ota_update) says `#include <SD.h>`
// and talks to the global `SD` object because the fleet's cards sit on SPI. The
// NANO's microSD is on the P4's SDMMC SLOT-0 IOMUX pins (CLK43/CMD44/D39-42,
// LDO channel 4, power pin 45 — all baked into the arduino esp32p4 variant), so
// map the same name onto the SD_MMC driver. Both are fs::FS: open/remove/mkdir/
// usedBytes/totalBytes behave identically. Mounting is done by p4_sd.cpp (via
// SD_MMC.begin), never through this alias.
#pragma once

#include <SD_MMC.h>

static fs::SDMMCFS &SD = SD_MMC;
