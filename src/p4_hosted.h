// p4_hosted.h — ESP32-P4 co-processor (ESP32-C6 / esp-hosted) maintenance.
//
// The P4 has no radio: WiFi runs on the onboard C6 over esp-hosted/SDIO. These
// endpoints let the C6's slave firmware be inspected and updated from the LAN
// (the shipped slave predates the arduino core's host library, so version RPCs
// time out and the boot log nags to upgrade).
//
//   GET /c6/version           JSON: host + slave esp-hosted versions
//   GET /c6/update?url=<bin>  fetch the slave bin over plain HTTP (serve it from
//                             a LAN machine) and flash it to the C6; on success
//                             the device restarts to re-init the transport.
//                             Write-auth required. Fails safe: any download or
//                             flash error keeps the current (working) slave.
//
// P4-only (compiled out elsewhere): the S3/CAM boards have no co-processor.
#pragma once
#if defined(CAMERA_MODEL_ESP32P4)

#include <WebServer.h>

void p4HostedRegisterEndpoints(WebServer &server);

#endif // CAMERA_MODEL_ESP32P4
