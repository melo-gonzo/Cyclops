# Force the GENERATED esp_ipa sensor-tuning config to win the link.
#
# esp_ipa ships a prebuilt libesp_ipa.a that bundles a default (empty)
# esp_video_ipa_config.c.obj: esp_ipa_pipeline_get_config() returning NULL. The
# real config is GENERATED at build time from the sensor's JSON (OV5647 AE/AWB
# tuning) and archived into the SCons component lib — but PlatformIO's link
# line scans the prebuilt archive first, so the empty one wins and esp_video
# boots with "failed to get configuration to initialize ISP controller" (no
# auto-exposure → black frames). Native IDF links the component lib first.
#
# Passing the generated object directly on the link line makes it
# unconditionally present before any archive is scanned, so both archive
# copies are skipped. (PIOBUILDFILES is consumed before post scripts run, so
# LINKFLAGS is the reliable injection point.)
Import("env")

obj = env.File("$BUILD_DIR/esp-idf/espressif__esp_ipa/esp_video_ipa_config.c.o")
env.Append(LINKFLAGS=[obj.get_abspath()])
env.Depends("$BUILD_DIR/${PROGNAME}$PROGSUFFIX", obj)
