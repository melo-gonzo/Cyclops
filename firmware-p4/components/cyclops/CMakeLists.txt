# The Cyclops application = the SHARED fleet src/ tree, built for the P4.
# "Reuse everything from the S3; swap only the camera": OV2640.cpp (the DVP
# esp_camera backend) is excluded — camera_p4.cpp provides the MIPI/esp_video
# backend. board_stubs.cpp (compiled on every non-XIAO board) supplies the
# no-audio/no-SD stubs, exactly as on the ESP32-CAM.
#
# WHOLE_ARCHIVE so arduino's autostart finds setup()/loop() (and the HTTP/OTA
# handlers referenced only indirectly) in this static library.
set(CYCLOPS_SRC "${CMAKE_CURRENT_LIST_DIR}/../../../src")
file(GLOB CYCLOPS_SRCS "${CYCLOPS_SRC}/*.cpp")
list(FILTER CYCLOPS_SRCS EXCLUDE REGEX "/OV2640\\.cpp$")
# src/main.cpp shares a basename with arduino-esp32's cores/esp32/main.cpp, which
# collides on PlatformIO's flattened main.cpp.o target. Compile it under a unique
# name via cyclops_main.cpp (a one-line #include) — no fleet rename needed.
list(FILTER CYCLOPS_SRCS EXCLUDE REGEX "/main\\.cpp$")
# Phase-0/1 bring-up scaffold (root [env:esp32p4]) — has its own setup()/loop().
list(FILTER CYCLOPS_SRCS EXCLUDE REGEX "/bringup_p4\\.cpp$")

idf_component_register(
    SRCS ${CYCLOPS_SRCS} "cyclops_main.cpp"
         "compat/jpg_compat.cpp" "compat/jpge.cpp"
    INCLUDE_DIRS "${CYCLOPS_SRC}" "compat"
    # framework-arduinoespressif32 is the arduino component's real name in the
    # pioarduino combined build ("arduino" is not a resolvable REQUIRES name).
    # No WHOLE_ARCHIVE: (a) PlatformIO's SCons wrapper mishandles it ("Two
    # environments ... main.cpp.o"), and (b) it isn't needed — arduino's core
    # main.cpp references setup()/loop(), so the linker pulls our objects.
    # espressif__esp_jpeg backs the compat esp_jpg_decode (same tjpgd family as
    # the fleet's S3 decode); compat/ provides the esp32-camera conversion API
    # the shared motion/thumbnail code expects (esp32-camera itself registers
    # nothing on the esp32p4 target).
    REQUIRES framework-arduinoespressif32
             esp_video esp_driver_jpeg esp_http_server nvs_flash
             esp_wifi esp_timer driver esp_psram
             espressif__esp_jpeg espressif__esp_codec_dev
)

# The board-select define, baked in at the CMake level: PlatformIO's build_flags
# only reliably reach the PROJECT component in combined builds (observed: the
# -D vanished from component compiles after a full reconfigure). PUBLIC so any
# dependent sees the same board.
target_compile_definitions(${COMPONENT_LIB} PUBLIC CAMERA_MODEL_ESP32P4)
