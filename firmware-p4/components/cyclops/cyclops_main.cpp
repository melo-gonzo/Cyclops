// Compiles the shared Cyclops src/main.cpp under a unique object basename, to
// avoid the main.cpp.o clash with arduino-esp32's cores/esp32/main.cpp in
// PlatformIO's flattened build. src/main.cpp stays the single source of truth;
// its headers resolve via the component INCLUDE_DIRS (the shared src/).
#include "main.cpp"
