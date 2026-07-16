# Thin main component. The actual Cyclops application — the SHARED fleet src/
# tree, including setup()/loop() — is built as the 'cyclops' component (see
# components/cyclops/CMakeLists.txt). Keeping the shared tree in its own IDF
# component avoids PlatformIO's SCons double-compiling the app sources (which
# clashed on the main.cpp.o target). This TU just keeps the main component
# non-empty and pulls in cyclops.
idf_component_register(SRCS "p4_main.c" INCLUDE_DIRS "." REQUIRES cyclops)
