# PlatformIO pre-build codegen.
#
# Turns the real source files under web/ into one generated PROGMEM blob pair
# (src/web_assets.gen.{h,cpp}). web/ is the single source of truth for the
# firmware's served UI; this keeps the HTML/JS/CSS out of the logic .cpp files
# while still baking the assets into the binary exactly as before - NO LittleFS,
# no partition change, no change to the OTA/flash workflow.
#
# Each web/<name.ext> becomes:
#   extern const char WEB_<NAME_EXT>[];   // e.g. web/ui.js -> WEB_UI_JS
# stored as a C++ raw string, so the compiled bytes are identical to the old
# hand-written R"...(...)..." literal. Served via server.send_P() as today.
#
# Wired in platformio.ini as `extra_scripts = pre:tools/gen_web_assets.py` on the
# ESP32 envs. The generated files are build artifacts (gitignored); this runs on
# every build and only rewrites them when the content actually changed, so it
# adds no rebuild churn.

import os

# Resolve the project root. Under PlatformIO the script is exec()'d by SCons with
# no __file__ defined, but an `env` is exported; standalone (manual run) it isn't.
try:
    Import("env")                       # noqa: F821  (SCons-injected)
    ROOT = env["PROJECT_DIR"]           # noqa: F821
except Exception:
    ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB = os.path.join(ROOT, "web")
GEN_H = os.path.join(ROOT, "src", "web_assets.gen.h")
GEN_C = os.path.join(ROOT, "src", "web_assets.gen.cpp")
DELIM = "WEBRAW"                       # raw-string delimiter; asserted collision-free
TERM = ")" + DELIM + '"'              # the only byte sequence that could close early


def sym(fn):
    return "WEB_" + fn.upper().replace(".", "_").replace("-", "_")


def _write_if_changed(path, text):
    old = None
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as fp:
            old = fp.read()
    if old != text:
        with open(path, "w", encoding="utf-8") as fp:
            fp.write(text)
        print("gen_web_assets: wrote %s" % os.path.relpath(path, ROOT))


def generate():
    files = []
    if os.path.isdir(WEB):
        files = sorted(f for f in os.listdir(WEB)
                       if not f.startswith(".") and
                       os.path.isfile(os.path.join(WEB, f)))
    banner = "// GENERATED from web/ by tools/gen_web_assets.py - DO NOT EDIT.\n"
    h = [banner, "#pragma once\n", "#include <Arduino.h>\n", "\n"]
    # branding.h supplies DEVICE_NAME, which pages splice in via the %DEVICE_NAME%
    # token below (expanded to the same compile-time concatenation as the old
    # hand-written literals, so the stored bytes are unchanged).
    c = [banner, '#include "web_assets.gen.h"\n', '#include "branding.h"\n', "\n"]
    # Template tokens expand back into the exact compile-time construction the old
    # hand-written literals used, so the stored bytes are unchanged:
    #   %DEVICE_NAME%  -> ...)WEBRAW" DEVICE_NAME R"WEBRAW(...   (macro splice)
    #   %IF_XIAO% / %ENDIF%  -> break the raw string around an
    #      #ifdef CAMERA_MODEL_XIAO_ESP32S3 ... #endif so board-conditional page
    #      sections are compiled only into the XIAO build (as the docs page did).
    token_expansions = {
        "%DEVICE_NAME%": ')%s" DEVICE_NAME R"%s(' % (DELIM, DELIM),
        "%IF_XIAO%": ')%s"\n#ifdef CAMERA_MODEL_XIAO_ESP32S3\nR"%s(' % (DELIM, DELIM),
        "%ENDIF%": ')%s"\n#endif\nR"%s(' % (DELIM, DELIM),
    }
    for fn in files:
        with open(os.path.join(WEB, fn), "rb") as fp:
            data = fp.read()
        text = data.decode("utf-8")          # web assets are UTF-8 source
        if TERM in text:
            raise SystemExit(
                "gen_web_assets: raw-string delimiter %r collides in web/%s "
                "- change DELIM in tools/gen_web_assets.py" % (DELIM, fn))
        body = text
        for tok, expansion in token_expansions.items():
            body = body.replace(tok, expansion)
        s = sym(fn)
        h.append("extern const char %s[];\n" % s)
        h.append("extern const unsigned %s_LEN;\n" % s)
        c.append('const char %s[] PROGMEM = R"%s(%s)%s";\n' % (s, DELIM, body, DELIM))
        c.append("const unsigned %s_LEN = sizeof(%s) - 1;\n\n" % (s, s))
    _write_if_changed(GEN_H, "".join(h))
    _write_if_changed(GEN_C, "".join(c))


generate()
