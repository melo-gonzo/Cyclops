#!/usr/bin/env python3
"""Serve the real Cyclops audio dashboard on localhost, driven by your laptop
microphone (via tools/live_mic.js + the Web Audio API) instead of the device.

  python3 tools/serve_live.py          # generate, serve on :8765, open browser
  python3 tools/serve_live.py --generate   # just write tools/ui_live.html

getUserMedia needs a secure context, so this serves over http://localhost
(file:// won't get mic access). Click "Start mic", grant permission, and the
live spectrum / heatmap / level plot / triggers all run off real sound.
"""
import re, sys, pathlib, http.server, socketserver, webbrowser, threading

ROOT = pathlib.Path(__file__).resolve().parent.parent
TOOLS = ROOT / "tools"
PORT = 8765

def build():
    src = (ROOT / "src" / "audio_capture.cpp").read_text()
    m = re.search(r'AUDIO_PAGE\[\]\s*PROGMEM\s*=\s*R"rawliteral\((.*?)\)rawliteral";', src, re.S)
    if not m:
        raise SystemExit("could not find AUDIO_PAGE literal")
    page = m.group(1).replace(')rawliteral" DEVICE_NAME R"rawliteral(', "Cyclops")
    if "rawliteral" in page:
        raise SystemExit("unhandled rawliteral splice remains")
    inject = "<script>\n" + (TOOLS / "live_mic.js").read_text() + "\n</script>\n"
    html = page.replace("</head>", inject + "</head>", 1)
    out = TOOLS / "ui_live.html"
    out.write_text(html)
    print(f"wrote {out} ({out.stat().st_size} bytes)")
    return out

def main():
    build()
    if "--generate" in sys.argv:
        return
    class H(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *a, **k): super().__init__(*a, directory=str(TOOLS), **k)
        def log_message(self, *a): pass
    url = f"http://localhost:{PORT}/ui_live.html"
    with socketserver.TCPServer(("127.0.0.1", PORT), H) as httpd:
        print(f"serving {url}  (Ctrl+C to stop)")
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
        try: httpd.serve_forever()
        except KeyboardInterrupt: print("\nstopped")

if __name__ == "__main__":
    main()
