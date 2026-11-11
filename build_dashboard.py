"""Copies dashboard/index.html into firmware/ACL_Guard/dashboard.h.
Run this again after you edit the dashboard:  python build_dashboard.py"""
from pathlib import Path
root = Path(__file__).parent
html = (root / "dashboard" / "index.html").read_text(encoding="utf-8")
assert ')rawliteral"' not in html
out = ("// AUTO-GENERATED from dashboard/index.html by build_dashboard.py - edit the HTML, not this file.\n"
       "#pragma once\n#include <Arduino.h>\n"
       'const char INDEX_HTML[] PROGMEM = R"rawliteral(' + html + ')rawliteral";\n')
(root / "firmware" / "ACL_Guard" / "dashboard.h").write_text(out, encoding="utf-8")
print("dashboard.h written,", len(html), "bytes of HTML")
