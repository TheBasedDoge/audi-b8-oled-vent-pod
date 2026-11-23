#!/bin/sh
# Rebuild the browser preview's renderer from the firmware's gauge_render.h.
# Compiles tools/preview/glue.cpp to WebAssembly and embeds it (base64) into
# docs/screen-mockup.html between the WASM-BEGIN / WASM-END markers.
# Needs clang with the wasm32 target and wasm-ld (LLVM 14+).
# Always built with the repository's own fonts and badge (-DGAUGE_PUBLIC), never private/ ones,
# since docs/screen-mockup.html is published.
set -e
cd "$(dirname "$0")/.."
OUT=$(mktemp -d)
clang++ --target=wasm32 -DGAUGE_PUBLIC -O2 -std=c++17 -nostdlib -ffreestanding -fno-exceptions -fno-rtti \
  -Wall -Wextra -Wl,--no-entry -Wl,--initial-memory=16777216 \
  -Wl,--export=g_fb,--export=g_status,--export=g_toast,--export=g_cfg,--export=g_set \
  -Wl,--export=g_render,--export=g_blank,--export=g_splash,--export=g_sys_tap,--export=g_sys_order,--export=g_theme,--export=g_tick,--export=g_demo,--export=g_sweep,--export=g_extra,--export=g_alert \
  -Wl,--export=__wasm_call_ctors \
  tools/preview/glue.cpp -o "$OUT/gauge.wasm"
python3 - "$OUT/gauge.wasm" docs/screen-mockup.html <<'PY'
import sys, base64, re
wasm = base64.b64encode(open(sys.argv[1], 'rb').read()).decode()
p = sys.argv[2]; s = open(p).read()
s = re.sub(r'(/\*WASM-BEGIN\*/").*?("/\*WASM-END\*/)', lambda m: m.group(1) + wasm + m.group(2), s, flags=re.S)
open(p, 'w').write(s)
print("embedded %d bytes of wasm into %s" % (len(wasm) * 3 // 4, p))
PY
rm -rf "$OUT"
