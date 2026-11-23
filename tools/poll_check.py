#!/usr/bin/env python3
"""Checks the OBD poll list against the freshness windows, straight from the firmware source.

Every reading on the pages, in the warnings, the logs and the web page counts as "known" only while
it's younger than its window (AGE_* / ageFor() in obd_ble.h). If a change to the poll list makes a
reading arrive less often than its window allows, it flickers to "--" and the warnings see gaps.
That happened twice (log_0033-0038), so this simulates the scheduler with the real list, at cruise
and on boost, and fails if any window is less than 2x how often its reading actually arrives
(for a reading that waits during pulls: less than the 20 s wait plus its cruise interval).

run: python3 tools/poll_check.py        (exit code 1 if a window is too tight)
Timing from the car: an OBD request about 56 ms, an AT command about 35 ms (log_0038: ~19 answers/s).
"""
import re, sys, os

SRC = os.path.join(os.path.dirname(__file__), "..", "firmware", "S5GaugePod_Test", "obd_ble.h")
REQ, AT = 56, 35
PULL_PAUSE_MAX = 20000

src = open(SRC).read()

# ---- the poll list
body = src[src.index("static Slow slowList[] = {"):]
body = body[:body.index("\n};")]
items = []
for m in re.finditer(r'^\s*\{"(\w+)",\s*(0x[0-9A-Fa-f]+|0),\s*(\d+),\s*0,\s*0,\s*(\d+),\s*(\d+)(?:,\s*(\d+),\s*\{([^}]*)\})?', body, re.M):
    cmd, pid, per, kind, bms = m.group(1), int(m.group(2), 16), int(m.group(3)), int(m.group(4)), int(m.group(5))
    dids = [int(x, 16) for x in re.findall(r"0x([0-9A-Fa-f]+)", m.group(7) or "")]
    items.append(dict(cmd=cmd, pid=pid, per=per, kind=kind, bms=bms, did0=dids[0] if dids else 0))

# not polled on this car (alternatives / unsupported): see the conditions in pollLoop()
NOT_ON_S5 = {"010F": "0177 is there", "015C": "not on the S5", "010B": "0170 is there", "010D": "speed comes with RPM"}
ONCE_IN_A_WHILE = {"222BC1", "0621", "22121C"}       # clock, catalyst tests, odometer: no window to check

# which readings each request feeds (fields as in enum Field)
FEEDS = {
    "0105": ["F_CLT"], "0177": ["F_CAC"], "010E": ["F_TIM"], "ATRV": ["F_BAT"], "0170": ["F_MAP"],
    "01343844": ["F_AFR1", "F_AFR2", "F_AFRCMD"], "01463C3D": ["F_AMB", "F_CAT1", "F_CAT2"],
    "2210702982": ["F_PEDAL", "F_BYPASS"], "222029": ["F_MAPCMD"], "22130E130F1321": ["F_HPFP", "F_HPFPSET", "F_LPFP"],
    "2213A5": ["F_INJ"], "2220A1100311D6": ["F_OIL", "F_PRESC", "F_ICPUMP"], "2213F213F313F4": ["F_KNOCK"],
    "2213F513F613F7": ["F_KNOCK"], "0133": ["F_BARO"], "2210351026": ["F_GSPORT", "F_GPROG"],
    "221259125A125B": ["F_GBT1", "F_GBT2", "F_GBT3"], "221306": ["F_DIFT1", "F_DIFT2"],
    "2216CE16CF": ["F_BRAKE1", "F_BRAKE2"], "222B78": ["F_EPB"], "011519": ["F_O2B1S2", "F_O2B2S2"],
}

# as pullPaused() / stoppedOnly() in obd_ble.h
def paused(it): return it["kind"] in (1, 3, 4, 5, 6, 7) or (it["kind"] == 0 and it["pid"] in (0x05, 0x33, 0x15, 0x11))
def stopped_only(it): return it["kind"] == 8 or (it["kind"] == 2 and it["did0"] == 0x16CE)
def cost(it):
    switches = {3: 2, 5: 4, 8: 4, 6: 7}.get(it["kind"], 0)
    return REQ * (2 if it["cmd"] == "221306" else 1) + AT * switches

# ---- the windows
ages = {k: int(v) for k, v in re.findall(r"(AGE_\w+) = (\d+)", src)}
fn = src[src.index("static uint32_t ageFor(int f)"):]
fn = fn[:fn.index("\n}\n")]
window = {}
for cases, ret in re.findall(r"((?:case F_\w+:\s*)+)return (\w+);", fn):
    for f in re.findall(r"F_\w+", cases):
        window[f] = ages[ret] if ret in ages else int(ret)
default_age = ages[re.search(r"default: return (\w+);", fn).group(1)]

# ---- the scheduler (as pollLoop: RPM + speed every round, then the most overdue as a share of its period)
def simulate(boosting, T=900000):
    live = [it for it in items if it["cmd"] not in NOT_ON_S5 and not stopped_only(it)]
    last = {it["cmd"]: 0 for it in live}; prev = {}; got = {it["cmd"]: [] for it in live}
    t = 0
    while t < T:
        t += REQ
        best, worst = None, 1.0
        for it in live:
            if boosting and paused(it): continue
            per = it["bms"] if (boosting and it["bms"]) else it["per"]
            r = (t - last[it["cmd"]]) / per
            if r >= worst: worst, best = r, it
        if not best: continue
        visit = [best]
        if best["kind"] == 3:                          # the gearbox's other reading rides along when half due
            for o in live:
                if o is not best and o["kind"] == 3 and (t - last[o["cmd"]]) >= 0.5 * o["per"]: visit.append(o)
        for it in visit:
            last[it["cmd"]] = t
            t += cost(it) - (AT * 2 if it is not best and it["kind"] == 3 else 0)
            if it["cmd"] in prev: got[it["cmd"]].append(t - prev[it["cmd"]])
            prev[it["cmd"]] = t
    return {k: max(v) if v else None for k, v in got.items()}   # worst gap

cruise, boost = simulate(False), simulate(True)

# ---- compare
fails = 0
rows = []
for it in items:
    c = it["cmd"]
    if c in NOT_ON_S5 or c in ONCE_IN_A_WHILE: continue
    if c not in FEEDS: print(f"UNKNOWN request {c}: add it to FEEDS in tools/poll_check.py"); fails += 1; continue
    if stopped_only(it):
        rows.append((c, "only when stopped", "", "", "")); continue
    gc = cruise[c] or 0
    gb = None if paused(it) else (boost[c] or 0)
    need = max(2 * gc, 2 * gb if gb else 0, (PULL_PAUSE_MAX + gc) if paused(it) else 0)
    for f in FEEDS[c]:
        w = window.get(f, default_age)
        ok = w >= need
        fails += not ok
        rows.append((f"{c} {f}", f"{gc/1000:.1f} s", "waits" if gb is None else f"{gb/1000:.1f} s", f"{w/1000:.0f} s (needs {need/1000:.1f})", "ok" if ok else "TOO TIGHT"))
print(f"{'reading':32s} {'worst gap cruise':>16s} {'on boost':>9s}   window")
for r in rows: print(f"{r[0]:32s} {r[1]:>16s} {r[2]:>9s}   {r[3]}  {r[4]}")
# ---- every consumer takes its window from ageFor(): a hard-coded window on a polled reading can't be
# checked here, so it's a problem too (RPM / speed come every round; F_MAP 1500 is the sender stand-in's
# own validity check, with manifold pressure at 120 ms then)
polled = {f for c, fs in FEEDS.items() for f in fs}
ALLOWED = {("F_MAP", "1500"), ("F_MAP", "1000"), ("F_RPM", "300"), ("F_RPM", "400")}
fw = os.path.join(os.path.dirname(__file__), "..", "firmware", "S5GaugePod_Test")
for name in sorted(os.listdir(fw)):
    if not name.endswith((".ino", ".h")): continue
    for n, line in enumerate(open(os.path.join(fw, name)), 1):
        for f, age in re.findall(r"fresh\((?:obd::)?(F_\w+),\s*(\d+)\)", line):
            if f in polled and (f, age) not in ALLOWED:
                print(f"{name}:{n}: {f} with a fixed {age} ms window - use ageFor({f})"); fails += 1
        for f in re.findall(r"fresh\((?:obd::)?(F_\w+)\)", line):
            if f in polled: print(f"{name}:{n}: {f} with the default window - use ageFor({f})"); fails += 1
print(f"\n{'all windows fit' if not fails else str(fails) + ' problem(s)'}")
sys.exit(1 if fails else 0)
