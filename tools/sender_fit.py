#!/usr/bin/env python3
"""Is the boost sender's idle difference from the ECU an offset, a slope error, or the supply voltage?

The pod zeroes the sender at every key-on, and "Match the ECU at idle" then subtracts a constant
trim learned at idle. A constant is right if the sender (or the ECU) is simply offset, wrong if the
real cause is the slope (Settings: Sender volts per psi): then boost reads too high or too low by a
few percent, more the further from atmosphere it goes. One idle point can't tell the two apart;
whole drives can.

How it works: at each new ECU manifold reading (map_kPa) taken while the sender (vsig_V) was steady,
the two are paired. The slope comes from VACUUM only (below 95 kPa): there are thousands of steady
moments there, and the ECU's sensor and the sender's tap see the same pressure. On boost the two
can differ for real (the ECU's sensor is after the intercooler cores, which lose some pressure under
flow), and steady moments are rare and the ECU's reading lags behind a rising pressure, so boost is
reported separately, as how far the sender sits from the vacuum line, and doesn't set the slope.

usage: python3 tools/sender_fit.py [--vpp 0.0862] log_0040.csv log_0041.csv ...
       (--vpp = the "Sender volts per psi" now in Settings; logs with idle, cruise and a few pulls)
"""
import csv, sys, math

KPA_PER_PSI = 6.894757
VAC_MAX_KPA = 95          # vacuum readings: below this
BOOST_MIN_KPA = 110       # boost readings: above this
STEADY_VAC_V = 0.012      # sender spread over half a second: about 0.15 psi
STEADY_BOOST_V = 0.05     # looser on boost (it's never still for long): about 0.6 psi

def solve(A, y):
    """least squares by normal equations: A = rows of features, y = targets"""
    n = len(A[0])
    M = [[sum(r[i] * r[j] for r in A) for j in range(n)] + [sum(r[i] * t for r, t in zip(A, y))] for i in range(n)]
    for i in range(n):
        p = max(range(i, n), key=lambda k: abs(M[k][i]))
        M[i], M[p] = M[p], M[i]
        if abs(M[i][i]) < 1e-12: return None
        for k in range(n):
            if k != i:
                f = M[k][i] / M[i][i]
                M[k] = [a - f * b for a, b in zip(M[k], M[i])]
    return [M[i][n] / M[i][i] for i in range(n)]

def rms(A, y, c): return math.sqrt(sum((sum(a * b for a, b in zip(r, c)) - t) ** 2 for r, t in zip(A, y)) / len(y))

def samples(path):
    """at each new ECU reading: (sender V, map kPa, battery V or None, sender spread V, sender trend V)"""
    with open(path, newline="") as fh: rows = list(csv.DictReader(fh))
    out, prev = [], None
    for i, r in enumerate(rows):
        m = r.get("map_kPa", "")
        if not m or r.get("boost_source", "sender") != "sender": prev = m; continue
        if m != prev and i >= 3 and i + 1 < len(rows):          # a new reading (it's held between readings)
            try: v = [float(rows[k]["vsig_V"]) for k in range(i - 3, i + 2)]
            except (ValueError, KeyError): prev = m; continue
            if min(v) > 0.1:
                b = r.get("battery_V", "")
                out.append((sum(v[1:4]) / 3, float(m), float(b) if b else None, max(v) - min(v), v[4] - v[0]))
        prev = m
    return out

def main():
    args = sys.argv[1:]
    if not args or any(a in ("-h", "--help") for a in args): print(__doc__); return 0 if args else 1
    vpp = 0.0862
    if "--vpp" in args:
        k = args.index("--vpp")
        try: vpp = float(args[k + 1])
        except (IndexError, ValueError): print("--vpp needs a number, e.g. --vpp 0.0862"); return 1
        del args[k:k + 2]
    S = []
    for p in args:
        try: s = samples(p)
        except OSError as e: print(f"{p}: can't read ({e.strerror})"); return 1
        S += s
        print(f"{p}: {len(s)} ECU readings" + (f", manifold {min(x[1] for x in s):.0f}-{max(x[1] for x in s):.0f} kPa" if s else ""))

    V = [x for x in S if x[1] < VAC_MAX_KPA and x[3] < STEADY_VAC_V]
    if len(V) < 30 or max(x[1] for x in V) - min(x[1] for x in V) < 30:
        print("\nNot enough steady vacuum readings (30 at least, over 30 kPa or more): add logs with idle and cruise."); return 1
    A = [[1.0, x[1]] for x in V]; y = [x[0] for x in V]
    a, b = solve(A, y)
    fit = b * KPA_PER_PSI
    print(f"\nVacuum: {len(V)} steady readings, {min(x[1] for x in V):.0f}-{max(x[1] for x in V):.0f} kPa absolute")
    print(f"  best fit: {fit:.4f} V per psi (setting now: {vpp:.4f}), {a:+.3f} V at zero pressure, scatter {rms(A, y, [a, b]) / fit:.2f} psi")
    absolute = abs(a) < 0.05
    print("  " + ("the line goes through about 0 V at 0 kPa, as an absolute sensor's should: the fit can be trusted"
                  if absolute else "the line does NOT go through 0 V at 0 kPa: something besides the slope is off (wiring, supply?)"))
    idle = 30.0                                                 # a typical idle, kPa absolute
    atm = max(x[1] for x in V) if max(x[1] for x in V) > 90 else 100.0
    idleErr = (atm - idle) / KPA_PER_PSI * (fit / vpp - 1)      # how much more vacuum the pod shows at idle than the ECU
    print(f"  with the setting as it is: idle shows {idleErr:.2f} psi more vacuum than the ECU, 15 psi of boost shows {15 * fit / vpp:.1f}")

    Bt = [x for x in S if x[1] > BOOST_MIN_KPA and x[3] < STEADY_BOOST_V]
    if len(Bt) >= 15:
        def off(sel): return sum((x[0] - (a + b * x[1])) / fit for x in sel) / len(sel)
        notRising = [x for x in Bt if x[4] <= 0.01]
        rising = [x for x in Bt if x[4] > 0.01]
        print(f"\nBoost: {len(Bt)} readings, {min(x[1] for x in Bt):.0f}-{max(x[1] for x in Bt):.0f} kPa absolute")
        if len(notRising) >= 8:
            print(f"  holding or falling ({len(notRising)}): the sender reads {off(notRising):+.2f} psi from the vacuum line")
        if rising:
            print(f"  rising ({len(rising)}): {off(rising):+.2f} psi (the ECU's reading lags a rising pressure: read this one with care)")
        print("  A positive number here is partly real: the ECU's sensor is after the intercooler cores, which lose some")
        print("  pressure under flow. The ECU can't separate that from a slope error on boost; a hand vacuum pump can't either")
        print("  (it only reaches vacuum). It doesn't change the slope above.")
    else:
        print(f"\nBoost: only {len(Bt)} usable readings (15 needed): add logs with pulls to see how the sender compares on boost.")

    B = [x for x in V if x[2] is not None]
    if len(B) >= 30:
        mb = sum(x[2] for x in B) / len(B); sd = math.sqrt(sum((x[2] - mb) ** 2 for x in B) / len(B))
        if sd < 0.15: print(f"\nSupply: battery voltage hardly moved in these readings ({mb:.1f} V): can't tell whether the sender follows it")
        else:
            c = solve([[1.0, x[1], x[2] - mb] for x in B], [x[0] for x in B])
            if c:
                psiPerV = c[2] / (c[1] * KPA_PER_PSI)
                print(f"\nSupply: {c[2] * 1000:+.0f} mV per volt of battery = {psiPerV * 1.8:+.2f} psi between 12.4 V (key-on zero) and 14.2 V (running)")
                if abs(psiPerV * 1.8) > 0.2: print("  -> the sender follows its supply: that alone explains an idle offset (a constant trim is the right fix)")

    print()
    if not absolute: print("Verdict: don't change the slope from this; check the sender's wiring and supply first.")
    elif abs(fit / vpp - 1) < 0.02: print("Verdict: the slope setting is right (within 2 %). What's left at idle is an offset; the idle trim as it is fits.")
    else: print(f"Verdict: the slope is off by {100 * (fit / vpp - 1):+.1f} % (from vacuum). Set 'Sender volts per psi' to {fit:.4f} in Settings; "
                "the idle trim should then settle near zero. A hand vacuum pump check at 10 and 20 inHg would confirm it.")
    return 0

if __name__ == "__main__": sys.exit(main())
