// =====================================================================
//  S5 Gauge Pod - summary of one pull (filled by watch.h, drawn by gauge_render.h)
//  Plain C++, no Arduino calls. NOV marks "no reading".
// =====================================================================
#pragma once
#include <stdint.h>

namespace gr {

static const float NOV = -9999.0f;                     // "no value"
static inline bool has(float v) { return v > -9000.0f; }

struct PullSum {
  bool     valid = false;
  uint32_t endMs = 0;                                  // when it ended (millis)
  float    durS = 0, kmh0 = 0, kmh1 = 0;
  char     gears[16] = "";                             // gears used, e.g. "2-3-4"
  float    peakPsi = NOV, peakRpm = 0;                 // sender
  float    obdPeak = NOV, tgtPeak = NOV;               // ECU manifold pressure / target, psi
  float    iat0 = NOV, iatMax = NOV;                   // intake after the intercooler, degF
  float    minTiming = NOV;
  float    knock = NOV; int knockCyl = 0;              // worst retard (<= 0), which cylinder
  float    hpfpDef = NOV;                              // worst (actual - setpoint), psi (negative = below)
  float    hpfpMin = NOV, lpfpMin = NOV;               // psi
  float    afrMax = NOV;                               // leanest AFR on boost, either bank
  float    afrOver = NOV;                              // most leaner than commanded, AFR (NOV = no commanded value)
  float    idcMax = NOV;                               // injector duty, %
  float    catMax = NOV;                               // degF, either bank
  float    bypassMin = NOV;                            // deg
  float    lonMax = NOV;                               // g
  float    vsNormal = NOV;                             // psi against this car's usual boost at the same RPM (NOV = still learning)
  int      normalPulls = 0;                            // pulls the usual-boost curve has learned from
  bool     slip = false;                               // boost fell while RPM kept rising (belt slip?)
  // highest boost in each 500 rpm band, 2500-7500, full pedal in 2nd and up (NOV = not reached):
  // the health trends compare these against a fixed baseline, so a slow drift still shows
  float    binPsi[10] = {NOV, NOV, NOV, NOV, NOV, NOV, NOV, NOV, NOV, NOV};
};

}  // namespace gr
