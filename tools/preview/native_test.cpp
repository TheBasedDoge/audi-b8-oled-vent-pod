// Native build of the renderer: writes raw RGB565 frames for a fixed set of
// scenes, so they can be turned into PNGs and compared with the wasm build.
// usage: native_test <outdir>
#include <stdio.h>
#include <string.h>
#include "glue.cpp"

struct Scene { const char *name; int page; float t; int obd; int night; const char *status; int warn; const char *toast; float toastA; float batOverride; };

static void save(const char *out, const char *name) {
  char path[256];
  snprintf(path, sizeof(path), "%s/%s.rgb565", out, name);
  FILE *f = fopen(path, "wb");
  fwrite(g_fb(), 2, W * H, f);
  fclose(f);
  printf("%s\n", path);
}

int main(int argc, char **argv) {
  const char *out = argc > 1 ? argv[1] : ".";
  Scene sc[] = {
    {"boost_idle",     P_BOOST,    1.0f,  1, 0, "ZERO 1.30 V", 0, "", 0, 0},
    {"boost_pull",     P_BOOST,    6.2f,  1, 0, "", 0, "", 0, 0},
    {"boost_night",    P_BOOST,    6.2f,  1, 1, "", 0, "", 0, 0},
    {"boost_toast",    P_BOOST,    12.0f, 1, 0, "", 0, "PEAK RESET", 1, 0},
    {"overview",       P_OVERVIEW, 6.2f,  1, 0, "", 0, "", 0, 0},
    {"overview_night", P_OVERVIEW, 6.2f,  1, 1, "", 0, "", 0, 0},
    {"overview_noobd", P_OVERVIEW, 1.0f,  0, 0, "NO OBD YET", 1, "", 0, 0},
    {"temps",          P_TEMPS,    10.0f, 1, 0, "", 0, "", 0, 0},
    {"temps_night",    P_TEMPS,    10.0f, 1, 1, "", 0, "", 0, 0},
    {"temps_noobd",    P_TEMPS,    10.0f, 0, 0, "NO OBD YET", 1, "", 0, 0},
    {"engine",         P_ENGINE,   6.9f,  1, 0, "", 0, "", 0, 0},
    {"engine_night",   P_ENGINE,   6.9f,  1, 1, "", 0, "", 0, 0},
    {"system",         P_SYSTEM,   14.0f, 1, 0, "", 0, "", 0, 0},
    {"system_night",   P_SYSTEM,   14.0f, 1, 1, "", 0, "", 0, 0},
    {"system_lowbat",  P_SYSTEM,   14.0f, 1, 0, "", 0, "", 0, 11.6f},   // warning: battery forced to the top
  };
  g_cfg(20, 7200, 7400, 1, 1);
  float now = 100000;
  for (auto &s : sc) {
    float *v = g_demo(s.t);
    strcpy(g_status(), s.status);
    strcpy(g_toast(), s.toast);
    float bat = s.batOverride > 0 ? s.batOverride : v[5];
    g_theme(s.night);
    g_set(v[0], 12.3f, s.obd, s.obd ? v[1] : 0, v[2], v[3], v[4], v[7], bat, v[6], 134, 7300, s.warn, s.toastA, 0);
    g_tick(now);
    g_render(s.page, 1000);
    save(out, s.name);
  }
  {  // System page after tapping PEAK BOOST (top-left small cell -> top)
    float *v = g_demo(14.0f);
    strcpy(g_status(), ""); strcpy(g_toast(), "");
    g_theme(0);
    g_set(v[0], 12.3f, 1, v[1], v[2], v[3], v[4], v[7], v[5], v[6], 134, 7300, 0, 0, 0);
    g_tick(now);
    g_sys_tap(C - 70, C + 70);
    g_render(P_SYSTEM, 1000);
    save(out, "system_tapped");
    g_sys_tap(C - 70, C + 70);   // swap back
  }
  strcpy(g_status(), "");
  g_splash(1, 0);
  save(out, "splash");
  return 0;
}
