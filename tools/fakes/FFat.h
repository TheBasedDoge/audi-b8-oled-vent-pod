// Desktop stand-ins for the bits of Arduino, FFat and SD_MMC that logger.h uses
// (tools/log_test.cpp). Each "storage" is a map of file name -> contents and records every
// write and save. The SD card can be taken out, or made to fail its writes.
#pragma once
#include <string>
#include <map>
#include <vector>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstdarg>
#include <cctype>
#define LOGX_FAKE_FS 1

extern uint32_t fakeNow;
static inline uint32_t millis() { return fakeNow; }
static inline void *ps_malloc(size_t n) { return malloc(n); }
// strlcpy comes from glibc (2.38+)
struct FakeSerial { bool quiet = true; void printf(const char *f, ...) { if (quiet) return; va_list a; va_start(a, f); vprintf(f, a); va_end(a); } };
extern FakeSerial Serial;

class String {
 public:
  std::string s;
  String() {}
  String(const char *c) : s(c ? c : "") {}
  String(const std::string &x) : s(x) {}
  bool startsWith(const char *p) const { return s.rfind(p, 0) == 0; }
  bool endsWith(const char *p) const { size_t L = strlen(p); return s.size() >= L && s.compare(s.size() - L, L, p) == 0; }
  String substring(size_t a, size_t b) const { return String(s.substr(a, b - a)); }
  String substring(size_t a) const { return String(s.substr(a)); }
  int lastIndexOf(char c) const { size_t p = s.rfind(c); return p == std::string::npos ? -1 : (int)p; }
  void toLowerCase() { for (auto &c : s) c = (char)tolower((unsigned char)c); }
  long toInt() const { return atol(s.c_str()); }
  size_t length() const { return s.size(); }
  const char *c_str() const { return s.c_str(); }
  bool operator==(const String &o) const { return s == o.s; }
  bool operator!=(const String &o) const { return s != o.s; }
  bool operator==(const char *o) const { return s == o; }
  friend String operator+(const char *a, const String &b) { return String(std::string(a) + b.s); }
  friend String operator+(const String &a, const char *b) { return String(a.s + b); }
  friend String operator+(const String &a, const String &b) { return String(a.s + b.s); }
};

struct FakeDisk {
  struct F { std::string data; size_t durable = 0; };   // durable = what survives a power cut (saved size)
  std::map<std::string, F> files;
  struct Op { uint32_t t; char kind; size_t n, endPos; bool busy; };   // 'w' write, 's' save
  std::vector<Op> ops;
  size_t total = 9900000;
  bool present = true, failWrites = false, listingBroken = false;   // listingBroken: reading the folder finds nothing
};
extern bool fakeBusyNow;

class File {
 public:
  FakeDisk *d = nullptr;
  std::string name_; bool ok = false, isDir = false; size_t pos = 0;
  std::vector<std::string> list; size_t li = 0;
  explicit operator bool() const { return ok; }
  size_t write(const uint8_t *b, size_t n) {
    if (!d->present || d->failWrites) return 0;
    auto &f = d->files[name_]; f.data.append((const char *)b, n); pos += n;
    d->ops.push_back({fakeNow, 'w', n, pos, fakeBusyNow}); return n; }
  void flush() { if (!d->present) return; auto &f = d->files[name_]; f.durable = f.data.size(); d->ops.push_back({fakeNow, 's', 0, pos, fakeBusyNow}); }
  void close() { if (ok && !isDir) flush(); ok = false; }
  String getNextFileName(bool *dir) { if (dir) *dir = false; return li < list.size() ? String(list[li++]) : String(); }
};
#define FILE_WRITE "w"
#define FILE_READ "r"
namespace fs {
class FS {
 public:
  FakeDisk d;
  File open(const char *n, const char *mode = "r") {
    File f; f.d = &d;
    if (!d.present) return f;
    f.ok = true;
    if (!strcmp(n, "/")) { f.isDir = true; if (!d.listingBroken) for (auto &kv : d.files) f.list.push_back(kv.first); return f; }
    f.name_ = n; if (mode[0] == 'w') d.files[n] = FakeDisk::F(); return f;
  }
  File open(const String &n, const char *mode = "r") { return open(n.c_str(), mode); }
  bool remove(const String &n) { return d.files.erase(n.s) > 0; }
  uint64_t totalBytes() { return d.present ? d.total : 0; }
  uint64_t usedBytes() { size_t u = 0; for (auto &kv : d.files) u += kv.second.data.size(); return u; }
};
}
struct FakeFFat : fs::FS { bool begin(bool) { return true; } };
struct FakeSD : fs::FS {
  bool setPins(int, int, int) { return true; }
  bool begin(const char *, bool, bool, int, int) { return d.present; }
  void end() {}
};
#define SDMMC_FREQ_DEFAULT 20000
extern FakeFFat FFat;
extern FakeSD SD_MMC;
static inline bool fakeFileExists(fs::FS *fs, const String &path) { return fs->d.present && fs->d.files.count(path.s) > 0; }
static inline uint32_t fakeFileSize(fs::FS *fs, const String &path) { auto it = fs->d.files.find(path.s); return it == fs->d.files.end() ? 0 : (uint32_t)it->second.data.size(); }
