// DAGMC driver of smrt-bench (dagmc-moab and dagmc-dd rows), derived from the W3a driver of
// sm-rt-capi/bench/dagmc (copied, not edited there).
//
//   dagmc_driver --version
//   dagmc_driver model.h5m rays.bin --tracker NAME --mode MODE --precision P
//                [--threads N] [--reps 7] [--warmup 3] [--audit 1000] [--tracks]
//
// W-track: follows every ray of "smrays v1" to leakage with ray_fire (orientation 1, with a
// RayHistory) + next_vol until it enters the implicit complement, as OpenMC would, starting from
// the region the ray file names (the hint). Prints one JSON object per line in the
// `smbench-result v1` row format (see DESIGN.md "Protocol for C++ drivers"), and with --tracks one
// more line {"tracks": [{"r": [...], "c": [...]}, ...]} holding every ray's region ids (region
// `id` is the global volume id minus one, -1 the complement) and chords in ray order, which the
// Rust runner compares with the sm-rt rows on the same facets. Only W-track is implemented here;
// W-fire and W-flight rows of DAGMC are not produced by this driver.
//
// No I/O and no allocation in the timed loop beyond reusing per-thread buffers that the warm-up
// repetitions have grown; a running FNV-1a hash of every output (ray order) is the anti-dead-code
// checksum and the determinism hash. A particle is lost when ray_fire or next_vol errors, when
// ray_fire finds no surface from inside a bounded volume, or when the 1-in-`audit`
// point_in_volume audit disagrees (the implicit complement is never audited).
#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "DagMC.hpp"

#ifndef SMRT_BENCH_DOUBLE_DOWN
#define SMRT_BENCH_BACKEND "moab-obb"
#else
#define SMRT_BENCH_BACKEND "double-down"
#endif

using moab::EntityHandle;

struct Rays {
  std::vector<double> o, d;  // 3 per ray
  std::vector<int32_t> region;
  size_t n = 0;
};

static Rays read_rays(const char* path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) { std::fprintf(stderr, "cannot open %s\n", path); std::exit(2); }
  uint32_t n = 0;
  f.read(reinterpret_cast<char*>(&n), 4);
  Rays r;
  r.n = n;
  std::vector<double> raw(static_cast<size_t>(n) * 6);
  f.read(reinterpret_cast<char*>(raw.data()), raw.size() * 8);
  r.region.resize(n);
  f.read(reinterpret_cast<char*>(r.region.data()), static_cast<std::streamsize>(n) * 4);
  if (!f) { std::fprintf(stderr, "%s: truncated smrays file\n", path); std::exit(2); }
  r.o.resize(static_cast<size_t>(n) * 3);
  r.d.resize(static_cast<size_t>(n) * 3);
  for (size_t i = 0; i < n; i++)
    for (int k = 0; k < 3; k++) { r.o[3 * i + k] = raw[6 * i + k]; r.d[3 * i + k] = raw[6 * i + 3 + k]; }
  return r;
}

// Per-thread output of a contiguous range of rays.
struct Chunk {
  std::vector<int32_t> region;   // one entry per segment
  std::vector<double> chord;
  std::vector<uint32_t> count;   // segments per ray of the range
  uint64_t crossings = 0, lost = 0, audits = 0;
  void clear() { region.clear(); chord.clear(); count.clear(); crossings = lost = audits = 0; }
};

static void trace_range(moab::DagMC& dag, const Rays& R, size_t lo, size_t hi, EntityHandle ic,
                        const std::vector<EntityHandle>& vols, unsigned audit_every, Chunk& c) {
  // Region id of a volume handle: its position in `vols` (region r has global id r + 1).
  auto region_of = [&](EntityHandle v) -> int32_t {
    if (v == ic) return -1;
    for (size_t k = 0; k < vols.size(); k++) if (vols[k] == v) return static_cast<int32_t>(k);
    return -2;
  };
  c.clear();
  for (size_t i = lo; i < hi; i++) {
    double pos[3] = {R.o[3 * i], R.o[3 * i + 1], R.o[3 * i + 2]};
    const double dir[3] = {R.d[3 * i], R.d[3 * i + 1], R.d[3 * i + 2]};
    const int32_t r0 = R.region[i];
    EntityHandle vol = r0 < 0 ? ic : vols[static_cast<size_t>(r0)];
    moab::DagMC::RayHistory history;
    uint32_t segs = 0;
    bool lost = false;
    if (audit_every && vol != ic && (i % audit_every) == 0) {
      int inside = 0;
      c.audits++;
      if (dag.point_in_volume(vol, pos, inside, dir) != moab::MB_SUCCESS || inside != 1) { c.lost++; c.count.push_back(0); continue; }
    }
    for (unsigned step = 0; step < 1000000 && !lost; step++) {
      EntityHandle surf = 0, next = 0;
      double dist = 0.0;
      if (dag.ray_fire(vol, pos, dir, surf, dist, &history, 0.0, 1) != moab::MB_SUCCESS || surf == 0) { lost = true; break; }
      if (dag.next_vol(surf, vol, next) != moab::MB_SUCCESS) { lost = true; break; }
      // The chord just flown lies in `vol` (region id = global id - 1).
      c.region.push_back(region_of(vol));
      c.chord.push_back(dist);
      segs++;
      for (int k = 0; k < 3; k++) pos[k] += dist * dir[k];
      c.crossings++;
      if (audit_every && next != ic && ((i + step) % audit_every) == 0) {
        c.audits++;
        const double probe[3] = {pos[0] + 1e-6 * dir[0], pos[1] + 1e-6 * dir[1], pos[2] + 1e-6 * dir[2]};
        int inside = 0;
        if (dag.point_in_volume(next, probe, inside, dir, &history) != moab::MB_SUCCESS || inside != 1) { lost = true; break; }
      }
      vol = next;
      if (vol == ic) break;  // leaked into the implicit complement
    }
    if (lost) c.lost++;
    c.count.push_back(segs);
  }
}

static uint64_t fnv(uint64_t h, uint64_t x) {
  for (int k = 0; k < 8; k++) { h ^= (x >> (8 * k)) & 0xff; h *= 0x100000001b3ULL; }
  return h;
}

static double secs(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b) {
  return std::chrono::duration<double>(b - a).count();
}

static void stat(std::vector<double> v, double& med, double& mn, double& mx) {
  std::sort(v.begin(), v.end());
  med = v[v.size() / 2]; mn = v.front(); mx = v.back();
}

int main(int argc, char** argv) {
  if (argc == 2 && std::strcmp(argv[1], "--version") == 0) {
    std::string v;
    moab::DagMC::version(&v);
    std::printf("dagmc_driver backend=%s dagmc=%s\n", SMRT_BENCH_BACKEND, v.c_str());
    return 0;
  }
  if (argc < 3) { std::fprintf(stderr, "usage: %s model.h5m rays.bin [options] | --version\n", argv[0]); return 2; }
  unsigned threads = 1, reps = 7, warmup = 3, audit_every = 1000;
  bool dump = false;
  std::string tracker = "dagmc", mode = SMRT_BENCH_BACKEND, precision = "f64";
  for (int i = 3; i < argc; i++) {
    const std::string k = argv[i];
    if (k == "--tracks") { dump = true; continue; }
    if (i + 1 >= argc) { std::fprintf(stderr, "option %s needs a value\n", k.c_str()); return 2; }
    const std::string v = argv[++i];
    if (k == "--threads") threads = std::max(1, std::atoi(v.c_str()));
    else if (k == "--reps") reps = std::max(1, std::atoi(v.c_str()));
    else if (k == "--warmup") warmup = std::max(0, std::atoi(v.c_str()));
    else if (k == "--audit") audit_every = std::max(0, std::atoi(v.c_str()));
    else if (k == "--tracker") tracker = v;
    else if (k == "--mode") mode = v;
    else if (k == "--precision") precision = v;
    else { std::fprintf(stderr, "unknown option %s\n", k.c_str()); return 2; }
  }

  using clk = std::chrono::steady_clock;
  auto mb = std::make_shared<moab::Core>();
  moab::DagMC dag(mb);
  const auto t0 = clk::now();
  if (dag.load_file(argv[1]) != moab::MB_SUCCESS) { std::fprintf(stderr, "load_file failed\n"); return 3; }
  const auto t1 = clk::now();
  if (dag.init_OBBTree() != moab::MB_SUCCESS) { std::fprintf(stderr, "init_OBBTree failed\n"); return 3; }
  const auto t2 = clk::now();
  const double load_s = secs(t0, t1), tree_s = secs(t1, t2);

  std::vector<EntityHandle> vols;
  const unsigned nvol = dag.num_entities(3);
  EntityHandle ic = 0;
  if (dag.geom_tool()->get_implicit_complement(ic) != moab::MB_SUCCESS) { std::fprintf(stderr, "no implicit complement\n"); return 3; }
  for (unsigned id = 1; id <= nvol; id++) vols.push_back(dag.entity_by_id(3, static_cast<int>(id)));

  const Rays R = read_rays(argv[2]);
  std::vector<Chunk> chunks(threads);
  const size_t span = (R.n + threads - 1) / threads;
  auto run_all = [&]() {
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < threads; t++) {
      const size_t lo = std::min<size_t>(R.n, static_cast<size_t>(t) * span), hi = std::min<size_t>(R.n, lo + span);
      pool.emplace_back([&, t, lo, hi] { trace_range(dag, R, lo, hi, ic, vols, audit_every, chunks[t]); });
    }
    for (auto& th : pool) th.join();
  };
  auto hash_all = [&]() {
    uint64_t h = 0xcbf29ce484222325ULL;
    for (const auto& c : chunks) {
      size_t s = 0;
      for (uint32_t n : c.count) {
        h = fnv(h, n);
        for (uint32_t k = 0; k < n; k++, s++) { h = fnv(h, static_cast<uint32_t>(c.region[s])); uint64_t b; std::memcpy(&b, &c.chord[s], 8); h = fnv(h, b); }
      }
    }
    return h;
  };

  std::vector<double> rates;
  uint64_t first_hash = 0, segments = 0, lost = 0;
  bool stable = true;
  for (unsigned rep = 0; rep < warmup + reps; rep++) {
    const auto a = clk::now();
    run_all();
    const double s = secs(a, clk::now());
    const uint64_t h = hash_all();
    if (rep == 0) first_hash = h; else if (h != first_hash) stable = false;
    if (rep >= warmup) rates.push_back(static_cast<double>(R.n) / s);
  }
  for (const auto& c : chunks) { segments += c.chord.size(); lost += c.lost; }
  double rm, rmin, rmax;
  stat(rates, rm, rmin, rmax);
  struct rusage ru;
  getrusage(RUSAGE_SELF, &ru);
  const unsigned long long rss = static_cast<unsigned long long>(ru.ru_maxrss) * 1024ULL;  // Linux: kilobytes
  const double spt = R.n ? static_cast<double>(segments) / static_cast<double>(R.n) : 0.0;
  char hashbuf[17];
  std::snprintf(hashbuf, sizeof hashbuf, "%016llx", static_cast<unsigned long long>(first_hash));
  std::printf(
      "{\"tracker\":\"%s\",\"mode\":\"%s\",\"workload\":\"w-track\",\"threads\":%u,\"precision\":\"%s\","
      "\"rays_per_s\":{\"median\":%.1f,\"min\":%.1f,\"max\":%.1f,\"reps\":%u},"
      "\"segments_per_track\":%.6f,\"ns_per_segment\":%.3f,\"build_s\":%.6f,\"load_s\":%.6f,\"tree_s\":%.6f,"
      "\"peak_rss_bytes\":%llu,\"lost\":%llu,"
      "\"determinism\":{\"hash\":\"%s\",\"stable\":%s},"
      "\"counters\":{\"steps\":%llu,\"segments\":%llu,\"lost\":%llu}}\n",
      tracker.c_str(), mode.c_str(), threads, precision.c_str(), rm, rmin, rmax, reps, spt,
      segments ? 1e9 / (rm * spt) : 0.0,
      load_s + tree_s, load_s, tree_s, rss, static_cast<unsigned long long>(lost), hashbuf, stable ? "true" : "false",
      static_cast<unsigned long long>(segments), static_cast<unsigned long long>(segments), static_cast<unsigned long long>(lost));
  if (dump) {
    std::printf("{\"tracks\":[");
    bool first = true;
    for (const auto& c : chunks) {
      size_t s = 0;
      for (uint32_t n : c.count) {
        std::printf("%s{\"r\":[", first ? "" : ",");
        first = false;
        for (uint32_t k = 0; k < n; k++) std::printf("%s%d", k ? "," : "", c.region[s + k]);
        std::printf("],\"c\":[");
        for (uint32_t k = 0; k < n; k++) std::printf("%s%.17g", k ? "," : "", c.chord[s + k]);
        std::printf("]}");
        s += n;
      }
    }
    std::printf("]}\n");
  }
  return 0;
}
