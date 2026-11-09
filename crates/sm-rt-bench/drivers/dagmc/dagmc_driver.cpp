// DAGMC driver of smrt-bench (dagmc-moab and dagmc-dd rows), derived from the W3a driver of
// sm-rt-capi/bench/dagmc (copied, not edited there).
//
//   dagmc_driver --version
//   dagmc_driver model.h5m rays.bin --tracker NAME --mode MODE --precision P
//                [--workload track|fire|fire-locate|flight] [--threads N] [--reps 7] [--warmup 3]
//                [--audit 1000] [--tracks] [--histories N --seed S --sigma sigma.txt]
//
// W-track: follows every ray of "smrays v1" to leakage with ray_fire (orientation 1, with a
// RayHistory) + next_vol until it enters the implicit complement, as OpenMC would, starting from
// the region the ray file names (the hint). Prints one JSON object per line in the
// `smbench-result v1` row format (see DESIGN.md "Protocol for C++ drivers"), and with --tracks one
// more line {"tracks": [{"r": [...], "c": [...]}, ...]} holding every ray's region ids (region
// `id` is the global volume id minus one, -1 the complement) and chords in ray order, which the
// Rust runner compares with the sm-rt rows on the same facets.
//
// W-fire (--workload fire): one ray_fire per ray from the region the ray file names (hinted): the
// distance to the first region boundary and the region entered (-1 complement, -2 lost). With
// --workload fire-locate the start volume comes from a find_volume loop instead of the hint. --tracks
// then prints {"fires": [[dist, to_region], ...]} in ray order.
//
// W-flight (--workload flight): seeded isotropic-scatter histories in surface mode, the same
// physics as the Rust adapter (adapter.rs `history`): exponential optical depth, collision and
// track-length tallies, leaked and absorbed counts, in blocks of 256 histories whose tallies are
// reduced in block order (the result does not depend on the thread count). Random numbers: a port
// of the Rust SplitMix64 stream per history, seeded from --seed and the history index (never from
// time or thread). std::log and std::sqrt may differ from Rust's libm in the last bit, so the
// tallies agree with the Rust rows statistically, not bitwise. History h starts from ray
// h % rays, in the region the ray file names. --sigma is a text file with one line per region:
// `id sigma_t sigma_a`. --tracks then prints {"tally": [...], "tally_sq": [...], "blocks": B}.
//
// No I/O and no allocation in the timed loop beyond reusing per-thread buffers that the warm-up
// repetitions have grown; a running FNV-1a hash of every output (ray order) is the anti-dead-code
// checksum and the determinism hash. A particle is lost when ray_fire or next_vol errors, when
// ray_fire finds no surface from inside a bounded volume, or when the 1-in-`audit`
// point_in_volume audit disagrees (the implicit complement is never audited; W-track only).
#include <sys/resource.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
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
    if (r0 >= 0 && static_cast<size_t>(r0) >= vols.size()) { c.lost++; c.count.push_back(0); continue; }
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

// ---- W-fire -------------------------------------------------------------------------------

struct FireChunk {
  std::vector<double> dist;
  std::vector<int32_t> to;  // region entered, -1 complement, -2 lost
  uint64_t lost = 0, locates = 0;
  void clear() { dist.clear(); to.clear(); lost = locates = 0; }
};

static void fire_range(moab::DagMC& dag, const Rays& R, size_t lo, size_t hi, EntityHandle ic,
                       const std::vector<EntityHandle>& vols, bool locate, FireChunk& c) {
  auto region_of = [&](EntityHandle v) -> int32_t {
    if (v == ic) return -1;
    for (size_t k = 0; k < vols.size(); k++) if (vols[k] == v) return static_cast<int32_t>(k);
    return -2;
  };
  c.clear();
  for (size_t i = lo; i < hi; i++) {
    const double pos[3] = {R.o[3 * i], R.o[3 * i + 1], R.o[3 * i + 2]};
    const double dir[3] = {R.d[3 * i], R.d[3 * i + 1], R.d[3 * i + 2]};
    const int32_t r0 = R.region[i];
    if (r0 >= 0 && static_cast<size_t>(r0) >= vols.size()) { c.lost++; c.dist.push_back(0.0); c.to.push_back(-2); continue; }
    EntityHandle vol = r0 < 0 ? ic : vols[static_cast<size_t>(r0)];
    if (locate) {
      c.locates++;
      // DAGMC's find_volume (a linear loop over the volumes, then the implicit complement) is not
      // exported by libdagmc 3.2.4 (its declaration is guarded by MOAB_VERSION_* macros no header
      // defines), so the same loop is written out here with point_in_volume.
      vol = 0;
      bool err = false;
      for (size_t k = 0; k < vols.size() && vol == 0 && !err; k++) {
        int inside = 0;
        if (dag.point_in_volume(vols[k], pos, inside, dir) != moab::MB_SUCCESS) err = true;
        else if (inside == 1) vol = vols[k];
      }
      if (vol == 0 && !err) vol = ic;
      if (err) { c.lost++; c.dist.push_back(0.0); c.to.push_back(-2); continue; }
    }
    EntityHandle surf = 0, next = 0;
    double d = 0.0;
    if (dag.ray_fire(vol, pos, dir, surf, d, nullptr, 0.0, 1) != moab::MB_SUCCESS || surf == 0 ||
        dag.next_vol(surf, vol, next) != moab::MB_SUCCESS) {
      c.lost++; c.dist.push_back(0.0); c.to.push_back(-2); continue;
    }
    c.dist.push_back(d);
    c.to.push_back(region_of(next));
  }
}

// ---- W-flight -----------------------------------------------------------------------------

// Port of sm-rt-bench's SplitMix64 (src/rng.rs).
struct SplitMix64 {
  uint64_t s;
  static SplitMix64 for_index(uint64_t seed, uint64_t index) {
    SplitMix64 m{seed ^ (index * 0x9E3779B97F4A7C15ULL)};
    m.next_u64();
    return m;
  }
  uint64_t next_u64() {
    s += 0x9E3779B97F4A7C15ULL;
    uint64_t z = s;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
  }
  double unit() { return static_cast<double>(next_u64() >> 11) * (1.0 / 9007199254740992.0); }
  void direction(double d[3]) {
    for (;;) {
      const double x = 2.0 * unit() - 1.0, y = 2.0 * unit() - 1.0, z = 2.0 * unit() - 1.0;
      const double n2 = x * x + y * y + z * z;
      if (1e-6 < n2 && n2 <= 1.0) {
        const double n = std::sqrt(n2);
        d[0] = x / n; d[1] = y / n; d[2] = z / n;
        return;
      }
    }
  }
};

struct Sigma {
  std::vector<double> t, a;  // by region id
};

static Sigma read_sigma(const char* path) {
  std::ifstream f(path);
  if (!f) { std::fprintf(stderr, "cannot open %s\n", path); std::exit(2); }
  std::vector<std::array<double, 3>> all;
  long id = 0;
  double st = 0, sa = 0;
  long maxid = -1;
  while (f >> id >> st >> sa) { all.push_back({static_cast<double>(id), st, sa}); maxid = std::max(maxid, id); }
  Sigma s;
  s.t.assign(static_cast<size_t>(maxid + 1), 0.0);
  s.a.assign(static_cast<size_t>(maxid + 1), 0.0);
  for (const auto& r : all)
    if (r[0] >= 0) { s.t[static_cast<size_t>(r[0])] = r[1]; s.a[static_cast<size_t>(r[0])] = r[2]; }
  return s;
}

constexpr size_t BLOCK = 256;

struct FlightCounts { uint64_t steps = 0, collisions = 0, lost = 0, histories = 0; };

// One history; tally layout [collisions per region | track length per region | leaked | absorbed].
static void flight_history(moab::DagMC& dag, const Rays& R, uint64_t h, uint64_t seed, EntityHandle ic,
                           const std::vector<EntityHandle>& vols, const Sigma& S, double* tally, FlightCounts& c) {
  const size_t nreg = S.t.size();
  const size_t k = static_cast<size_t>(h % R.n);
  SplitMix64 rng = SplitMix64::for_index(seed, h);
  double pos[3] = {R.o[3 * k], R.o[3 * k + 1], R.o[3 * k + 2]};
  double dir[3] = {R.d[3 * k], R.d[3 * k + 1], R.d[3 * k + 2]};
  c.histories++;
  const int32_t r0 = R.region[k];
  if (r0 < 0 || static_cast<size_t>(r0) >= vols.size()) { c.lost++; return; }
  EntityHandle vol = vols[static_cast<size_t>(r0)];
  size_t reg = static_cast<size_t>(r0);
  moab::DagMC::RayHistory hist;
  double tau = -std::log(1.0 - rng.unit());
  for (unsigned step = 0; step < 10000000; step++) {
    if (reg >= nreg) { c.lost++; return; }
    const double sig = S.t[reg];
    const double max = sig > 0.0 ? tau / sig : std::numeric_limits<double>::infinity();
    EntityHandle surf = 0, next = 0;
    double dist = 0.0;
    if (dag.ray_fire(vol, pos, dir, surf, dist, &hist, 0.0, 1) != moab::MB_SUCCESS || surf == 0) { c.lost++; return; }
    c.steps++;
    if (max < dist) {  // collision inside the volume
      tally[nreg + reg] += max;
      for (int q = 0; q < 3; q++) pos[q] += max * dir[q];
      tally[reg] += 1.0;
      c.collisions++;
      if (rng.unit() * sig < S.a[reg]) { tally[2 * nreg + 1] += 1.0; return; }
      rng.direction(dir);
      hist.reset();
      tau = -std::log(1.0 - rng.unit());
      continue;
    }
    tally[nreg + reg] += dist;
    tau -= sig * dist;
    for (int q = 0; q < 3; q++) pos[q] += dist * dir[q];
    if (dag.next_vol(surf, vol, next) != moab::MB_SUCCESS) { c.lost++; return; }
    if (next == ic) { tally[2 * nreg] += 1.0; return; }
    vol = next;
    reg = nreg;
    for (size_t q = 0; q < vols.size(); q++) if (vols[q] == vol) { reg = q; break; }
  }
  c.lost++;
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

static uint64_t bits(double x) { uint64_t b; std::memcpy(&b, &x, 8); return b; }

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
  uint64_t histories = 10000, seed = 0x5eed;
  std::string tracker = "dagmc", mode = SMRT_BENCH_BACKEND, precision = "f64", workload = "track", sigma_path;
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
    else if (k == "--workload") workload = v;
    else if (k == "--histories") histories = std::strtoull(v.c_str(), nullptr, 0);
    else if (k == "--seed") seed = std::strtoull(v.c_str(), nullptr, 0);
    else if (k == "--sigma") sigma_path = v;
    else { std::fprintf(stderr, "unknown option %s\n", k.c_str()); return 2; }
  }
  const bool w_track = workload == "track", w_fire = workload == "fire", w_locate = workload == "fire-locate", w_flight = workload == "flight";
  if (!(w_track || w_fire || w_locate || w_flight)) { std::fprintf(stderr, "unknown workload %s\n", workload.c_str()); return 2; }
  if (w_flight && sigma_path.empty()) { std::fprintf(stderr, "--workload flight needs --sigma\n"); return 2; }

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
  if (R.n == 0) { std::fprintf(stderr, "no rays\n"); return 2; }
  Sigma S;
  if (w_flight) {
    S = read_sigma(sigma_path.c_str());  // regions without a line are lost (as in the Rust adapter)
  }
  const size_t nreg = S.t.size(), width = 2 * nreg + 2;
  const size_t n_blocks = static_cast<size_t>((histories + BLOCK - 1) / BLOCK);

  std::vector<Chunk> chunks(threads);
  std::vector<FireChunk> fchunks(threads);
  std::vector<double> block_tally(w_flight ? n_blocks * width : 0);
  std::vector<FlightCounts> block_counts(w_flight ? n_blocks : 0);
  const size_t span = (R.n + threads - 1) / threads;
  const auto spawn = [&](const std::function<void(unsigned)>& body) {
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < threads; t++) pool.emplace_back([&body, t] { body(t); });
    for (auto& th : pool) th.join();
  };
  auto run_all = [&]() {
    if (w_flight) {
      std::fill(block_tally.begin(), block_tally.end(), 0.0);
      std::atomic<size_t> next_block{0};
      spawn([&](unsigned) {
        for (size_t b = next_block++; b < n_blocks; b = next_block++) {
          FlightCounts fc;
          const uint64_t h1 = std::min<uint64_t>(histories, (b + 1) * BLOCK);
          for (uint64_t h = b * BLOCK; h < h1; h++) flight_history(dag, R, h, seed, ic, vols, S, &block_tally[b * width], fc);
          block_counts[b] = fc;
        }
      });
      return;
    }
    spawn([&](unsigned t) {
      const size_t lo = std::min<size_t>(R.n, static_cast<size_t>(t) * span), hi = std::min<size_t>(R.n, lo + span);
      if (w_track) trace_range(dag, R, lo, hi, ic, vols, audit_every, chunks[t]);
      else fire_range(dag, R, lo, hi, ic, vols, w_locate, fchunks[t]);
    });
  };
  auto hash_all = [&]() {
    uint64_t h = 0xcbf29ce484222325ULL;
    if (w_flight) {
      for (double v : block_tally) h = fnv(h, bits(v));
      return h;
    }
    if (!w_track) {
      for (const auto& c : fchunks)
        for (size_t s = 0; s < c.dist.size(); s++) { h = fnv(h, bits(c.dist[s])); h = fnv(h, static_cast<uint32_t>(c.to[s])); }
      return h;
    }
    for (const auto& c : chunks) {
      size_t s = 0;
      for (uint32_t n : c.count) {
        h = fnv(h, n);
        for (uint32_t k = 0; k < n; k++, s++) { h = fnv(h, static_cast<uint32_t>(c.region[s])); h = fnv(h, bits(c.chord[s])); }
      }
    }
    return h;
  };

  std::vector<double> rates;
  uint64_t first_hash = 0;
  bool stable = true;
  const double units = w_flight ? static_cast<double>(histories) : static_cast<double>(R.n);
  for (unsigned rep = 0; rep < warmup + reps; rep++) {
    const auto a = clk::now();
    run_all();
    const double s = secs(a, clk::now());
    const uint64_t h = hash_all();
    if (rep == 0) first_hash = h; else if (h != first_hash) stable = false;
    if (rep >= warmup) rates.push_back(units / s);
  }
  uint64_t segments = 0, lost = 0, steps = 0, collisions = 0, locates = 0;
  std::vector<double> tally(width, 0.0), tally_sq(width, 0.0);
  if (w_track) {
    for (const auto& c : chunks) { segments += c.chord.size(); lost += c.lost; }
    steps = segments;
  } else if (w_flight) {
    for (size_t b = 0; b < n_blocks; b++) {  // fixed block order
      for (size_t k = 0; k < width; k++) {
        const double v = block_tally[b * width + k];
        tally[k] += v;
        tally_sq[k] += v * v;
      }
      steps += block_counts[b].steps; collisions += block_counts[b].collisions; lost += block_counts[b].lost;
    }
  } else {
    for (const auto& c : fchunks) { lost += c.lost; locates += c.locates; steps += c.dist.size(); }
  }
  double rm, rmin, rmax;
  stat(rates, rm, rmin, rmax);
  struct rusage ru;
  getrusage(RUSAGE_SELF, &ru);
  const unsigned long long rss = static_cast<unsigned long long>(ru.ru_maxrss) * 1024ULL;  // Linux: kilobytes
  const double spt = w_track ? static_cast<double>(segments) / static_cast<double>(R.n) : 0.0;
  const double fph = w_flight ? static_cast<double>(steps) / static_cast<double>(histories ? histories : 1) : (w_track ? 0.0 : 1.0);
  char hashbuf[17];
  std::snprintf(hashbuf, sizeof hashbuf, "%016llx", static_cast<unsigned long long>(first_hash));
  const char* wname = w_track ? "w-track" : w_fire ? "w-fire" : w_locate ? "w-fire-locate" : "w-flight";
  std::printf(
      "{\"tracker\":\"%s\",\"mode\":\"%s\",\"workload\":\"%s\",\"threads\":%u,\"precision\":\"%s\","
      "\"rays_per_s\":{\"median\":%.1f,\"min\":%.1f,\"max\":%.1f,\"reps\":%u},"
      "\"segments_per_track\":%.6f,\"ns_per_segment\":%.3f,\"fires_per_history\":%.6f,"
      "\"build_s\":%.6f,\"load_s\":%.6f,\"tree_s\":%.6f,"
      "\"peak_rss_bytes\":%llu,\"lost\":%llu,"
      "\"determinism\":{\"hash\":\"%s\",\"stable\":%s},"
      "\"counters\":{\"steps\":%llu,\"segments\":%llu,\"fires\":%llu,\"locates\":%llu,\"histories\":%llu,\"collisions\":%llu,\"lost\":%llu}}\n",
      tracker.c_str(), mode.c_str(), wname, threads, precision.c_str(), rm, rmin, rmax, reps, spt,
      segments ? 1e9 / (rm * spt) : 0.0, fph,
      load_s + tree_s, load_s, tree_s, rss, static_cast<unsigned long long>(lost), hashbuf, stable ? "true" : "false",
      static_cast<unsigned long long>(steps), static_cast<unsigned long long>(segments),
      static_cast<unsigned long long>(w_track || w_flight ? 0 : steps), static_cast<unsigned long long>(locates),
      static_cast<unsigned long long>(w_flight ? histories : 0), static_cast<unsigned long long>(collisions),
      static_cast<unsigned long long>(lost));
  if (dump && w_track) {
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
  } else if (dump && !w_flight) {
    std::printf("{\"fires\":[");
    bool first = true;
    for (const auto& c : fchunks)
      for (size_t s = 0; s < c.dist.size(); s++) {
        std::printf("%s[%.17g,%d]", first ? "" : ",", c.dist[s], c.to[s]);
        first = false;
      }
    std::printf("]}\n");
  } else if (dump) {
    std::printf("{\"tally\":[");
    for (size_t k = 0; k < width; k++) std::printf("%s%.17g", k ? "," : "", tally[k]);
    std::printf("],\"tally_sq\":[");
    for (size_t k = 0; k < width; k++) std::printf("%s%.17g", k ? "," : "", tally_sq[k]);
    std::printf("],\"blocks\":%zu}\n", n_blocks);
  }
  return 0;
}
