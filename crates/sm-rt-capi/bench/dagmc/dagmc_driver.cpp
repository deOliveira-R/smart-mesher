// DAGMC reference driver of the sm-rt-capi benchmark (record-only).
//
//   dagmc_driver model.h5m rays.bin [--threads N] [--reps 7] [--warmup 3] [--audit 1000]
//
// Loads the h5m, builds the OBB trees (DagMC::init_OBBTree) and follows every ray of "smrays v1"
// to leakage with the standard loop: ray_fire (orientation 1, with a RayHistory) + next_vol, until
// the ray enters the implicit complement. A particle is counted as lost when
//   * ray_fire or next_vol returns an error,
//   * ray_fire finds no surface from inside a bounded volume, or
//   * the 1-in-`audit` point_in_volume audit disagrees: the start point must be in its start
//     volume, and a point 1e-6 beyond a crossing along the ray must be in the volume next_vol
//     reported. The implicit complement is never audited (it has no bounding surface).
// Prints one JSON object. Rays are processed in index order within contiguous per-thread chunks;
// the counts do not depend on the thread count (each ray is independent).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "DagMC.hpp"

using moab::EntityHandle;
using moab::ErrorCode;

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

struct Counters {
  uint64_t crossings = 0, lost = 0, lost_error = 0, lost_no_hit = 0, lost_audit = 0, audits = 0;
  void add(const Counters& c) {
    crossings += c.crossings; lost += c.lost; lost_error += c.lost_error;
    lost_no_hit += c.lost_no_hit; lost_audit += c.lost_audit; audits += c.audits;
  }
};

static void trace_range(moab::DagMC& dag, const Rays& R, size_t lo, size_t hi, EntityHandle ic,
                        const std::vector<EntityHandle>& vols, unsigned audit_every, Counters& c) {
  for (size_t i = lo; i < hi; i++) {
    double pos[3] = {R.o[3 * i], R.o[3 * i + 1], R.o[3 * i + 2]};
    const double dir[3] = {R.d[3 * i], R.d[3 * i + 1], R.d[3 * i + 2]};
    int32_t r0 = R.region[i];
    EntityHandle vol = r0 < 0 ? ic : vols[static_cast<size_t>(r0)];
    moab::DagMC::RayHistory history;
    // Audit the start point too (the implicit complement has no bounding surface, so DAGMC's
    // point_in_volume is not defined for it and it is never audited).
    if (audit_every && vol != ic && (i % audit_every) == 0) {
      int inside = 0;
      c.audits++;
      if (dag.point_in_volume(vol, pos, inside, dir) != moab::MB_SUCCESS || inside != 1) { c.lost++; c.lost_audit++; continue; }
    }
    for (unsigned step = 0; step < 1000000; step++) {
      EntityHandle surf = 0, next = 0;
      double dist = 0.0;
      if (dag.ray_fire(vol, pos, dir, surf, dist, &history, 0.0, 1) != moab::MB_SUCCESS) {
        c.lost++; c.lost_error++; break;
      }
      if (surf == 0) {
        if (std::getenv("SMRT_DEBUG_AUDIT") && c.lost_no_hit < 12) {
          int in_start = -1;
          dag.point_in_volume(vol, pos, in_start, dir);
          std::fprintf(stderr, "no hit: ray %zu step %u start_region %d |pos|=%.9g point_in_volume(vol, pos)=%d\n", i, step, (int)r0,
                       std::sqrt(pos[0]*pos[0]+pos[1]*pos[1]+pos[2]*pos[2]), in_start);
        }
        c.lost++; c.lost_no_hit++; break;
      }
      if (dag.next_vol(surf, vol, next) != moab::MB_SUCCESS) { c.lost++; c.lost_error++; break; }
      for (int k = 0; k < 3; k++) pos[k] += dist * dir[k];
      c.crossings++;
      if (audit_every && next != ic && ((i + step) % audit_every) == 0) {
        c.audits++;
        const double probe[3] = {pos[0] + 1e-6 * dir[0], pos[1] + 1e-6 * dir[1], pos[2] + 1e-6 * dir[2]};
        int inside = 0;
        if (dag.point_in_volume(next, probe, inside, dir, &history) != moab::MB_SUCCESS || inside != 1) {
          if (std::getenv("SMRT_DEBUG_AUDIT") && c.lost_audit < 5)
            std::fprintf(stderr, "audit fail: ray %zu step %u next=%llu ic=%llu inside=%d probe=(%g %g %g) |p|=%g\n", i, step,
                         (unsigned long long)next, (unsigned long long)ic, inside, probe[0], probe[1], probe[2],
                         std::sqrt(probe[0]*probe[0]+probe[1]*probe[1]+probe[2]*probe[2]));
          c.lost++; c.lost_audit++; break;
        }
      }
      vol = next;
      if (vol == ic) break;  // leaked into the implicit complement
    }
  }
}

static Counters run_all(moab::DagMC& dag, const Rays& R, EntityHandle ic, const std::vector<EntityHandle>& vols,
                        unsigned threads, unsigned audit_every) {
  std::vector<Counters> per(threads);
  std::vector<std::thread> pool;
  const size_t chunk = (R.n + threads - 1) / threads;
  for (unsigned t = 0; t < threads; t++) {
    const size_t lo = std::min<size_t>(R.n, static_cast<size_t>(t) * chunk), hi = std::min<size_t>(R.n, lo + chunk);
    pool.emplace_back([&, t, lo, hi] { trace_range(dag, R, lo, hi, ic, vols, audit_every, per[t]); });
  }
  for (auto& th : pool) th.join();
  Counters total;
  for (auto& c : per) total.add(c);
  return total;
}

static std::string cpu_model() {
  std::ifstream f("/proc/cpuinfo");
  std::string line;
  while (std::getline(f, line)) {
    for (const char* key : {"model name", "Model", "Hardware", "cpu model"}) {
      if (line.rfind(key, 0) == 0) {
        auto p = line.find(':');
        if (p != std::string::npos) {
          std::string v = line.substr(p + 1);
          v.erase(0, v.find_first_not_of(" \t"));
          return v;
        }
      }
    }
  }
  return "unknown";
}

int main(int argc, char** argv) {
  if (argc < 3) { std::fprintf(stderr, "usage: %s model.h5m rays.bin [--threads N] [--reps R] [--warmup W] [--audit K]\n", argv[0]); return 2; }
  unsigned threads = 1, reps = 7, warmup = 3, audit_every = 1000;
  for (int i = 3; i + 1 < argc; i += 2) {
    const std::string k = argv[i];
    const unsigned v = static_cast<unsigned>(std::atoi(argv[i + 1]));
    if (k == "--threads") threads = std::max(1u, v);
    else if (k == "--reps") reps = std::max(1u, v);
    else if (k == "--warmup") warmup = v;
    else if (k == "--audit") audit_every = v;
    else { std::fprintf(stderr, "unknown option %s\n", k.c_str()); return 2; }
  }

  using clk = std::chrono::steady_clock;
  auto secs = [](clk::time_point a, clk::time_point b) { return std::chrono::duration<double>(b - a).count(); };

  auto mb = std::make_shared<moab::Core>();
  moab::DagMC dag(mb);
  const auto t0 = clk::now();
  if (dag.load_file(argv[1]) != moab::MB_SUCCESS) { std::fprintf(stderr, "load_file failed\n"); return 3; }
  if (dag.init_OBBTree() != moab::MB_SUCCESS) { std::fprintf(stderr, "init_OBBTree failed\n"); return 3; }
  const double build_s = secs(t0, clk::now());

  // Volumes by global id (region r has id r + 1) and the implicit complement.
  std::vector<EntityHandle> vols;
  const unsigned nvol = dag.num_entities(3);
  EntityHandle ic = 0;
  if (dag.geom_tool()->get_implicit_complement(ic) != moab::MB_SUCCESS) { std::fprintf(stderr, "no implicit complement\n"); return 3; }
  for (unsigned id = 1; id <= nvol; id++) {
    EntityHandle v = dag.entity_by_id(3, static_cast<int>(id));
    vols.push_back(v);  // the complement has no global id and is not listed here
  }

  const Rays R = read_rays(argv[2]);
  Counters last;
  std::vector<double> rates, crates;
  for (unsigned rep = 0; rep < warmup + reps; rep++) {
    const auto a = clk::now();
    last = run_all(dag, R, ic, vols, threads, audit_every);
    const double s = secs(a, clk::now());
    if (rep >= warmup) { rates.push_back(static_cast<double>(R.n) / s); crates.push_back(static_cast<double>(last.crossings) / s); }
  }
  auto stat = [](std::vector<double> v, double& med, double& mn, double& mx) {
    std::sort(v.begin(), v.end());
    med = v[v.size() / 2]; mn = v.front(); mx = v.back();
  };
  double rm, rmin, rmax, cm, cmin, cmax;
  stat(rates, rm, rmin, rmax);
  stat(crates, cm, cmin, cmax);

  std::printf(
      "{\"tool\":\"dagmc_driver\",\"backend\":\"DAGMC OBB tree (conda-forge dagmc 3.2.4, nodoubledown)\","
      "\"rays\":%zu,\"threads\":%u,\"cpu_model\":\"%s\",\"build_s\":%.6f,\"lost\":%llu,"
      "\"lost_breakdown\":{\"ray_fire_error\":%llu,\"no_hit_inside_bounded_volume\":%llu,\"point_in_volume_audit\":%llu},"
      "\"audits\":%llu,\"crossings\":%llu,"
      "\"rays_per_s\":{\"median\":%.1f,\"min\":%.1f,\"max\":%.1f},"
      "\"crossings_per_s\":{\"median\":%.1f,\"min\":%.1f,\"max\":%.1f},\"warmup\":%u,\"reps\":%u}\n",
      R.n, threads, cpu_model().c_str(), build_s, (unsigned long long)last.lost, (unsigned long long)last.lost_error,
      (unsigned long long)last.lost_no_hit, (unsigned long long)last.lost_audit, (unsigned long long)last.audits,
      (unsigned long long)last.crossings, rm, rmin, rmax, cm, cmin, cmax, warmup, reps);
  return 0;
}
