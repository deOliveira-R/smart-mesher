// tets.bin ("smtets v1", formats.rs `Tets`) -> MOAB h5m for XDG's element walk.
//
//   smtets_to_moab tets.bin out.h5m [--report report.json]
//
// The tets are written in file order, so MOAB's element i is tet i. A `REGION` integer tag per tet
// carries the region of tets.bin. No volume or surface sets are written: XDG loads a bare tet file
// as one volume (its boundary skin is the surface), and the walk maps element ids back to regions
// through tets.bin. Per-region volume sets and sense tags are a second step that only an XDG fire
// row on tet input would need.
//
// The file is read back through a fresh MOAB instance and checked: every tet appears once and in
// order with the vertex coordinates of tets.bin bit for bit, the vertex count is the node count,
// and the volume summed over the tets equals the volume of tets.bin to 1e-12 relative. The verdict
// is printed (and stored with --report) as one JSON line; the exit code is 3 when a check fails.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "moab/Core.hpp"

using namespace moab;

#define CHECK(x) do { ErrorCode e_ = (x); if (e_ != MB_SUCCESS) { \
  std::fprintf(stderr, "MOAB error %d at line %d: %s\n", (int)e_, __LINE__, #x); std::exit(4); } } while (0)

static uint64_t bits(double x) { uint64_t b; std::memcpy(&b, &x, 8); return b; }

// Signed volume of the tet (a, b, c, d).
static double tet_volume(const double* a, const double* b, const double* c, const double* d) {
  const double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
  const double v[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
  const double w[3] = {d[0] - a[0], d[1] - a[1], d[2] - a[2]};
  return (u[0] * (v[1] * w[2] - v[2] * w[1]) - u[1] * (v[0] * w[2] - v[2] * w[0]) + u[2] * (v[0] * w[1] - v[1] * w[0])) / 6.0;
}

int main(int argc, char** argv) {
  if (argc < 3) { std::fprintf(stderr, "usage: smtets_to_moab tets.bin out.h5m [--report report.json]\n"); return 2; }
  const char* path = argv[1];
  const char* h5m = argv[2];
  const char* report = nullptr;
  for (int i = 3; i < argc; i++) {
    if (!std::strcmp(argv[i], "--report") && i + 1 < argc) report = argv[++i];
    else { std::fprintf(stderr, "unknown argument %s\n", argv[i]); return 2; }
  }

  std::ifstream f(path, std::ios::binary);
  if (!f) { std::fprintf(stderr, "cannot open %s\n", path); return 2; }
  uint32_t nn = 0, nt = 0;
  f.read(reinterpret_cast<char*>(&nn), 4);
  f.read(reinterpret_cast<char*>(&nt), 4);
  std::vector<double> nodes(static_cast<size_t>(nn) * 3);
  std::vector<uint32_t> tets(static_cast<size_t>(nt) * 4);
  std::vector<int32_t> region(nt);
  f.read(reinterpret_cast<char*>(nodes.data()), nodes.size() * 8);
  f.read(reinterpret_cast<char*>(tets.data()), tets.size() * 4);
  f.read(reinterpret_cast<char*>(region.data()), static_cast<std::streamsize>(nt) * 4);
  if (!f || f.peek() != EOF) { std::fprintf(stderr, "%s: truncated or trailing bytes\n", path); return 2; }
  for (uint32_t v : tets) if (v >= nn) { std::fprintf(stderr, "%s: node index %u out of range\n", path, v); return 2; }

  double volume_in = 0.0;
  for (size_t i = 0; i < nt; i++) {
    const double* p[4];
    for (int a = 0; a < 4; a++) p[a] = &nodes[3 * tets[4 * i + static_cast<size_t>(a)]];
    volume_in += tet_volume(p[0], p[1], p[2], p[3]);
  }

  {
    Core core;
    Interface* mb = &core;
    Tag t_region;
    CHECK(mb->tag_get_handle("REGION", 1, MB_TYPE_INTEGER, t_region, MB_TAG_DENSE | MB_TAG_CREAT));
    std::vector<EntityHandle> vh(nn), th(nt);
    for (uint32_t i = 0; i < nn; i++) CHECK(mb->create_vertex(&nodes[3 * i], vh[i]));
    for (uint32_t i = 0; i < nt; i++) {
      EntityHandle c[4] = {vh[tets[4 * i]], vh[tets[4 * i + 1]], vh[tets[4 * i + 2]], vh[tets[4 * i + 3]]};
      CHECK(mb->create_element(MBTET, c, 4, th[i]));
    }
    if (nt) CHECK(mb->tag_set_data(t_region, th.data(), static_cast<int>(nt), region.data()));
    CHECK(mb->write_file(h5m));
  }

  // Read back through a fresh instance.
  Core back;
  CHECK(back.load_file(h5m));
  Range found, verts;
  CHECK(back.get_entities_by_type(0, MBTET, found));
  CHECK(back.get_entities_by_type(0, MBVERTEX, verts));
  Tag t_region;
  CHECK(back.tag_get_handle("REGION", t_region));
  size_t bad = 0, bad_region = 0;
  double volume_out = 0.0;
  if (found.size() == nt) {
    size_t i = 0;
    for (Range::iterator it = found.begin(); it != found.end(); ++it, ++i) {
      const EntityHandle* conn = nullptr;
      int n = 0;
      CHECK(back.get_connectivity(*it, conn, n));
      double xyz[4][3];
      bool same = n == 4;
      for (int a = 0; a < 4 && same; a++) {
        CHECK(back.get_coords(&conn[a], 1, xyz[a]));
        for (int q = 0; q < 3; q++) same = same && bits(xyz[a][q]) == bits(nodes[3 * tets[4 * i + static_cast<size_t>(a)] + static_cast<size_t>(q)]);
      }
      int r = 0;
      EntityHandle h = *it;
      CHECK(back.tag_get_data(t_region, &h, 1, &r));
      if (r != region[i]) bad_region++;
      if (same) volume_out += tet_volume(xyz[0], xyz[1], xyz[2], xyz[3]); else bad++;
    }
  } else {
    bad = nt;
  }
  const double rel = volume_in != 0.0 ? std::fabs(volume_out - volume_in) / std::fabs(volume_in) : (volume_out == 0.0 ? 0.0 : 1.0);
  const bool ok = found.size() == nt && verts.size() == nn && bad == 0 && bad_region == 0 && rel <= 1e-12;
  char buf[600];
  std::snprintf(buf, sizeof buf,
                "{\"nt\": %u, \"nn\": %u, \"tets_found\": %zu, \"vertices_found\": %zu, \"elements_differing\": %zu, \"regions_differing\": %zu, "
                "\"volume_in\": %.17g, \"volume_out\": %.17g, \"volume_rel_diff\": %.3g, \"ok\": %s}",
                nt, nn, static_cast<size_t>(found.size()), static_cast<size_t>(verts.size()), bad, bad_region, volume_in, volume_out, rel, ok ? "true" : "false");
  if (report) { std::ofstream(report) << buf << "\n"; }
  std::printf("%s\n", buf);
  return ok ? 0 : 3;
}
