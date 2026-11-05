// facets.bin ("smfacets v1", see bench/README.md) -> DAGMC h5m through MOAB's C++ API.
//
//   facets_to_h5m facets.bin out.h5m [--report report.json] [--swap-sense]
//
// (Replaces the pymoab script: conda-forge has no pymoab build on linux-aarch64.)
// Triangles with the same (forward, reverse) region pair that are connected through shared
// vertices form one surface. Each region becomes a volume set; each surface set carries the
// GEOM_SENSE_2 tag [forward volume, reverse volume] (0 = none, the implicit complement), where
// "forward" is the volume the triangle normals point OUT of (the same rule as the smfacets files).
// Then `check_watertight` (DAGMC's make_watertight tool chain) is run on the file and its verdict
// is stored in the --report JSON; the exit code is non-zero if the model is not watertight.
#include <sys/wait.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "moab/Core.hpp"

using namespace moab;

#define CHECK(x) do { ErrorCode e_ = (x); if (e_ != MB_SUCCESS) { \
  std::fprintf(stderr, "MOAB error %d at line %d: %s\n", (int)e_, __LINE__, #x); std::exit(4); } } while (0)

static std::vector<int> parent;
static int find(int x) {
  while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
  return x;
}

static void set_name(Interface* mb, Tag t, EntityHandle s, const char* text) {
  char buf[32] = {0};
  std::strncpy(buf, text, 31);
  CHECK(mb->tag_set_data(t, &s, 1, buf));
}

static std::string json_escape(const std::string& s) {
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n";
    else if (c == '\t') o += "\\t";
    else if (static_cast<unsigned char>(c) >= 0x20) o += c;
  }
  return o;
}

int main(int argc, char** argv) {
  if (argc < 3) { std::fprintf(stderr, "usage: facets_to_h5m facets.bin out.h5m [--report f] [--swap-sense]\n"); return 2; }
  const char* facets = argv[1];
  const char* h5m = argv[2];
  const char* report = nullptr;
  bool swap = false;
  for (int i = 3; i < argc; i++) {
    if (!std::strcmp(argv[i], "--report") && i + 1 < argc) report = argv[++i];
    else if (!std::strcmp(argv[i], "--swap-sense")) swap = true;
    else { std::fprintf(stderr, "unknown argument %s\n", argv[i]); return 2; }
  }

  std::ifstream f(facets, std::ios::binary);
  if (!f) { std::fprintf(stderr, "cannot open %s\n", facets); return 2; }
  uint32_t nv = 0, nt = 0;
  f.read(reinterpret_cast<char*>(&nv), 4);
  f.read(reinterpret_cast<char*>(&nt), 4);
  std::vector<double> verts(static_cast<size_t>(nv) * 3);
  std::vector<uint32_t> tris(static_cast<size_t>(nt) * 3);
  std::vector<int32_t> sides(static_cast<size_t>(nt) * 2);
  f.read(reinterpret_cast<char*>(verts.data()), verts.size() * 8);
  f.read(reinterpret_cast<char*>(tris.data()), tris.size() * 4);
  f.read(reinterpret_cast<char*>(sides.data()), sides.size() * 4);
  if (!f || f.peek() != EOF) { std::fprintf(stderr, "%s: truncated or trailing bytes\n", facets); return 2; }

  Core core;
  Interface* mb = &core;
  EntityHandle root = mb->get_root_set();
  Tag t_geom, t_gid, t_cat, t_name, t_sense, t_tol;
  CHECK(mb->tag_get_handle("GEOM_DIMENSION", 1, MB_TYPE_INTEGER, t_geom, MB_TAG_SPARSE | MB_TAG_CREAT));
  CHECK(mb->tag_get_handle("GLOBAL_ID", 1, MB_TYPE_INTEGER, t_gid, MB_TAG_DENSE | MB_TAG_CREAT));
  CHECK(mb->tag_get_handle("CATEGORY", 32, MB_TYPE_OPAQUE, t_cat, MB_TAG_SPARSE | MB_TAG_CREAT));
  CHECK(mb->tag_get_handle("NAME", 32, MB_TYPE_OPAQUE, t_name, MB_TAG_SPARSE | MB_TAG_CREAT));
  CHECK(mb->tag_get_handle("GEOM_SENSE_2", 2, MB_TYPE_HANDLE, t_sense, MB_TAG_SPARSE | MB_TAG_CREAT));
  CHECK(mb->tag_get_handle("FACETING_TOL", 1, MB_TYPE_DOUBLE, t_tol, MB_TAG_SPARSE | MB_TAG_CREAT));
  double tol = 1e-4;
  CHECK(mb->tag_set_data(t_tol, &root, 1, &tol));

  std::vector<EntityHandle> vh(nv), th(nt);
  for (uint32_t i = 0; i < nv; i++) CHECK(mb->create_vertex(&verts[3 * i], vh[i]));
  for (uint32_t i = 0; i < nt; i++) {
    EntityHandle c[3] = {vh[tris[3 * i]], vh[tris[3 * i + 1]], vh[tris[3 * i + 2]]};
    CHECK(mb->create_element(MBTRI, c, 3, th[i]));
  }

  std::set<int> regions;
  for (int32_t r : sides) if (r >= 0) regions.insert(r);
  std::map<int, EntityHandle> vols;
  for (int r : regions) {
    EntityHandle s;
    CHECK(mb->create_meshset(MESHSET_SET, s));
    int dim = 3, gid = r + 1;
    CHECK(mb->tag_set_data(t_geom, &s, 1, &dim));
    CHECK(mb->tag_set_data(t_gid, &s, 1, &gid));
    set_name(mb, t_cat, s, "Volume");
    set_name(mb, t_name, s, ("region" + std::to_string(r)).c_str());
    vols[r] = s;
  }

  // Surfaces: (forward, reverse) key x connected component over shared vertices.
  parent.resize(nv);
  for (uint32_t i = 0; i < nv; i++) parent[i] = static_cast<int>(i);
  for (uint32_t i = 0; i < nt; i++) {
    int a = find(static_cast<int>(tris[3 * i])), b = find(static_cast<int>(tris[3 * i + 1]));
    parent[b] = a;
    parent[find(static_cast<int>(tris[3 * i + 2]))] = find(a);
  }
  std::map<std::pair<std::pair<int, int>, int>, std::vector<uint32_t>> groups;
  for (uint32_t i = 0; i < nt; i++)
    groups[{{sides[2 * i], sides[2 * i + 1]}, find(static_cast<int>(tris[3 * i]))}].push_back(i);
  std::vector<std::pair<std::pair<int, int>, const std::vector<uint32_t>*>> surfaces;
  for (auto& kv : groups) surfaces.push_back({kv.first.first, &kv.second});
  std::sort(surfaces.begin(), surfaces.end(), [](const auto& a, const auto& b) { return (*a.second)[0] < (*b.second)[0]; });

  int gid = 0;
  for (auto& sf : surfaces) {
    ++gid;
    int fwd = sf.first.first, rev = sf.first.second;
    EntityHandle s;
    CHECK(mb->create_meshset(MESHSET_SET, s));
    int dim = 2;
    CHECK(mb->tag_set_data(t_geom, &s, 1, &dim));
    CHECK(mb->tag_set_data(t_gid, &s, 1, &gid));
    set_name(mb, t_cat, s, "Surface");
    std::vector<EntityHandle> members;
    for (uint32_t i : *sf.second) members.push_back(th[i]);
    CHECK(mb->add_entities(s, members.data(), static_cast<int>(members.size())));
    EntityHandle sense[2] = {fwd >= 0 ? vols[fwd] : 0, rev >= 0 ? vols[rev] : 0};
    if (swap) std::swap(sense[0], sense[1]);
    CHECK(mb->tag_set_data(t_sense, &s, 1, sense));
    std::set<int> vs = {fwd, rev};
    for (int v : vs) if (v >= 0) CHECK(mb->add_parent_child(vols[v], s));
  }
  CHECK(mb->write_file(h5m));

  // DAGMC's own watertightness check (the make_watertight tool chain).
  std::string cmd = std::string("check_watertight '") + h5m + "' 2>&1";
  std::string text;
  int rc = -1;
  if (FILE* p = popen(cmd.c_str(), "r")) {
    char buf[4096];
    size_t n;
    while ((n = fread(buf, 1, sizeof buf, p)) > 0) text.append(buf, n);
    int st = pclose(p);
    rc = (st != -1 && WIFEXITED(st)) ? WEXITSTATUS(st) : -1;
  }
  std::vector<std::string> lines;
  size_t pos = 0;
  while (pos < text.size()) {
    size_t e = text.find('\n', pos);
    if (e == std::string::npos) e = text.size();
    std::string l = text.substr(pos, e - pos);
    if (!l.empty()) lines.push_back(l);
    pos = e + 1;
  }
  std::string lower = text;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return std::tolower(c); });
  bool watertight = rc == 0 && lower.find("not watertight") == std::string::npos;
  std::string tail;
  for (size_t i = lines.size() > 6 ? lines.size() - 6 : 0; i < lines.size(); i++)
    tail += std::string(tail.empty() ? "" : ", ") + "\"" + json_escape(lines[i]) + "\"";
  std::string json = "{\"check_watertight_exit\": " + std::to_string(rc) + ", \"surfaces\": " + std::to_string(surfaces.size()) +
                     ", \"triangles\": " + std::to_string(nt) + ", \"vertices\": " + std::to_string(nv) +
                     ", \"volumes\": " + std::to_string(regions.size()) + ", \"watertight\": " + (watertight ? "true" : "false") +
                     ", \"facets\": \"" + json_escape(facets) + "\", \"check_watertight_tail\": [" + tail + "]}";
  if (report) { std::ofstream(report) << json << "\n"; }
  std::printf("%s\n", json.c_str());
  return watertight ? 0 : 3;
}
