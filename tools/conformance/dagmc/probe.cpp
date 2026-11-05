// DAGMC probe for the DAGMC export conformance (Tier B): loads an .h5m, builds the OBB tree and
// answers point_in_volume and ray_fire queries read from stdin, printing one line per answer.
//
//   probe FILE.h5m < queries.txt
//
// queries.txt: "P n" then n lines "x y z", then "R m" then m lines "volume_id x y z u v w".
// Output: COUNTS surfaces volumes / VOL id material / PIV point volume_id 0|1 /
//         RAY ray volume_id surface_id distance (surface 0, distance -1: no hit).
// Licensed like the rest of the repository (MIT); links DAGMC and MOAB (not copied).
#include <cstdio>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "DagMC.hpp"

using namespace moab;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: probe FILE.h5m < queries\n");
    return 2;
  }
  auto dag = std::make_shared<DagMC>();
  ErrorCode rv = dag->load_file(argv[1]);
  if (rv != MB_SUCCESS) {
    std::printf("ERROR load_file %d\n", static_cast<int>(rv));
    return 3;
  }
  rv = dag->init_OBBTree();
  if (rv != MB_SUCCESS) {
    std::printf("ERROR init_OBBTree %d\n", static_cast<int>(rv));
    return 4;
  }
  std::vector<std::string> keywords = {"mat"};
  std::map<std::string, std::string> synonyms;
  // `mat:Name` groups: the colon is the delimiter (DAGMC's default is "_").
  rv = dag->parse_properties(keywords, synonyms, ":");
  if (rv != MB_SUCCESS) {
    std::printf("ERROR parse_properties %d\n", static_cast<int>(rv));
    return 5;
  }
  const unsigned nv = dag->num_entities(3);
  const unsigned ns = dag->num_entities(2);
  std::printf("COUNTS %u %u\n", ns, nv);
  std::map<int, EntityHandle> vol;
  for (unsigned i = 1; i <= nv; ++i) {
    EntityHandle h = dag->entity_by_index(3, i);
    int id = dag->id_by_index(3, i);
    std::string mat = "-";
    if (dag->has_prop(h, "mat")) dag->prop_value(h, "mat", mat);
    std::printf("VOL %d %s\n", id, mat.c_str());
    vol[id] = h;
  }
  for (unsigned i = 1; i <= ns; ++i) std::printf("SURF %d\n", dag->id_by_index(2, i));
  std::string tag;
  while (std::cin >> tag) {
    if (tag == "P") {
      int n;
      std::cin >> n;
      for (int k = 0; k < n; ++k) {
        double p[3];
        std::cin >> p[0] >> p[1] >> p[2];
        for (auto& kv : vol) {
          int inside = -1;
          // A fixed direction keeps the answer a function of the file alone.
          const double uvw[3] = {0.5773502691896258, 0.5773502691896258, 0.5773502691896258};
          rv = dag->point_in_volume(kv.second, p, inside, uvw);
          std::printf("PIV %d %d %d\n", k, kv.first, rv == MB_SUCCESS ? inside : -1);
        }
      }
    } else if (tag == "R") {
      int m;
      std::cin >> m;
      for (int k = 0; k < m; ++k) {
        int id;
        double o[3], d[3];
        std::cin >> id >> o[0] >> o[1] >> o[2] >> d[0] >> d[1] >> d[2];
        EntityHandle next = 0;
        double dist = -1;
        auto it = vol.find(id);
        if (it == vol.end()) {
          std::printf("RAY %d %d 0 -1\n", k, id);
          continue;
        }
        rv = dag->ray_fire(it->second, o, d, next, dist);
        int sid = 0;
        if (rv == MB_SUCCESS && next != 0) sid = dag->id_by_index(2, dag->index_by_handle(next));
        else dist = -1;
        std::printf("RAY %d %d %d %.17g\n", k, id, sid, dist);
      }
    }
  }
  return 0;
}
