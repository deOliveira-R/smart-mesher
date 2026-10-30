#!/usr/bin/env python3
"""Join docs/graph/stages.yaml with ontology.json into stages.json; check placement; list ordering conflicts."""
import json, sys, pathlib, yaml
REPO = pathlib.Path(sys.argv[1]); ONT = json.load(open(sys.argv[2])); OUT = pathlib.Path(sys.argv[3])
s = yaml.safe_load((REPO / "docs/graph/stages.yaml").read_text())
concepts = ONT["concepts"]
place = {}  # node -> (stage index or None, group id)
groups = []
for sid, sp in s["spans"].items():
    groups.append(dict(id=sid, stage=None, title=sp["title"], note=sp.get("role", ""), nodes=sp["nodes"]))
stages = []
for i, st in enumerate(s["stages"]):
    gids = []
    for gid, g in st["groups"].items():
        groups.append(dict(id=gid, stage=i, title=g["title"], note=g.get("note", ""), nodes=g["nodes"])); gids.append(gid)
    stages.append(dict(id=st["id"], title=st["title"], makes=st["makes"], groups=gids))
for g in groups:
    for n in g["nodes"]:
        assert n in concepts, f"unknown node {n}"
        assert n not in place, f"{n} placed twice"
        place[n] = (g["stage"], g["id"])
missing = set(concepts) - set(place)
assert not missing, f"unplaced: {missing}"
edges, conflicts = [], []
for e in ONT["concept_edges"]:
    user, prov = e["s"], e["t"]
    su, sp_ = place[user][0], place[prov][0]
    back = su is not None and sp_ is not None and sp_ > su
    edges.append(dict(user=user, prov=prov, label=e["label"], back=back))
    if back:
        conflicts.append(dict(user=user, prov=prov, label=e["label"],
                              ustage=s["stages"][su]["title"], pstage=s["stages"][sp_]["title"]))
for g in groups:
    g["crates"] = sorted({c for n in g["nodes"] for c in concepts[n]["crates"]})
for st in stages:
    ns = [n for g in groups if g["id"] in st["groups"] for n in g["nodes"]]
    st["n_nodes"] = len(ns)
    st["n_crates"] = len({c for n in ns for c in concepts[n]["crates"]})
    st["n_built"] = sum(concepts[n]["status"] == "built" for n in ns)
crates = {k: dict(description=v["description"], loc=v["loc"], kind=v["kind"]) for k, v in ONT["crates"].items()}
OUT.write_text(json.dumps(dict(stages=stages, groups=groups, concepts=concepts, edges=edges,
                               conflicts=conflicts, crates=crates, commit=ONT["commit"])))
print([(st["title"], st["n_nodes"], st["n_crates"]) for st in stages])
print("conflicts:", [(c["user"], c["prov"]) for c in conflicts])
