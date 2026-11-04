#!/usr/bin/env python3
"""Validate the concept graph and render it as Mermaid.

Usage: tools/graph.py [--domain NAME | --stages]
  no arg: domain-level view; --domain: zoom into one domain; --stages: workflow view (docs/graph/stages.yaml).
Checks: every edge endpoint exists, every node has a known domain, the graph is acyclic,
every node sits in exactly one stage group or span. Stage-order findings are warnings (stderr):
a dependency on a node from a later stage, and a span that depends on stage nodes.
"""
import sys, pathlib, re

try:
    import yaml
except ImportError:
    sys.exit("needs PyYAML: pip install pyyaml")

ROOT = pathlib.Path(__file__).resolve().parents[1]
g = yaml.safe_load((ROOT / "docs/graph/concepts.yaml").read_text())
doms, nodes = g["domains"], g["nodes"]
errors = []

def expand(edges, universe):
    out = []
    for e in edges:
        a, b = e[0], e[1]
        label = e[2] if len(e) > 2 else ""
        targets = [n for n in universe if n != a] if b == "*" else [b]
        for t in targets:
            for x in (a, t):
                if x not in universe:
                    errors.append(f"unknown endpoint {x!r} in edge {e}")
            out.append((a, t, label))
    return out

for n, d in nodes.items():
    if d["domain"] not in doms:
        errors.append(f"node {n} has unknown domain {d['domain']}")

node_edges = expand([e for e in g["edges"] if e[1] != "*"], nodes)
dom_edges = expand([e for e in g["domain_edges"] if e[1] != "*"], doms)

def find_cycle(vertices, edges):
    adj = {v: [] for v in vertices}
    for a, b, _ in edges:
        adj[a].append(b)
    state = {}
    def dfs(v, path):
        state[v] = 1
        for w in adj[v]:
            if state.get(w) == 1:
                return path + [v, w]
            if w not in state:
                c = dfs(w, path + [v])
                if c:
                    return c
        state[v] = 2
    for v in vertices:
        if v not in state:
            c = dfs(v, [])
            if c:
                return c

for name, vs, es in (("node", nodes, node_edges), ("domain", doms, dom_edges)):
    c = find_cycle(vs, es)
    if c:
        errors.append(f"{name} graph cycle: {' -> '.join(c)}")

# node edges must respect domain edges (cross-domain edge needs a domain edge)
allowed = {(a, b) for a, b, _ in expand(g["domain_edges"], doms)}
for a, b, _ in node_edges:
    da, db = nodes[a]["domain"], nodes[b]["domain"]
    if da != db and db != "foundation" and (da, db) not in allowed:  # foundation is usable by all
        errors.append(f"edge {a}->{b} crosses {da}->{db}, which has no domain edge")

# ---- workflow view: stages.yaml places every node in one stage group or one span
st = yaml.safe_load((ROOT / "docs/graph/stages.yaml").read_text())
stage_of, group_of, warnings = {}, {}, []  # stage_of: stage index, or None for a span
for sid, sp in st["spans"].items():
    for n in sp["nodes"]:
        if n in group_of:
            errors.append(f"stages: {n} placed twice ({group_of[n]}, {sid})")
        stage_of[n], group_of[n] = None, sid
for i, stg in enumerate(st["stages"]):
    for gid, gr in stg["groups"].items():
        for n in gr["nodes"]:
            if n in group_of:
                errors.append(f"stages: {n} placed twice ({group_of[n]}, {gid})")
            stage_of[n], group_of[n] = i, gid
for n in group_of:
    if n not in nodes:
        errors.append(f"stages: unknown node {n}")
for n in nodes:
    if n not in group_of:
        errors.append(f"stages: node {n} is not placed")
stage_ids = [s_["id"] for s_ in st["stages"]]
back = set()
for a, b, lab in node_edges:
    if a not in stage_of or b not in stage_of:
        continue
    sa, sb = stage_of[a], stage_of[b]
    if sa is not None and sb is not None and sb > sa:
        back.add((a, b))
        warnings.append(f"stage order: {a} ({stage_ids[sa]}) uses {b} ({stage_ids[sb]}) via {lab}")
    elif sa is None and sb is not None:
        warnings.append(f"span {group_of[a]}: {a} uses stage node {b} ({stage_ids[sb]}) via {lab}")

if errors:
    print("\n".join("ERROR: " + e for e in errors))
    sys.exit(1)

def mid(s):
    return re.sub(r"[^A-Za-z0-9_]", "_", s)

for w in warnings:
    print("WARNING: " + w, file=sys.stderr)

if "--stages" in sys.argv:
    print("graph LR")
    for i, stg in enumerate(st["stages"]):
        print(f'  subgraph {mid(stg["id"])}["{i + 1}. {stg["title"]}: {stg["makes"]}"]')
        for gid, gr in stg["groups"].items():
            print(f'    {mid("g_" + gid)}["{gr["title"]}<br/><small>{", ".join(gr["nodes"])}</small>"]')
        print("  end")
    for sid, sp in st["spans"].items():
        print(f'  {mid("s_" + sid)}[/"{sp["title"]} (all stages)<br/><small>{", ".join(sp["nodes"])}</small>"/]')
    seen, back_groups = set(), {(group_of[a], group_of[b]) for a, b in back}
    for a, b, _ in node_edges:  # group-level edges; backward ones dashed
        ga, gb = group_of[a], group_of[b]
        if ga == gb or stage_of[a] is None or stage_of[b] is None or (ga, gb) in seen:
            continue
        seen.add((ga, gb))
        arrow = "-.->" if (ga, gb) in back_groups else "-->"
        print(f"  {mid('g_' + ga)} {arrow} {mid('g_' + gb)}")
elif "--domain" in sys.argv:
    dom = sys.argv[sys.argv.index("--domain") + 1]
    inside = {n for n, d in nodes.items() if d["domain"] == dom}
    print("graph BT")
    for n in sorted(inside):
        print(f'  {mid(n)}["{n}<br/><small>{nodes[n]["summary"]}</small>"]')
    for a, b, lab in node_edges:
        if a in inside or b in inside:
            for x in (a, b):
                if x not in inside:
                    print(f'  {mid(x)}(["{x} ({nodes[x]["domain"]})"])')
            print(f'  {mid(a)} -->|{lab}| {mid(b)}')
else:
    print("graph BT")
    for d, v in doms.items():
        layers = ",".join(v.get("layers", []))
        print(f'  {mid(d)}["{d} {("[" + layers + "]") if layers else ""}<br/><small>{v["summary"]}</small>"]')
    for a, b, _ in dom_edges:
        print(f"  {mid(a)} --> {mid(b)}")
print(f"%% ok: {len(nodes)} nodes, {len(node_edges)} edges, {len(doms)} domains, {len(st['stages'])} stages, {len(warnings)} stage warnings", file=sys.stderr)
