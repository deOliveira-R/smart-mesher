#!/usr/bin/env python3
"""Validate the concept graph and render it as Mermaid.

Usage: tools/graph.py [--domain NAME]   # no arg: domain-level view; --domain: zoom into one domain
Checks: every edge endpoint exists, every node has a known domain, the graph is acyclic.
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

if errors:
    print("\n".join("ERROR: " + e for e in errors))
    sys.exit(1)

def mid(s):
    return re.sub(r"[^A-Za-z0-9_]", "_", s)

if "--domain" in sys.argv:
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
print(f"%% ok: {len(nodes)} nodes, {len(node_edges)} edges, {len(doms)} domains", file=sys.stderr)
