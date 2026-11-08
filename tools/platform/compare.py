#!/usr/bin/env python3
"""Compare two hashes.sh outputs (`script|key|value` lines).

usage: compare.py A.txt B.txt [--ignore-prefix P ...]

Prints, per script, whether it is equal, and for a difference the first differing key in
timeline order (node.NNN.* first, then branches, layers, state), with the layer that moved and
the first differing node (the bisect point: its op is in `smesh log`). A key present on one side
only is a difference. Exit 1 when anything differs, 0 when all equal.
"""
import sys
from collections import OrderedDict


def load(p):
    d = OrderedDict()
    for ln in open(p):
        ln = ln.rstrip("\n")
        if not ln:
            continue
        s, k, v = ln.split("|", 2)
        d.setdefault(s, {})[k] = v
    return d


def order(k):
    if k.startswith("node."):
        return (0, k)
    if k.startswith("branch."):
        return (1, k)
    if k.startswith("layer."):
        return (2, k)
    return (3, k)


def main():
    args = sys.argv[1:]
    ignore = []
    while "--ignore-prefix" in args:
        i = args.index("--ignore-prefix")
        ignore.append(args[i + 1])
        del args[i:i + 2]
    a, b = load(args[0]), load(args[1])
    bad = 0
    for s in sorted(set(a) | set(b)):
        da, db = a.get(s), b.get(s)
        if da is None or db is None:
            print(f"{s}: DIFFERS (script missing on {'A' if da is None else 'B'})")
            bad += 1
            continue
        keys = [k for k in set(da) | set(db) if not any(k.startswith(p) for p in ignore)]
        diff = sorted((k for k in keys if da.get(k) != db.get(k)), key=order)
        if not diff:
            print(f"{s}: equal ({len(keys)} keys)")
            continue
        bad += 1
        print(f"{s}: DIFFERS ({len(diff)} of {len(keys)} keys)")
        print(f"  first differing key: {diff[0]}\n    A {da.get(diff[0])}\n    B {db.get(diff[0])}")
        layers = [k[6:] for k in diff if k.startswith("layer.")]
        if layers:
            print(f"  layers that differ: {', '.join(layers)}")
        nodes = sorted(k for k in diff if k.startswith("node.") and k.endswith(".state"))
        if nodes:
            i = nodes[0].split(".")[1]
            print(f"  first differing node: #{i} id A {da.get(f'node.{i}.id')} (see `smesh log` for its op)")
        for k in diff[1:12]:
            print(f"  also: {k}")
    print("RESULT: " + ("all equal" if not bad else f"{bad} script(s) differ"))
    sys.exit(1 if bad else 0)


main()
