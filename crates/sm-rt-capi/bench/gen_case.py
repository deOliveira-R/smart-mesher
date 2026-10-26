#!/usr/bin/env python3
"""Deterministic benchmark cases for the DAGMC comparison (stdlib only).

Writes <out>/<case>/facets.bin ("smfacets v1"), <out>/<case>/rays.bin ("smrays v1") and
<out>/<case>/manifest.json (counts and SHA-256 of both files) for the cases

  C1  icosphere shell (radius 1, subdivision 5) in the box [-2,2]^3
  C2  five nested icospheres (radii 0.2 .. 1.0, subdivision 4) in the box [-2,2]^3
  C3  box [-2,2]^3 with a spherical void (radius 1, subdivision 5)

Formats are defined in bench/README.md. Only IEEE-exact operations (+ - * / sqrt) are used, so
the bytes are identical on every platform. Rays are drawn with splitmix64 seeded from the case
name (FNV-1a 64 of the UTF-8 bytes): uniform interior starts, isotropic directions by rejection
sampling in the unit ball.
"""
import argparse
import hashlib
import json
import math
import os
import struct
import sys

CASES = ("C1", "C2", "C3")
HALF = 2.0  # box half-width
DEFAULT_RAYS = 200_000
MASK = (1 << 64) - 1


def fnv1a64(s):
    h = 0xCBF29CE484222325
    for b in s.encode("utf-8"):
        h = ((h ^ b) * 0x100000001B3) & MASK
    return h


class SplitMix64:
    def __init__(self, seed):
        self.s = seed & MASK

    def next_u64(self):
        self.s = (self.s + 0x9E3779B97F4A7C15) & MASK
        z = self.s
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK
        return z ^ (z >> 31)

    def unit(self):
        return (self.next_u64() >> 11) * (1.0 / 9007199254740992.0)

    def direction(self):
        """Isotropic unit vector: rejection in the unit ball, then normalise."""
        while True:
            x, y, z = (2.0 * self.unit() - 1.0 for _ in range(3))
            n2 = x * x + y * y + z * z
            if 1e-6 < n2 <= 1.0:
                n = math.sqrt(n2)
                return x / n, y / n, z / n


def icosphere(radius, subdivisions):
    """Closed triangle surface, outward normals. Returns (vertices, triangles)."""
    t = (1.0 + math.sqrt(5.0)) / 2.0
    raw = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]

    def unit(p):
        n = math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2])
        return (p[0] / n, p[1] / n, p[2] / n)

    verts = [unit(p) for p in raw]
    tris = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
            (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    for _ in range(subdivisions):
        cache = {}

        def mid(a, b):
            key = (a, b) if a < b else (b, a)
            if key not in cache:
                m = ((verts[a][0] + verts[b][0]) / 2.0, (verts[a][1] + verts[b][1]) / 2.0, (verts[a][2] + verts[b][2]) / 2.0)
                verts.append(unit(m))
                cache[key] = len(verts) - 1
            return cache[key]

        new = []
        for a, b, c in tris:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            new += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        tris = new
    verts = [(radius * x, radius * y, radius * z) for x, y, z in verts]
    # Outward check: the right-hand normal of every triangle points away from the centre.
    for a, b, c in tris:
        pa, pb, pc = verts[a], verts[b], verts[c]
        u = (pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2])
        v = (pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2])
        n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        assert n[0] * pa[0] + n[1] * pa[1] + n[2] * pa[2] > 0.0, "icosphere triangle is not outward"
    return verts, tris


def inscribed_radius(verts, tris):
    """Smallest distance from the origin to a triangle plane: the faceted sphere contains the ball
    of this radius, and every vertex lies on the true sphere."""
    best = float("inf")
    for a, b, c in tris:
        pa, pb, pc = verts[a], verts[b], verts[c]
        u = (pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2])
        v = (pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2])
        n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        norm = math.sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2])
        best = min(best, abs(n[0] * pa[0] + n[1] * pa[1] + n[2] * pa[2]) / norm)
    return best


def box(half):
    """Axis-aligned box, 12 triangles, outward normals."""
    h = half
    verts = [(-h, -h, -h), (h, -h, -h), (h, h, -h), (-h, h, -h), (-h, -h, h), (h, -h, h), (h, h, h), (-h, h, h)]
    tris = [(0, 3, 2), (0, 2, 1), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (2, 3, 7), (2, 7, 6), (1, 2, 6), (1, 6, 5), (3, 0, 4), (3, 4, 7)]
    return verts, tris


class Soup:
    """Triangle soup with per-triangle (forward, reverse) regions; -1 is the implicit complement."""

    def __init__(self):
        self.verts, self.tris, self.sides = [], [], []

    def add(self, verts, tris, forward, reverse, flip=False):
        base = len(self.verts)
        self.verts += verts
        for a, b, c in tris:
            self.tris.append((base + a, base + c, base + b) if flip else (base + a, base + b, base + c))
            self.sides.append((forward, reverse))


def build_case(name):
    soup = Soup()
    if name == "C1":
        v, t = icosphere(1.0, 5)
        soup.add(v, t, 0, 1)  # ball inside, shell region outside
        inradii = [inscribed_radius(v, t)]
        v, t = box(HALF)
        soup.add(v, t, 1, -1)
        radii = [1.0]
    elif name == "C2":
        radii = [0.2, 0.4, 0.6, 0.8, 1.0]
        inradii = []
        for k, r in enumerate(radii):
            v, t = icosphere(r, 4)
            soup.add(v, t, k, k + 1)
            inradii.append(inscribed_radius(v, t))
        v, t = box(HALF)
        soup.add(v, t, len(radii), -1)
    elif name == "C3":
        v, t = box(HALF)
        soup.add(v, t, 0, -1)
        v, t = icosphere(1.0, 5)
        soup.add(v, t, 0, -1, flip=True)  # normal points into the void, out of the solid
        inradii = [inscribed_radius(v, t)]
        radii = [1.0]
    else:
        raise SystemExit(f"unknown case {name!r}")
    return soup, radii, inradii


def start_region(name, radii, inradii, p):
    """Region of a start point, or None if it is not a valid start.

    The spheres are icospheres: inscribed polyhedra whose vertices lie on the true sphere, so the
    facets lie up to a sagitta inside it. A point with inradius < |p| < radius is inside the
    analytic ball but outside the faceted one, and its region would depend on the model. Such
    points are rejected (the band is ~1e-3 of the radius), which makes every label exact for both
    the faceted model and the analytic ball."""
    r = math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2])
    if any(rin < r < rad for rin, rad in zip(inradii, radii)):
        return None
    if name == "C3":
        return 0 if r >= radii[0] else None  # inside the void: not a start
    return sum(1 for q in radii if r >= q)


def make_rays(name, radii, inradii, n):
    rng = SplitMix64(fnv1a64(name))
    out, regions = [], []
    while len(regions) < n:
        p = tuple((2.0 * rng.unit() - 1.0) * HALF for _ in range(3))
        d = rng.direction()
        reg = start_region(name, radii, inradii, p)
        if reg is None:
            continue
        out.append(p + d)
        regions.append(reg)
    return out, regions


def write_case(name, outdir, n_rays):
    soup, radii, inradii = build_case(name)
    d = os.path.join(outdir, name)
    os.makedirs(d, exist_ok=True)
    fpath, rpath = os.path.join(d, "facets.bin"), os.path.join(d, "rays.bin")
    with open(fpath, "wb") as f:
        f.write(struct.pack("<II", len(soup.verts), len(soup.tris)))
        for v in soup.verts:
            f.write(struct.pack("<ddd", *v))
        for t in soup.tris:
            f.write(struct.pack("<III", *t))
        for s in soup.sides:
            f.write(struct.pack("<ii", *s))
    rays, regions = make_rays(name, radii, inradii, n_rays)
    with open(rpath, "wb") as f:
        f.write(struct.pack("<I", len(rays)))
        for r in rays:
            f.write(struct.pack("<dddddd", *r))
        for g in regions:
            f.write(struct.pack("<i", g))
    sha = lambda p: hashlib.sha256(open(p, "rb").read()).hexdigest()
    manifest = {"case": name, "vertices": len(soup.verts), "triangles": len(soup.tris), "rays": len(rays),
                "facets_sha256": sha(fpath), "rays_sha256": sha(rpath)}
    with open(os.path.join(d, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
        f.write("\n")
    return manifest


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="output directory (one subdirectory per case)")
    ap.add_argument("--cases", default=",".join(CASES))
    ap.add_argument("--rays", type=int, default=DEFAULT_RAYS)
    a = ap.parse_args()
    for name in a.cases.split(","):
        m = write_case(name, a.out, a.rays)
        print(f"{name} facets.bin {m['facets_sha256']}  rays.bin {m['rays_sha256']}  ({m['triangles']} triangles, {m['rays']} rays)")


if __name__ == "__main__":
    sys.exit(main())
