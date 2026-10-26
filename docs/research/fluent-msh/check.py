# Minimal ASCII Fluent .msh checker: parses 10/12/13, checks RHR normal -> c0 rule,
# closure (sum of outward area vectors == 0) and positive volume per cell.
import re, sys

class V(tuple):
    def __new__(c, it): return tuple.__new__(c, [float(x) for x in it])
    def __add__(a, b): return V(x+y for x,y in zip(a,b))
    def __sub__(a, b): return V(x-y for x,y in zip(a,b))
    def __neg__(a): return V(-x for x in a)
    def __truediv__(a, k): return V(x/k for x in a)
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def mean(L):
    s = V((0,0,0))
    for p in L: s = s + p
    return s/len(L)
txt = open(sys.argv[1]).read()
pts = {}; faces = []; decl = {}
for m in re.finditer(r'\((10|12|13)\s*\(([^)]*)\)\s*(\(([^()]*)\))?\s*\)', txt):
    idx, hdr, body = m.group(1), m.group(2).split(), m.group(4)
    zone, first, last = (int(h, 16) for h in hdr[:3])
    if zone == 0: decl[idx] = last; continue
    if idx == '10':
        v = [float(x) for x in body.split()]; nd = int(hdr[4], 16)
        for k in range(first, last + 1):
            pts[k] = V(v[(k-first)*nd:(k-first+1)*nd])
    elif idx == '13':
        ft = int(hdr[4], 16)
        for line in body.strip().splitlines():
            t = [int(x, 16) for x in line.split()]
            if ft in (0, 5): n = t[0]; t = t[1:]
            else: n = ft
            assert len(t) == n + 2, line
            faces.append((int(hdr[0],16), t[:n], t[n], t[n+1]))
assert len(pts) == decl['10'] and len(faces) == decl['13']
def newell(ids):
    P = [pts[i] for i in ids]; n = V((0,0,0))
    for a, b in zip(P, P[1:] + P[:1]):
        n = n + V([(a[1]-b[1])*(a[2]+b[2]), (a[2]-b[2])*(a[0]+b[0]), (a[0]-b[0])*(a[1]+b[1])])
    return n / 2
ncell = decl['12']; cnodes = {c: set() for c in range(1, ncell+1)}
for _, ids, c0, c1 in faces:
    for c in (c0, c1):
        if c: cnodes[c].update(ids)
cent = {c: mean([pts[i] for i in s]) for c, s in cnodes.items()}
close = {c: V((0,0,0)) for c in cnodes}; vol = {c: 0.0 for c in cnodes}
ok = True
for z, ids, c0, c1 in faces:
    n = newell(ids); fc = mean([pts[i] for i in ids])
    toward_c0 = dot(n, cent[c0] - fc) > 0
    print(f"zone {z} face {ids} c0={c0} c1={c1} normal={n} points_toward_c0={toward_c0}")
    ok &= toward_c0
    # outward area for c0 is -n, for c1 is +n
    close[c0] = close[c0] - n; vol[c0] += dot(fc, -n) / 3
    if c1: close[c1] = close[c1] + n; vol[c1] += dot(fc, n) / 3
for c in cnodes:
    print(f"cell {c}: closure={close[c]} volume={vol[c]:.6f}")
    ok &= all(abs(x) < 1e-12 for x in close[c]) and vol[c] > 0
print("ALL OK" if ok else "FAIL")
