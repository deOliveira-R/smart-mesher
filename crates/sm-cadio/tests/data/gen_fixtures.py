#!/usr/bin/env python3
"""Generates the hand-written STEP fixtures of sm-cadio (run from this directory).

The checked-in .stp files are the output; this script documents how they were built. Entities
are numbered so that vertices, edges and faces ascend in the order of
`sm_topo_api::fixtures::brick` (so the imported brick hashes like the fixture).
"""
import math
import os

TAU = 2 * math.pi


def fmt(x):
    if x == 0:
        return "0."
    s = repr(float(x))
    if "e" in s:
        m, e = s.split("e")
        if "." not in m:
            m += "."
        return f"{m}E{int(e):+03d}".replace("E+", "E+")
    if s.endswith(".0"):
        return s[:-1]
    return s


def vec(v):
    return "(" + ",".join(fmt(c) for c in v) + ")"


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def ref_for(z):
    """The x axis `Frame::from_z` picks for axis z."""
    z = unit(z)
    if abs(z[0]) <= abs(z[1]) and abs(z[0]) <= abs(z[2]):
        h = (1.0, 0.0, 0.0)
    elif abs(z[1]) <= abs(z[2]):
        h = (0.0, 1.0, 0.0)
    else:
        h = (0.0, 0.0, 1.0)
    d = dot(h, z)
    return unit(tuple(h[i] - z[i] * d for i in range(3)))


class Doc:
    def __init__(self):
        self.ents = {}
        self.n = 0

    def new(self):
        self.n += 1
        return self.n

    def put(self, i, text):
        self.ents[i] = text
        return i

    def add(self, text):
        return self.put(self.new(), text)

    def point(self, p):
        return self.add(f"CARTESIAN_POINT('',{vec(p)})")

    def direction(self, d):
        return self.add(f"DIRECTION('',{vec(d)})")

    def axis(self, origin, z=None, x=None):
        o = self.point(origin)
        zz = self.direction(z) if z is not None else None
        xx = self.direction(x) if x is not None else None
        return self.add(f"AXIS2_PLACEMENT_3D('',#{o},{'#%d' % zz if zz else '$'},{'#%d' % xx if xx else '$'})")

    def text(self):
        return "\n".join(f"#{i} = {t};" for i, t in sorted(self.ents.items()))


def header(schema, name="fixture"):
    return (
        "ISO-10303-21;\nHEADER;\n"
        "FILE_DESCRIPTION(('sm-cadio hand-written fixture'),'2;1');\n"
        f"FILE_NAME('{name}','1970-01-01T00:00:00',('smart-mesh'),(''),'gen_fixtures.py','',' ');\n"
        f"FILE_SCHEMA(('{schema}'));\nENDSEC;\nDATA;\n"
    )


FOOTER = "\nENDSEC;\nEND-ISO-10303-21;\n"

AP214 = "AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }"
AP203 = "CONFIG_CONTROL_DESIGN"


def unit_context(d, length="mm"):
    """The shape representation context with units; returns its id."""
    if length == "mm":
        lu = d.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )")
    elif length == "inch":
        mm = d.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )")
        meas = d.add(f"LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#{mm})")
        dim = d.add("DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.)")
        lu = d.add(f"( CONVERSION_BASED_UNIT('INCH',#{meas}) LENGTH_UNIT() NAMED_UNIT(#{dim}) )")
    elif length is None:
        lu = None
    au = d.add("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )")
    su = d.add("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )")
    if lu is None:
        return d.add("( GEOMETRIC_REPRESENTATION_CONTEXT(3) REPRESENTATION_CONTEXT('Context #1','3D Context') )")
    unc = d.add(f"UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#{lu},'distance_accuracy_value','confusion accuracy')")
    return d.add(
        "( GEOMETRIC_REPRESENTATION_CONTEXT(3) "
        f"GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{lu},#{au},#{su})) "
        "REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') )"
    )


def product(d, ctx, solids, ap203=False):
    """Product structure around the representation `ctx` holding `solids`."""
    ac = d.add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')")
    if ap203:
        d.add(f"APPLICATION_PROTOCOL_DEFINITION('international standard','config_control_design',1994,#{ac})")
    else:
        d.add(f"APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{ac})")
    pc = d.add(f"PRODUCT_CONTEXT('',#{ac},'mechanical')")
    prod = d.add(f"PRODUCT('part','part','',(#{pc}))")
    form = d.add(f"PRODUCT_DEFINITION_FORMATION('','',#{prod})")
    pdc = d.add(f"PRODUCT_DEFINITION_CONTEXT('part definition',#{ac},'design')")
    pd = d.add(f"PRODUCT_DEFINITION('design','',#{form},#{pdc})")
    pds = d.add(f"PRODUCT_DEFINITION_SHAPE('','',#{pd})")
    org = d.point((0.0, 0.0, 0.0))
    origin = d.axis((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
    items = ",".join(f"#{i}" for i in [origin] + solids)
    rep = d.add(f"ADVANCED_BREP_SHAPE_REPRESENTATION('',({items}),#{ctx})")
    d.add(f"SHAPE_DEFINITION_REPRESENTATION(#{pds},#{rep})")
    return rep


class Solid:
    """Builds the B-rep entities of one solid into a Doc.

    vertices: list of points; edges: list of dicts {a, b, curve} (curve = id of the 3D curve
    entity); faces: list of dicts {surface, same, loops: [[(edge, forward)]]}.
    """

    def __init__(self, d):
        self.d = d
        self.v = []
        self.e = []
        self.f = []
        self.vpts = []

    def vertex(self, p):
        self.vpts.append(p)
        pt = self.d.point(p)
        self.v.append(self.d.add(f"VERTEX_POINT('',#{pt})"))
        return len(self.v) - 1

    def edge(self, a, b, curve, same=True):
        self.e.append((a, b, self.d.add(f"EDGE_CURVE('',#{self.v[a]},#{self.v[b]},#{curve},{'.T.' if same else '.F.'})")))
        return len(self.e) - 1

    def line_edge(self, a, b):
        pa, pb = self.vpts[a], self.vpts[b]
        d = unit(sub(pb, pa))
        pt = self.d.point(pa)
        dr = self.d.direction(d)
        vc = self.d.add(f"VECTOR('',#{dr},1.)")
        ln = self.d.add(f"LINE('',#{pt},#{vc})")
        return self.edge(a, b, ln)

    def find(self, a, b):
        for i, (x, y, _) in enumerate(self.e):
            if (x, y) == (a, b):
                return i, True
            if (x, y) == (b, a):
                return i, False
        raise KeyError((a, b))

    def loop_from_vertices(self, cycle):
        return [self.find(cycle[i], cycle[(i + 1) % len(cycle)]) for i in range(len(cycle))]

    def face(self, surface, loops, same=True):
        bounds = []
        for k, lp in enumerate(loops):
            oes = [self.d.add(f"ORIENTED_EDGE('',*,*,#{self.e[ei][2]},{'.T.' if fw else '.F.'})") for ei, fw in lp]
            el = self.d.add(f"EDGE_LOOP('',({','.join('#%d' % o for o in oes)}))")
            kind = "FACE_OUTER_BOUND" if k == 0 else "FACE_BOUND"
            bounds.append(self.d.add(f"{kind}('',#{el},.T.)"))
        fid = self.d.add(f"ADVANCED_FACE('',({','.join('#%d' % b for b in bounds)}),#{surface},{'.T.' if same else '.F.'})")
        self.f.append(fid)
        return fid

    def plane(self, origin, normal):
        ax = self.d.axis(origin, normal, ref_for(normal))
        return self.d.add(f"PLANE('',#{ax})")

    def shell(self):
        return self.d.add(f"CLOSED_SHELL('',({','.join('#%d' % f for f in self.f)}))")


CORNER_CYCLES = [[0, 4, 6, 2], [1, 3, 7, 5], [0, 1, 5, 4], [2, 6, 7, 3], [0, 2, 3, 1], [4, 5, 7, 6]]
NORMALS = [(-1.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0), (0.0, 0.0, 1.0)]


def brick_parts(d, mn, size, inward=False, off_face0=0.0):
    """Mirror of sm_topo_api::fixtures::brick_parts: same entity order."""
    s = Solid(d)

    def corner(i):
        return (mn[0] + (i & 1) * size[0], mn[1] + ((i >> 1) & 1) * size[1], mn[2] + ((i >> 2) & 1) * size[2])

    for i in range(8):
        s.vertex(corner(i))
    spec = []
    for k in range(2):
        for j in range(2):
            spec.append((2 * j + 4 * k, 1 + 2 * j + 4 * k))
    for k in range(2):
        for i in range(2):
            spec.append((i + 4 * k, i + 2 + 4 * k))
    for j in range(2):
        for i in range(2):
            spec.append((i + 2 * j, i + 2 * j + 4))
    for a, b in spec:
        s.line_edge(a, b)
    for f in range(6):
        cyc = list(CORNER_CYCLES[f])
        n = NORMALS[f]
        if inward:
            cyc.reverse()
            n = tuple(-c for c in n)
        o = corner(cyc[0])
        if f == 0 and off_face0:
            o = tuple(o[k] + n[k] * off_face0 for k in range(3))
        surf = s.plane(o, n)
        s.face(surf, [s.loop_from_vertices(cyc)])
    return s


def write(name, text):
    with open(name, "w", newline="\n") as fh:
        fh.write(text)


def brick_doc(mn, size, schema=AP214, length="mm", ap203=False, name="brick"):
    d = Doc()
    ctx = unit_context(d, length)
    s = brick_parts(d, mn, size)
    sh = s.shell()
    solid = d.add(f"MANIFOLD_SOLID_BREP('{name}',#{sh})")
    product(d, ctx, [solid], ap203)
    return d, s


def brick_text(mn, size, schema=AP214, length="mm", ap203=False, name="brick"):
    d, _ = brick_doc(mn, size, schema, length, ap203, name)
    return header(schema, name) + d.text() + FOOTER


def two_bricks_text():
    d = Doc()
    ctx = unit_context(d, "mm")
    solids = []
    for mn, size, nm in [((0.0, 0.0, 0.0), (1.0, 1.0, 1.0), "A"), ((1.0, 0.0, 0.0), (1.0, 2.0, 3.0), "B")]:
        s = brick_parts(d, mn, size)
        solids.append(d.add(f"MANIFOLD_SOLID_BREP('{nm}',#{s.shell()})"))
    product(d, ctx, solids)
    return header(AP214, "two_bricks") + d.text() + FOOTER


def void_text(inward):
    d = Doc()
    ctx = unit_context(d, "mm")
    outer = brick_parts(d, (0.0, 0.0, 0.0), (10.0, 10.0, 10.0))
    cav = brick_parts(d, (4.0, 4.0, 4.0), (2.0, 2.0, 2.0), inward=inward)
    o_sh = outer.shell()
    c_sh = cav.shell()
    # OCC style: outward cube faces + ORIENTED_CLOSED_SHELL .F.; the inward variant carries
    # inward planes and .T. (same cells as sm_topo_api::fixtures::brick_with_void).
    oc = d.add(f"ORIENTED_CLOSED_SHELL('',*,#{c_sh},{'.T.' if inward else '.F.'})")
    solid = d.add(f"BREP_WITH_VOIDS('void',#{o_sh},(#{oc}))")
    product(d, ctx, [solid])
    return header(AP214, "brick_with_void") + d.text() + FOOTER


def frustum_text():
    d = Doc()
    ctx = unit_context(d, "mm")
    s = Solid(d)
    vb = s.vertex((2.0, 0.0, 0.0))
    vt = s.vertex((1.0, 0.0, 3.0))
    zdir, xdir = (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)
    cb = d.add(f"CIRCLE('',#{d.axis((0.0, 0.0, 0.0), zdir, xdir)},2.)")
    ct = d.add(f"CIRCLE('',#{d.axis((0.0, 0.0, 3.0), zdir, xdir)},1.)")
    L = math.sqrt(10.0)
    pt = d.point((2.0, 0.0, 0.0))
    dr = d.direction((-1.0 / L, 0.0, 3.0 / L))
    vc = d.add(f"VECTOR('',#{dr},1.)")
    ln = d.add(f"LINE('',#{pt},#{vc})")
    e_bot = s.edge(vb, vb, cb)
    e_top = s.edge(vt, vt, ct)
    e_seam = s.edge(vb, vt, ln)
    s_bot = s.plane((0.0, 0.0, 0.0), (0.0, 0.0, -1.0))
    s_top = s.plane((0.0, 0.0, 3.0), (0.0, 0.0, 1.0))
    cone_ax = d.axis((0.0, 0.0, 3.0), (0.0, 0.0, -1.0), xdir)
    s_lat = d.add(f"CONICAL_SURFACE('',#{cone_ax},1.,{fmt(math.atan(1.0 / 3.0))})")
    s.face(s_bot, [[(e_bot, False)]])
    s.face(s_top, [[(e_top, True)]])
    s.face(s_lat, [[(e_bot, True), (e_seam, True), (e_top, False), (e_seam, False)]])
    solid = d.add(f"MANIFOLD_SOLID_BREP('frustum',#{s.shell()})")
    product(d, ctx, [solid])
    return header(AP214, "frustum") + d.text() + FOOTER


def sphere_text():
    d = Doc()
    ctx = unit_context(d, "mm")
    s = Solid(d)
    vs = s.vertex((0.0, 0.0, -3.0))
    vn = s.vertex((0.0, 0.0, 3.0))
    # Meridian in the xz plane: circle frame z = -y, x = +x gives (3 cos t, 0, 3 sin t).
    mer = d.add(f"CIRCLE('',#{d.axis((0.0, 0.0, 0.0), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0))},3.)")
    seam = s.edge(vs, vn, mer)
    sp = d.add(f"SPHERICAL_SURFACE('',#{d.axis((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))},3.)")
    s.face(sp, [[(seam, True), (seam, False)]])
    solid = d.add(f"MANIFOLD_SOLID_BREP('sphere',#{s.shell()})")
    product(d, ctx, [solid])
    return header(AP214, "sphere") + d.text() + FOOTER


def fillet_text():
    d = Doc()
    ctx = unit_context(d, "mm")
    s = Solid(d)
    r = 2.0
    # bottom ring (z = 0): 0..4 then top ring 5..9, same xy order
    xy = [(r, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, r)]
    for z in (0.0, 10.0):
        for x, y in xy:
            s.vertex((x, y, z))
    w = math.sqrt(2.0) / 2.0
    # rational quadratic arcs from (0, r) to (r, 0), centre of curvature (r, r)
    def arc_curve(z):
        cps = [d.point((0.0, r, z)), d.point((0.0, 0.0, z)), d.point((r, 0.0, z))]
        return d.add(
            "( B_SPLINE_CURVE(2,(" + ",".join("#%d" % c for c in cps) + "),.UNSPECIFIED.,.F.,.F.) "
            "B_SPLINE_CURVE_WITH_KNOTS((3,3),(0.,1.),.UNSPECIFIED.) BOUNDED_CURVE() CURVE() "
            f"GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE((1.,{fmt(w)},1.)) REPRESENTATION_ITEM('') )"
        )
    # edges: bottom ring lines 0-1,1-2,2-3,3-4 and the arc 4->0 (curve runs (0,r)->(r,0))
    ring = {}
    for z, base in ((0.0, 0), (10.0, 5)):
        for k in range(3):
            ring[(base, k)] = s.line_edge(base + k, base + k + 1)
        ring[(base, 3)] = s.line_edge(base + 3, base + 4)
        ring[(base, 4)] = s.edge(base + 4, base, arc_curve(z))
    vert = {k: s.line_edge(k, k + 5) for k in range(5)}
    # planes
    bot = s.plane((0.0, 0.0, 0.0), (0.0, 0.0, -1.0))
    top = s.plane((0.0, 0.0, 10.0), (0.0, 0.0, 1.0))
    # bottom: outward normal -z, so CCW seen from below: the xy ring reversed
    def ring_loop(base, forward):
        seq = [(ring[(base, k)], True) for k in range(5)]
        if not forward:
            seq = [(e, not f) for e, f in reversed(seq)]
        return seq
    s.face(bot, [ring_loop(0, False)])
    s.face(top, [ring_loop(5, True)])
    # side faces (vertical rectangles): outward normals +x (x=10), +y (y=10), -x (x=0), -y (y=0)
    def side(k_bottom, plane_origin, normal):
        # edge k of the ring between vertices k and k+1 (bottom), verticals at k and k+1
        a, b = k_bottom, k_bottom + 1
        cyc = [a, b, b + 5, a + 5]
        return s.face(s.plane(plane_origin, normal), [s.loop_from_vertices(cyc)])
    # y = 0 face: ring edge 0 (vertices 0,1), outward -y: CCW seen from outside = 0,1,6,5
    side(0, (r, 0.0, 0.0), (0.0, -1.0, 0.0))
    side(1, (10.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    side(2, (10.0, 10.0, 0.0), (0.0, 1.0, 0.0))
    side(3, (0.0, 10.0, 0.0), (-1.0, 0.0, 0.0))
    # the fillet: u along the arc (0,r)->(r,0), v = z; control net u-major
    cps = []
    for (x, y) in [(0.0, r), (0.0, 0.0), (r, 0.0)]:
        row = [d.point((x, y, 0.0)), d.point((x, y, 10.0))]
        cps.append(row)
    rows = ",".join("(" + ",".join("#%d" % p for p in row) + ")" for row in cps)
    wrows = f"((1.,1.),({fmt(w)},{fmt(w)}),(1.,1.))"
    surf = d.add(
        "( B_SPLINE_SURFACE(2,1,(" + rows + "),.UNSPECIFIED.,.F.,.F.,.F.) "
        "B_SPLINE_SURFACE_WITH_KNOTS((3,3),(2,2),(0.,1.),(0.,10.),.UNSPECIFIED.) BOUNDED_SURFACE() "
        f"GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE({wrows}) REPRESENTATION_ITEM('') SURFACE() )"
    )
    # loop: bottom arc forward (4->0 is the curve direction) then vertical 0 up, top arc reversed, vertical 4 down.
    # The edge 4->0 runs along the curve; the face boundary is traversed along u (curve) first.
    loop = [(ring[(0, 4)], True), (vert[0], True), (ring[(5, 4)], False), (vert[4], False)]
    s.face(surf, [loop])
    solid = d.add(f"MANIFOLD_SOLID_BREP('fillet',#{s.shell()})")
    product(d, ctx, [solid])
    return header(AP214, "block_fillet_nurbs") + d.text() + FOOTER


def inverted_text():
    """A brick whose faces point inward: the importer must flip it (step.inverted_solid)."""
    d = Doc()
    ctx = unit_context(d, "mm")
    s = brick_parts(d, (0.0, 0.0, 0.0), (2.0, 3.0, 4.0), inward=True)
    solid = d.add(f"MANIFOLD_SOLID_BREP('inverted',#{s.shell()})")
    product(d, ctx, [solid])
    return header(AP214, "brick_inverted") + d.text() + FOOTER


def off_surface_text():
    """A brick whose first plane does not contain its boundary edges (validate_geometry fails)."""
    d = Doc()
    ctx = unit_context(d, "mm")
    s = brick_parts(d, (0.0, 0.0, 0.0), (2.0, 3.0, 4.0), off_face0=0.5)
    solid = d.add(f"MANIFOLD_SOLID_BREP('off',#{s.shell()})")
    product(d, ctx, [solid])
    return header(AP214, "face_off_surface") + d.text() + FOOTER


def main():
    write("brick_inverted.stp", inverted_text())
    write("bad/face_off_surface.stp", off_surface_text())
    write("brick_2x3x4.stp", brick_text((1.0, 2.0, 3.0), (2.0, 3.0, 4.0), name="brick_2x3x4"))
    write("brick_ap203.stp", brick_text((1.0, 2.0, 3.0), (2.0, 3.0, 4.0), schema=AP203, ap203=True, name="brick_2x3x4"))
    write("brick_inch.stp", brick_text((0.0, 0.0, 0.0), (1.0, 2.0, 3.0), length="inch", name="brick_inch"))
    write("two_bricks.stp", two_bricks_text())
    write("brick_with_void.stp", void_text(False))
    write("brick_with_void_inward.stp", void_text(True))
    write("frustum_r2_r1_h3.stp", frustum_text())
    write("sphere_seam_r3.stp", sphere_text())
    write("block_fillet_nurbs.stp", fillet_text())
    base = brick_text((0.0, 0.0, 0.0), (2.0, 3.0, 4.0), name="bad")
    lines = base.split("\n")
    # bad/not_step.txt
    write("bad/not_step.txt", "This is not a STEP file.\nJust text.\n")
    # truncated: cut the file before ENDSEC of the data section
    cut = base[: base.index("ENDSEC;\nEND-ISO")]
    write("bad/truncated_no_endsec.stp", cut)
    # unterminated string: the first entity gets a string that never closes
    ds = base.index("DATA;\n") + len("DATA;\n")
    first_nl = base.index("\n", ds)
    write("bad/unterminated_string.stp", base[:ds] + "#1 = APPLICATION_CONTEXT('core data" + base[first_nl:].replace("'", ""))
    # dangling reference: drop a CARTESIAN_POINT that a VERTEX_POINT uses
    d, s = brick_doc((0.0, 0.0, 0.0), (2.0, 3.0, 4.0), name="bad")
    vp = d.ents[s.v[3]]
    pid = int(vp.split("#")[1].rstrip(")"))
    del d.ents[pid]
    write("bad/dangling_ref.stp", header(AP214, "bad") + d.text() + FOOTER)
    # cyclic reference: an edge geometry that is a SURFACE_CURVE pair pointing at each other
    d, s = brick_doc((0.0, 0.0, 0.0), (2.0, 3.0, 4.0), name="bad")
    a, b = d.new(), d.new()
    d.put(a, f"SURFACE_CURVE('',#{b},(),.PCURVE_S1.)")
    d.put(b, f"SURFACE_CURVE('',#{a},(),.PCURVE_S1.)")
    v0, v1, eid = s.e[0]
    d.put(eid, f"EDGE_CURVE('',#{s.v[s.e[0][0]]},#{s.v[s.e[0][1]]},#{a},.T.)")
    write("bad/cyclic_ref.stp", header(AP214, "bad") + d.text() + FOOTER)
    # wrong parameter type: same_sense given as a number
    d, s = brick_doc((0.0, 0.0, 0.0), (2.0, 3.0, 4.0), name="bad")
    fid = s.f[0]
    d.ents[fid] = d.ents[fid].rsplit(",", 1)[0] + ",3)"
    write("bad/wrong_param_type.stp", header(AP214, "bad") + d.text() + FOOTER)
    # unsupported: face geometry is an OFFSET_SURFACE
    d, s = brick_doc((0.0, 0.0, 0.0), (2.0, 3.0, 4.0), name="bad")
    pl = d.ents[s.f[0]].split("),#")[1].split(",")[0]
    off = d.add(f"OFFSET_SURFACE('',#{pl},0.5,.F.)")
    d.ents[s.f[0]] = d.ents[s.f[0]].replace(f"),#{pl},", f"),#{off},")
    write("bad/unsupported_offset_surface.stp", header(AP214, "bad") + d.text() + FOOTER)
    # zero radius cylinder: take the frustum and zero the cylinder radius
    d = Doc()
    ctx = unit_context(d, "mm")
    s = Solid(d)
    vb = s.vertex((0.0, 0.0, 0.0))
    vt = s.vertex((0.0, 0.0, 5.0))
    zdir, xdir = (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)
    cb = d.add(f"CIRCLE('',#{d.axis((0.0, 0.0, 0.0), zdir, xdir)},2.)")
    ct = d.add(f"CIRCLE('',#{d.axis((0.0, 0.0, 5.0), zdir, xdir)},2.)")
    pt = d.point((0.0, 0.0, 0.0))
    vc = d.add(f"VECTOR('',#{d.direction(zdir)},1.)")
    ln = d.add(f"LINE('',#{pt},#{vc})")
    e_bot = s.edge(vb, vb, cb)
    e_top = s.edge(vt, vt, ct)
    e_seam = s.edge(vb, vt, ln)
    s_bot = s.plane((0.0, 0.0, 0.0), (0.0, 0.0, -1.0))
    s_top = s.plane((0.0, 0.0, 5.0), (0.0, 0.0, 1.0))
    cyl = d.add(f"CYLINDRICAL_SURFACE('',#{d.axis((0.0, 0.0, 0.0), zdir, xdir)},0.)")
    s.face(s_bot, [[(e_bot, False)]])
    s.face(s_top, [[(e_top, True)]])
    s.face(cyl, [[(e_bot, True), (e_seam, True), (e_top, False), (e_seam, False)]])
    solid = d.add(f"MANIFOLD_SOLID_BREP('cyl',#{s.shell()})")
    product(d, ctx, [solid])
    write("bad/zero_radius_cylinder.stp", header(AP214, "bad") + d.text() + FOOTER)
    # no units
    d = Doc()
    ctx = unit_context(d, None)
    s = brick_parts(d, (0.0, 0.0, 0.0), (2.0, 3.0, 4.0))
    solid = d.add(f"MANIFOLD_SOLID_BREP('nounits',#{s.shell()})")
    product(d, ctx, [solid])
    write("bad/no_units.stp", header(AP214, "bad") + d.text() + FOOTER)
    # open shell: drop the last face from the shell
    d = Doc()
    ctx = unit_context(d, "mm")
    s = brick_parts(d, (0.0, 0.0, 0.0), (2.0, 3.0, 4.0))
    s.f.pop()
    solid = d.add(f"MANIFOLD_SOLID_BREP('open',#{s.shell()})")
    product(d, ctx, [solid])
    write("bad/open_shell.stp", header(AP214, "bad") + d.text() + FOOTER)


if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    main()
