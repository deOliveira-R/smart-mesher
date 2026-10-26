# Milestone W3a part: three separate solids (not fused), written by Gmsh's OCC kernel.
import gmsh, sys
gmsh.initialize(); gmsh.option.setNumber("General.Verbosity", 1)
occ = gmsh.model.occ
def prism(pts, z0, z1):
    tags = [occ.addPoint(x, y, z0) for x, y in pts]
    lines = [occ.addLine(tags[i], tags[(i + 1) % len(tags)]) for i in range(len(tags))]
    s = occ.addPlaneSurface([occ.addCurveLoop(lines)])
    return [e for e in occ.extrude([(2, s)], 0, 0, z1 - z0) if e[0] == 3][0][1]
v1 = prism([(0,0),(20,0),(20,10),(10,10),(10,20),(0,20)], 0, 4)
v2 = prism([(10,10),(20,10),(10,20)], 0, 4)
v3 = occ.addBox(10, 10, 4, 10, 10, 6)
occ.synchronize()
for v in (v1, v2, v3): print("vol", v, occ.getMass(3, v))
gmsh.write(sys.argv[1]); gmsh.finalize()
