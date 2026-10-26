# Single-solid STEP fixtures written by Gmsh's OCC kernel; prints analytic and OCC volumes.
import gmsh, math
gmsh.initialize(); gmsh.option.setNumber("General.Verbosity", 1)
occ = gmsh.model.occ
def out(name, make, exact):
    gmsh.clear(); gmsh.model.add(name); t = make(); occ.synchronize()
    print(f"{name}: occ_volume={occ.getMass(3, t):.12g} exact={exact}")
    gmsh.write(f"{name}.step")
out("cylinder_r2_h5", lambda: occ.addCylinder(0,0,0, 0,0,5, 2), f"{math.pi*4*5:.12g}")
out("sphere_r3", lambda: occ.addSphere(0,0,0, 3), f"{4/3*math.pi*27:.12g}")
out("torus_R5_r1", lambda: occ.addTorus(0,0,0, 5, 1), f"{2*math.pi**2*5:.12g}")
out("cone_r2_h3", lambda: occ.addCone(0,0,0, 0,0,3, 2, 0), f"{math.pi*4*3/3:.12g}")
def fillet():
    b = occ.addBox(0,0,0, 10,10,10); occ.synchronize()
    e = [t for d,t in gmsh.model.getBoundary([(3,b)], combined=False, oriented=False, recursive=True) if d==1]
    edges = sorted({t for d,t in gmsh.model.getEntities(1)})
    return occ.fillet([b], edges[:1], [1.0])[0][1]
out("box_fillet1", fillet, f"{1000-(1-math.pi/4)*10:.12g} (one edge r=1)")
def loft():
    c1 = occ.addCurveLoop([occ.addCircle(0,0,0, 2)]); c2 = occ.addCurveLoop([occ.addEllipse(0,0,4, 3,1.5)])
    return [e for e in occ.addThruSections([c1,c2], makeSolid=True, makeRuled=False) if e[0]==3][0][1]
out("loft_bspline", loft, "numeric only")
gmsh.finalize()
