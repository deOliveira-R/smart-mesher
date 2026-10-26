// A generator for large synthetic STEP files (disjoint bricks), shared by tests and benches.

use std::fmt::Write as _;

const CORNER_CYCLES: [[usize; 4]; 6] = [[0, 4, 6, 2], [1, 3, 7, 5], [0, 1, 5, 4], [2, 6, 7, 3], [0, 2, 3, 1], [4, 5, 7, 6]];
const NORMALS: [[i32; 3]; 6] = [[-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0], [0, 0, -1], [0, 0, 1]];

/// A STEP file holding `n` unit bricks `[x, x+1] x [y, y+1] x [z, z+1]` on a grid of pitch 3
/// (so they are disjoint), all in one `ADVANCED_BREP_SHAPE_REPRESENTATION`, millimetres.
#[allow(dead_code)]
pub fn bricks_step(n: usize) -> Vec<u8> {
    let mut s = String::with_capacity(n * 9000 + 2000);
    s.push_str("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('generated'),'2;1');\nFILE_NAME('bricks','1970-01-01T00:00:00',('x'),(''),'gen','',' ');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n");
    s.push_str("#1 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n#2 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n#3 = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );\n");
    s.push_str("#4 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#1,'distance_accuracy_value','confusion accuracy');\n");
    s.push_str("#5 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#4)) GLOBAL_UNIT_ASSIGNED_CONTEXT((#1,#2,#3)) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') );\n");
    let mut id = 5u64;
    let mut solids = Vec::with_capacity(n);
    let next = |id: &mut u64| {
        *id += 1;
        *id
    };
    for k in 0..n {
        let (ox, oy, oz) = (3.0 * (k % 32) as f64, 3.0 * ((k / 32) % 32) as f64, 3.0 * (k / 1024) as f64);
        let corner = |i: usize| [ox + (i & 1) as f64, oy + ((i >> 1) & 1) as f64, oz + ((i >> 2) & 1) as f64];
        let mut vids = [0u64; 8];
        for (i, vid) in vids.iter_mut().enumerate() {
            let p = corner(i);
            let pid = next(&mut id);
            let _ = writeln!(s, "#{pid} = CARTESIAN_POINT('',({:?},{:?},{:?}));", p[0], p[1], p[2]);
            let v = next(&mut id);
            let _ = writeln!(s, "#{v} = VERTEX_POINT('',#{pid});");
            *vid = v;
        }
        let mut spec: Vec<(usize, usize, usize)> = Vec::with_capacity(12);
        for kk in 0..2 {
            for j in 0..2 {
                spec.push((2 * j + 4 * kk, 1 + 2 * j + 4 * kk, 0));
            }
        }
        for kk in 0..2 {
            for i in 0..2 {
                spec.push((i + 4 * kk, i + 2 + 4 * kk, 1));
            }
        }
        for j in 0..2 {
            for i in 0..2 {
                spec.push((i + 2 * j, i + 2 * j + 4, 2));
            }
        }
        let mut eids = Vec::with_capacity(12);
        for (a, b, axis) in &spec {
            let pa = corner(*a);
            let pt = next(&mut id);
            let _ = writeln!(s, "#{pt} = CARTESIAN_POINT('',({:?},{:?},{:?}));", pa[0], pa[1], pa[2]);
            let dr = next(&mut id);
            let d = [(*axis == 0) as u8 as f64, (*axis == 1) as u8 as f64, (*axis == 2) as u8 as f64];
            let _ = writeln!(s, "#{dr} = DIRECTION('',({:?},{:?},{:?}));", d[0], d[1], d[2]);
            let vc = next(&mut id);
            let _ = writeln!(s, "#{vc} = VECTOR('',#{dr},1.);");
            let ln = next(&mut id);
            let _ = writeln!(s, "#{ln} = LINE('',#{pt},#{vc});");
            let e = next(&mut id);
            let _ = writeln!(s, "#{e} = EDGE_CURVE('',#{},#{},#{ln},.T.);", vids[*a], vids[*b]);
            eids.push(e);
        }
        let mut faces = Vec::with_capacity(6);
        for f in 0..6 {
            let cyc = CORNER_CYCLES[f];
            let nrm = NORMALS[f];
            let org = corner(cyc[0]);
            let po = next(&mut id);
            let _ = writeln!(s, "#{po} = CARTESIAN_POINT('',({:?},{:?},{:?}));", org[0], org[1], org[2]);
            let dn = next(&mut id);
            let _ = writeln!(s, "#{dn} = DIRECTION('',({:?},{:?},{:?}));", f64::from(nrm[0]), f64::from(nrm[1]), f64::from(nrm[2]));
            let r = if nrm[0] != 0 { [0.0, 1.0, 0.0] } else if nrm[1] != 0 { [0.0, 0.0, 1.0] } else { [1.0, 0.0, 0.0] };
            let dr = next(&mut id);
            let _ = writeln!(s, "#{dr} = DIRECTION('',({:?},{:?},{:?}));", r[0], r[1], r[2]);
            let ax = next(&mut id);
            let _ = writeln!(s, "#{ax} = AXIS2_PLACEMENT_3D('',#{po},#{dn},#{dr});");
            let pl = next(&mut id);
            let _ = writeln!(s, "#{pl} = PLANE('',#{ax});");
            let mut oes = Vec::with_capacity(4);
            for i in 0..4 {
                let (a, b) = (cyc[i], cyc[(i + 1) % 4]);
                let (ei, fwd) = match spec.iter().position(|x| x.0 == a && x.1 == b) {
                    Some(p) => (p, true),
                    None => (spec.iter().position(|x| x.0 == b && x.1 == a).unwrap_or(0), false),
                };
                let oe = next(&mut id);
                let _ = writeln!(s, "#{oe} = ORIENTED_EDGE('',*,*,#{},{});", eids[ei], if fwd { ".T." } else { ".F." });
                oes.push(oe);
            }
            let lp = next(&mut id);
            let _ = writeln!(s, "#{lp} = EDGE_LOOP('',(#{},#{},#{},#{}));", oes[0], oes[1], oes[2], oes[3]);
            let fb = next(&mut id);
            let _ = writeln!(s, "#{fb} = FACE_OUTER_BOUND('',#{lp},.T.);");
            let fa = next(&mut id);
            let _ = writeln!(s, "#{fa} = ADVANCED_FACE('',(#{fb}),#{pl},.T.);");
            faces.push(fa);
        }
        let sh = next(&mut id);
        let list: Vec<String> = faces.iter().map(|f| format!("#{f}")).collect();
        let _ = writeln!(s, "#{sh} = CLOSED_SHELL('',({}));", list.join(","));
        let so = next(&mut id);
        let _ = writeln!(s, "#{so} = MANIFOLD_SOLID_BREP('b{k}',#{sh});");
        solids.push(so);
    }
    let items: Vec<String> = solids.iter().map(|x| format!("#{x}")).collect();
    let rep = next(&mut id);
    let _ = writeln!(s, "#{rep} = ADVANCED_BREP_SHAPE_REPRESENTATION('',({}),#5);", items.join(","));
    s.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    s.into_bytes()
}
