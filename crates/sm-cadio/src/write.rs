//! The STEP writer (stretch S2): a cell complex back to AP214 / AP242 text.
//!
//! One `MANIFOLD_SOLID_BREP` / `BREP_WITH_VOIDS` per volume in one
//! `ADVANCED_BREP_SHAPE_REPRESENTATION`. Entities are numbered in a fixed order (units and
//! product structure, then per solid its vertices, edges, faces, shells and the solid, each
//! ascending by model id), so reading the output reproduces the cell order and writing the
//! result again gives the same bytes. Reals are the shortest text that round-trips, always with
//! a `.`.
//!
//! What STEP cannot hold is reported in [`WriteReport::losses`]: a face shared by two volumes is
//! written once per volume, pole edges are dropped (the seam pair closes the loop), and
//! baffles, wires, free faces, coincidence records and tolerance overrides are not written.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::io::Write;

use sm_core::{EntityId, Point3};
use sm_geom_api::{CurveDef, Frame, SurfaceDef};
use sm_layer_api::ModelLayer;
use sm_op_api::State;
use sm_topo_api::{Carrier, CellFlags, Dim, Sense, ShellKind, Side, TopologyView, TopologyViewExt};

use crate::error::StepError;
use crate::model::LengthUnit;

/// The STEP application protocol written.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum StepSchema {
    /// `AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }` (default).
    #[default]
    Ap214,
    /// `AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 1 1 4 }`.
    Ap242,
}

/// Options of [`write_step`].
#[derive(Clone, Debug, PartialEq)]
pub struct WriteOptions {
    /// Application protocol.
    pub schema: StepSchema,
    /// The unit the coordinates are declared in. Nothing is scaled: this only names the unit
    /// the model's numbers are in. Default millimetres.
    pub unit: LengthUnit,
    /// The `FILE_NAME` time stamp. Default the fixed `1970-01-01T00:00:00`, so output is
    /// reproducible.
    pub timestamp: Option<String>,
}

impl Default for WriteOptions {
    fn default() -> Self {
        Self { schema: StepSchema::Ap214, unit: LengthUnit::Mm, timestamp: None }
    }
}

/// Something the STEP file cannot represent.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Loss {
    /// The cell concerned, if one.
    pub cell: Option<EntityId>,
    /// What was lost.
    pub what: String,
}

/// What [`write_step`] wrote.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct WriteReport {
    /// Solids written (one per volume).
    pub solids: usize,
    /// `ADVANCED_FACE`s written (a face shared by two volumes counts twice).
    pub faces: usize,
    /// `EDGE_CURVE`s written (counted per solid).
    pub edges: usize,
    /// `VERTEX_POINT`s written (counted per solid).
    pub vertices: usize,
    /// Size of the output in bytes.
    pub bytes: u64,
    /// What STEP could not hold.
    pub losses: Vec<Loss>,
}

fn export_err(msg: impl Into<String>) -> StepError {
    StepError::Export(msg.into())
}

fn unsupported(cell: EntityId, what: impl std::fmt::Display) -> StepError {
    StepError::Unsupported { id: cell.serial(), entity: format!("{cell}: {what}") }
}

/// The shortest text that parses back to `x`, with a `.` (or an exponent after one).
fn real(x: f64, cell: EntityId) -> Result<String, StepError> {
    if !x.is_finite() {
        return Err(unsupported(cell, "a non-finite number"));
    }
    let s = format!("{x:?}");
    Ok(match s.split_once('e') {
        Some((m, e)) => {
            let m = if m.contains('.') { m.to_owned() } else { format!("{m}.") };
            format!("{m}E{e}")
        }
        None => match s.strip_suffix(".0") {
            Some(i) => format!("{i}."),
            None => s,
        },
    })
}

struct Writer<'a> {
    view: &'a dyn TopologyView,
    out: String,
    next: u64,
}

impl Writer<'_> {
    fn add(&mut self, text: &str) -> u64 {
        self.next += 1;
        let _ = writeln!(self.out, "#{} = {text};", self.next);
        self.next
    }

    fn point(&mut self, p: Point3, cell: EntityId) -> Result<u64, StepError> {
        let t = format!("CARTESIAN_POINT('',({},{},{}))", real(p.x, cell)?, real(p.y, cell)?, real(p.z, cell)?);
        Ok(self.add(&t))
    }

    fn direction(&mut self, v: [f64; 3], cell: EntityId) -> Result<u64, StepError> {
        let t = format!("DIRECTION('',({},{},{}))", real(v[0], cell)?, real(v[1], cell)?, real(v[2], cell)?);
        Ok(self.add(&t))
    }

    fn placement(&mut self, f: &Frame, cell: EntityId) -> Result<u64, StepError> {
        let o = self.point(f.origin, cell)?;
        let z = self.direction([f.z.x, f.z.y, f.z.z], cell)?;
        let x = self.direction([f.x.x, f.x.y, f.x.z], cell)?;
        Ok(self.add(&format!("AXIS2_PLACEMENT_3D('',#{o},#{z},#{x})")))
    }

    fn curve(&mut self, def: &CurveDef, cell: EntityId) -> Result<u64, StepError> {
        match def {
            CurveDef::Line(l) => {
                let p = self.point(l.origin, cell)?;
                let d = self.direction([l.dir.x, l.dir.y, l.dir.z], cell)?;
                let v = self.add(&format!("VECTOR('',#{d},1.)"));
                Ok(self.add(&format!("LINE('',#{p},#{v})")))
            }
            CurveDef::Circle(c) => {
                let a = self.placement(&c.frame, cell)?;
                let r = real(c.radius, cell)?;
                Ok(self.add(&format!("CIRCLE('',#{a},{r})")))
            }
            CurveDef::Ellipse(e) => {
                let a = self.placement(&e.frame, cell)?;
                let (ra, rb) = (real(e.a, cell)?, real(e.b, cell)?);
                Ok(self.add(&format!("ELLIPSE('',#{a},{ra},{rb})")))
            }
            CurveDef::Nurbs(n) => {
                if n.periodic {
                    return Err(unsupported(cell, "a periodic B-spline curve"));
                }
                let pts: Vec<String> = n.control.iter().map(|p| self.point(*p, cell).map(|i| format!("#{i}"))).collect::<Result<_, _>>()?;
                let (m, k) = knots(&n.knots, cell)?;
                let closed = if n.control.first() == n.control.last() { ".T." } else { ".F." };
                let pl = pts.join(",");
                Ok(match &n.weights {
                    None => self.add(&format!("B_SPLINE_CURVE_WITH_KNOTS('',{},({pl}),.UNSPECIFIED.,{closed},.F.,({m}),({k}),.UNSPECIFIED.)", n.degree)),
                    Some(w) => {
                        let w: Vec<String> = w.iter().map(|x| real(*x, cell)).collect::<Result<_, _>>()?;
                        self.add(&format!(
                            "( BOUNDED_CURVE() B_SPLINE_CURVE({},({pl}),.UNSPECIFIED.,{closed},.F.) B_SPLINE_CURVE_WITH_KNOTS(({m}),({k}),.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(({})) REPRESENTATION_ITEM('') )",
                            n.degree,
                            w.join(",")
                        ))
                    }
                })
            }
            _ => Err(unsupported(cell, "a curve kind the writer does not know")),
        }
    }

    fn surface(&mut self, def: &SurfaceDef, cell: EntityId) -> Result<u64, StepError> {
        match def {
            SurfaceDef::Plane(p) => {
                let a = self.placement(&p.frame, cell)?;
                Ok(self.add(&format!("PLANE('',#{a})")))
            }
            SurfaceDef::Cylinder(c) => {
                let a = self.placement(&c.frame, cell)?;
                let r = real(c.radius, cell)?;
                Ok(self.add(&format!("CYLINDRICAL_SURFACE('',#{a},{r})")))
            }
            SurfaceDef::Cone(c) => {
                let a = self.placement(&c.frame, cell)?;
                let (r, h) = (real(c.radius, cell)?, real(c.half_angle, cell)?);
                Ok(self.add(&format!("CONICAL_SURFACE('',#{a},{r},{h})")))
            }
            SurfaceDef::Sphere(s) => {
                let a = self.placement(&s.frame, cell)?;
                let r = real(s.radius, cell)?;
                Ok(self.add(&format!("SPHERICAL_SURFACE('',#{a},{r})")))
            }
            SurfaceDef::Torus(t) => {
                let a = self.placement(&t.frame, cell)?;
                let (r, q) = (real(t.major, cell)?, real(t.minor, cell)?);
                Ok(self.add(&format!("TOROIDAL_SURFACE('',#{a},{r},{q})")))
            }
            SurfaceDef::Nurbs(n) => {
                if n.periodic != [false, false] {
                    return Err(unsupported(cell, "a periodic B-spline surface"));
                }
                let mut rows = Vec::with_capacity(n.nu);
                for iu in 0..n.nu {
                    let mut row = Vec::with_capacity(n.nv);
                    for iv in 0..n.nv {
                        row.push(format!("#{}", self.point(n.control[iu * n.nv + iv], cell)?));
                    }
                    rows.push(format!("({})", row.join(",")));
                }
                let rows = rows.join(",");
                let (mu, ku) = knots(&n.knots_u, cell)?;
                let (mv, kv) = knots(&n.knots_v, cell)?;
                Ok(match &n.weights {
                    None => self.add(&format!(
                        "B_SPLINE_SURFACE_WITH_KNOTS('',{},{},({rows}),.UNSPECIFIED.,.F.,.F.,.F.,({mu}),({mv}),({ku}),({kv}),.UNSPECIFIED.)",
                        n.degree_u, n.degree_v
                    )),
                    Some(w) => {
                        let mut wrows = Vec::with_capacity(n.nu);
                        for iu in 0..n.nu {
                            let row: Vec<String> = (0..n.nv).map(|iv| real(w[iu * n.nv + iv], cell)).collect::<Result<_, _>>()?;
                            wrows.push(format!("({})", row.join(",")));
                        }
                        self.add(&format!(
                            "( B_SPLINE_SURFACE({},{},({rows}),.UNSPECIFIED.,.F.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS(({mu}),({mv}),({ku}),({kv}),.UNSPECIFIED.) BOUNDED_SURFACE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(({})) REPRESENTATION_ITEM('') SURFACE() )",
                            n.degree_u,
                            n.degree_v,
                            wrows.join(",")
                        ))
                    }
                })
            }
            _ => Err(unsupported(cell, "a surface kind STEP export does not support (implicit or elliptic)")),
        }
    }
}

/// Distinct knots and multiplicities as the two STEP lists.
fn knots(k: &[f64], cell: EntityId) -> Result<(String, String), StepError> {
    let mut mult: Vec<(f64, usize)> = Vec::new();
    for &x in k {
        match mult.last_mut() {
            Some((v, c)) if *v == x => *c += 1,
            _ => mult.push((x, 1)),
        }
    }
    let m: Vec<String> = mult.iter().map(|(_, c)| c.to_string()).collect();
    let v: Vec<String> = mult.iter().map(|(x, _)| real(*x, cell)).collect::<Result<_, _>>()?;
    Ok((m.join(","), v.join(",")))
}

fn unit_entities(w: &mut Writer<'_>, unit: LengthUnit, unc: f64, cell: EntityId) -> Result<u64, StepError> {
    let mm = "( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )";
    let lu = match unit {
        LengthUnit::Mm => w.add(mm),
        LengthUnit::Um => w.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MICRO.,.METRE.) )"),
        LengthUnit::Cm => w.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.CENTI.,.METRE.) )"),
        LengthUnit::M => w.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.) )"),
        LengthUnit::In | LengthUnit::Ft => {
            let (name, f) = if unit == LengthUnit::In { ("INCH", "25.4") } else { ("FOOT", "304.8") };
            let base = w.add(mm);
            let m = w.add(&format!("LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE({f}),#{base})"));
            let dim = w.add("DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.)");
            w.add(&format!("( CONVERSION_BASED_UNIT('{name}',#{m}) LENGTH_UNIT() NAMED_UNIT(#{dim}) )"))
        }
    };
    let au = w.add("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )");
    let su = w.add("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )");
    let u = real(unc, cell)?;
    let unc = w.add(&format!("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({u}),#{lu},'distance_accuracy_value','confusion accuracy')"));
    Ok(w.add(&format!(
        "( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{lu},#{au},#{su})) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') )"
    )))
}

fn volumes_of(view: &dyn TopologyView, bodies: Option<&[EntityId]>) -> Result<Vec<EntityId>, StepError> {
    let mut out: BTreeSet<EntityId> = BTreeSet::new();
    match bodies {
        None => {
            for &b in view.cells(Dim::Body) {
                out.extend(view.boundary(b).iter().map(|e| e.cell).filter(|c| view.dim_of(*c) == Some(Dim::Volume)));
            }
            if out.is_empty() {
                // A model without bodies: every volume.
                out.extend(view.cells(Dim::Volume).iter().copied());
            }
        }
        Some(list) => {
            for &id in list {
                match view.dim_of(id) {
                    Some(Dim::Body) => out.extend(view.boundary(id).iter().map(|e| e.cell).filter(|c| view.dim_of(*c) == Some(Dim::Volume))),
                    Some(Dim::Volume) => {
                        out.insert(id);
                    }
                    _ => return Err(export_err(format!("{id} is not a body or a volume"))),
                }
            }
        }
    }
    Ok(out.into_iter().collect())
}

/// Write the bodies (all when `None`; body or volume ids) of `view` as a STEP file to `w`.
///
/// # Errors
/// [`StepError::Unsupported`] naming the cell for a curve or surface kind STEP export cannot
/// hold (implicit and elliptic surfaces, periodic B-splines, non-finite numbers),
/// [`StepError::GeometryMismatch`] when an edge's range does not end at its vertices,
/// [`StepError::Export`] for other problems and I/O failures. Nothing is written on error.
pub fn write_step(view: &dyn TopologyView, bodies: Option<&[EntityId]>, o: &WriteOptions, w: &mut dyn Write) -> Result<WriteReport, StepError> {
    let volumes = volumes_of(view, bodies)?;
    let tol = view.tolerance().linear;
    let mut wr = Writer { view, out: String::new(), next: 0 };
    let mut report = WriteReport::default();
    let Some(&anchor) = volumes.first() else { return Err(export_err("there is no volume to write")) };
    let ctx = unit_entities(&mut wr, o.unit, tol, anchor)?;
    // Product structure around the one representation (written last, it lists the solids).
    let ac = wr.add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')");
    match o.schema {
        StepSchema::Ap214 => wr.add(&format!("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{ac})")),
        StepSchema::Ap242 => wr.add(&format!("APPLICATION_PROTOCOL_DEFINITION('international standard','ap242_managed_model_based_3d_engineering',2011,#{ac})")),
    };
    let pc = wr.add(&format!("PRODUCT_CONTEXT('',#{ac},'mechanical')"));
    let prod = wr.add(&format!("PRODUCT('smart-mesh model','smart-mesh model','',(#{pc}))"));
    let form = wr.add(&format!("PRODUCT_DEFINITION_FORMATION('','',#{prod})"));
    let pdc = wr.add(&format!("PRODUCT_DEFINITION_CONTEXT('part definition',#{ac},'design')"));
    let pd = wr.add(&format!("PRODUCT_DEFINITION('design','',#{form},#{pdc})"));
    let pds = wr.add(&format!("PRODUCT_DEFINITION_SHAPE('','',#{pd})"));
    let origin = wr.placement(&Frame::at(Point3::new(0.0, 0.0, 0.0)), anchor)?;

    if bodies.is_none() {
        for &b in view.cells(Dim::Body) {
            if !view.boundary(b).iter().any(|e| view.dim_of(e.cell) == Some(Dim::Volume)) {
                report.losses.push(Loss { cell: Some(b), what: format!("{b} has no volume (a sheet or wire body): not written") });
            }
        }
    }
    let mut solids: Vec<u64> = Vec::new();
    for &vol in &volumes {
        solids.push(write_solid(&mut wr, vol, &mut report, tol)?);
    }
    let items: Vec<String> = std::iter::once(origin).chain(solids.iter().copied()).map(|i| format!("#{i}")).collect();
    let rep = wr.add(&format!("ADVANCED_BREP_SHAPE_REPRESENTATION('',({}),#{ctx})", items.join(",")));
    wr.add(&format!("SHAPE_DEFINITION_REPRESENTATION(#{pds},#{rep})"));
    if !view.coincidences().is_empty() {
        report.losses.push(Loss { cell: None, what: "coincidence records are not written".into() });
    }
    report.solids = solids.len();
    let ts = o.timestamp.clone().unwrap_or_else(|| "1970-01-01T00:00:00".to_owned());
    let schema = match o.schema {
        StepSchema::Ap214 => "AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }",
        StepSchema::Ap242 => "AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 1 1 4 }",
    };
    let esc = |s: &str| s.replace('\'', "''");
    let text = format!(
        "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('smart-mesh STEP export'),'2;1');\nFILE_NAME('smart-mesh','{}',(''),(''),'smart-mesh sm-cadio','smart-mesh',' ');\nFILE_SCHEMA(('{}'));\nENDSEC;\nDATA;\n{}ENDSEC;\nEND-ISO-10303-21;\n",
        esc(&ts),
        schema,
        wr.out
    );
    report.bytes = text.len() as u64;
    w.write_all(text.as_bytes()).map_err(|e| export_err(format!("write failed: {e}")))?;
    Ok(report)
}

/// Write the bodies of a session state (its `model` layer) as a STEP file.
///
/// # Errors
/// As [`write_step`], plus [`StepError::Export`] when the state has no `model` layer.
pub fn export_step(state: &State, bodies: Option<&[EntityId]>, o: &WriteOptions, w: &mut dyn Write) -> Result<WriteReport, StepError> {
    let layer = state.layer::<ModelLayer>().ok_or_else(|| export_err("the state has no model layer"))?;
    write_step(layer.topo(), bodies, o, w)
}

fn write_solid(wr: &mut Writer<'_>, vol: EntityId, report: &mut WriteReport, tol: f64) -> Result<u64, StepError> {
    let view = wr.view;
    // Face uses by shell: (ring, kind, [(face, sense)]).
    struct ShellUses {
        kind: ShellKind,
        uses: Vec<(EntityId, Sense)>,
    }
    let mut shells: Vec<ShellUses> = Vec::new();
    for sh in view.shells(vol) {
        let kind = view.shell_kind(vol, sh.ring).unwrap_or(ShellKind::Outer);
        let mut uses = Vec::new();
        for e in sh.entries {
            match e.side {
                Side::Left => uses.push((e.cell, Sense::Forward)),
                Side::Right => uses.push((e.cell, Sense::Reversed)),
                Side::Both => report.losses.push(Loss { cell: Some(e.cell), what: format!("baffle face of {vol} is not written") }),
            }
        }
        if kind == ShellKind::Internal {
            for (f, _) in &uses {
                report.losses.push(Loss { cell: Some(*f), what: format!("internal shell of {vol} is not written") });
            }
            continue;
        }
        uses.sort_by_key(|u| u.0);
        shells.push(ShellUses { kind, uses });
    }
    if shells.is_empty() {
        return Err(export_err(format!("{vol} has no shell to write")));
    }
    for w in view.wire_edges(vol).collect::<Vec<_>>() {
        report.losses.push(Loss { cell: Some(w), what: format!("wire embedded in {vol} is not written") });
    }
    // Faces of the solid, ascending, with the shell they sit in.
    let mut faces: BTreeMap<EntityId, (usize, bool)> = BTreeMap::new();
    for (si, sh) in shells.iter().enumerate() {
        for &(f, sense) in &sh.uses {
            let flip = (sense == Sense::Reversed) != (sh.kind == ShellKind::Void);
            faces.insert(f, (si, flip));
        }
    }
    for &f in faces.keys() {
        if view.face_uses_of(f).len() > 1 {
            report.losses.push(Loss { cell: Some(f), what: format!("face shared by several volumes is written once per volume (here for {vol})") });
        }
    }
    // Cells below the faces.
    let mut edges: BTreeSet<EntityId> = BTreeSet::new();
    for &f in faces.keys() {
        for lp in view.loops(f) {
            for e in lp.entries {
                if !view.flags(e.cell).contains(CellFlags::DEGENERATE) {
                    edges.insert(e.cell);
                }
            }
        }
    }
    let mut vertices: BTreeSet<EntityId> = BTreeSet::new();
    for &e in &edges {
        let (a, b) = view.edge_ends(e).ok_or_else(|| export_err(format!("{e} has no end vertices")))?;
        vertices.insert(a);
        vertices.insert(b);
    }
    // Vertices.
    let mut vstep: BTreeMap<EntityId, u64> = BTreeMap::new();
    for &v in &vertices {
        let Carrier::Point(p) = view.carrier(v) else { return Err(export_err(format!("{v} has no point"))) };
        let pt = wr.point(*p, v)?;
        vstep.insert(v, wr.add(&format!("VERTEX_POINT('',#{pt})")));
    }
    // Edges.
    let mut cstep: BTreeMap<EntityId, u64> = BTreeMap::new();
    let mut estep: BTreeMap<EntityId, u64> = BTreeMap::new();
    for &e in &edges {
        let Carrier::Curve(cc) = view.carrier(e) else { return Err(unsupported(e, "an edge without a curve")) };
        let def = view.shapes().curve_def(cc.curve).ok_or_else(|| export_err(format!("{e}: curve {} is not in the store", cc.curve)))?;
        let (a, b) = view.edge_ends(e).ok_or_else(|| export_err(format!("{e} has no end vertices")))?;
        // The range must end at the vertices: STEP stores no range.
        let curve = def.as_curve();
        let (lo, hi) = (curve.eval(cc.range.lo), curve.eval(cc.range.hi));
        let (pa, pb) = match (view.carrier(a), view.carrier(b)) {
            (Carrier::Point(pa), Carrier::Point(pb)) => (*pa, *pb),
            _ => return Err(export_err(format!("{e}: an end vertex has no point"))),
        };
        let (at_start, at_end) = if cc.sense == Sense::Forward { (lo, hi) } else { (hi, lo) };
        let limit = (view.tolerance_of(e).max(tol) * 100.0).max(1e-9);
        if (at_start - pa).norm() > limit || (at_end - pb).norm() > limit {
            return Err(StepError::GeometryMismatch { id: e.serial(), msg: format!("{e}: the curve range does not end at the vertices of the edge") });
        }
        let cid = match cstep.get(&cc.curve) {
            Some(i) => *i,
            None => {
                let i = wr.curve(def, e)?;
                cstep.insert(cc.curve, i);
                i
            }
        };
        let same = if cc.sense == Sense::Forward { ".T." } else { ".F." };
        let id = wr.add(&format!("EDGE_CURVE('',#{},#{},#{cid},{same})", vstep[&a], vstep[&b]));
        estep.insert(e, id);
    }
    // Faces.
    let mut sstep: BTreeMap<EntityId, u64> = BTreeMap::new();
    let mut fstep: BTreeMap<EntityId, u64> = BTreeMap::new();
    for (&f, &(_, flip)) in &faces {
        let Carrier::Surface(sc) = view.carrier(f) else { return Err(unsupported(f, "a face without a surface")) };
        let def = view.shapes().surface_def(sc.surface).ok_or_else(|| export_err(format!("{f}: surface {} is not in the store", sc.surface)))?;
        let sid = match sstep.get(&sc.surface) {
            Some(i) => *i,
            None => {
                let i = wr.surface(def, f)?;
                sstep.insert(sc.surface, i);
                i
            }
        };
        let mut bounds = Vec::new();
        for (k, lp) in view.loops(f).enumerate() {
            let mut seq: Vec<(u64, bool)> = lp
                .entries
                .iter()
                .filter(|e| !view.flags(e.cell).contains(CellFlags::DEGENERATE))
                .map(|e| (estep[&e.cell], e.side != Side::Right))
                .collect();
            if seq.is_empty() {
                return Err(unsupported(f, "a loop made only of pole edges"));
            }
            if flip {
                seq.reverse();
                for s in &mut seq {
                    s.1 = !s.1;
                }
            }
            let oes: Vec<String> = seq.iter().map(|(e, fwd)| format!("#{}", wr.add(&format!("ORIENTED_EDGE('',*,*,#{e},{})", if *fwd { ".T." } else { ".F." })))).collect();
            let el = wr.add(&format!("EDGE_LOOP('',({}))", oes.join(",")));
            let kind = if k == 0 { "FACE_OUTER_BOUND" } else { "FACE_BOUND" };
            bounds.push(format!("#{}", wr.add(&format!("{kind}('',#{el},.T.)"))));
        }
        let same = (sc.sense == Sense::Forward) != flip;
        let id = wr.add(&format!("ADVANCED_FACE('',({}),#{sid},{})", bounds.join(","), if same { ".T." } else { ".F." }));
        fstep.insert(f, id);
        report.faces += 1;
    }
    report.edges += edges.len();
    report.vertices += vertices.len();
    // Shells and the solid.
    let mut shell_ids: Vec<(u64, bool)> = Vec::new();
    for (si, sh) in shells.iter().enumerate() {
        let list: Vec<String> = faces.iter().filter(|(_, (s, _))| *s == si).map(|(f, _)| format!("#{}", fstep[f])).collect();
        let cs = wr.add(&format!("CLOSED_SHELL('',({}))", list.join(",")));
        let is_void = sh.kind == ShellKind::Void;
        let id = if is_void { wr.add(&format!("ORIENTED_CLOSED_SHELL('',*,#{cs},.F.)")) } else { cs };
        shell_ids.push((id, is_void));
    }
    let Some(&(outer, _)) = shell_ids.first() else { return Err(StepError::Export("the volume has no shell".into())) };
    Ok(if shell_ids.len() == 1 {
        wr.add(&format!("MANIFOLD_SOLID_BREP('',#{outer})"))
    } else {
        let voids: Vec<String> = shell_ids[1..].iter().map(|(i, _)| format!("#{i}")).collect();
        wr.add(&format!("BREP_WITH_VOIDS('',#{outer},({}))", voids.join(",")))
    })
}
