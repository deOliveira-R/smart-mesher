//! ASCII polyMesh files: `points faces owner neighbour boundary` and, when present, `cellZones`
//! and `faceZones`. Output is a pure function of the [`PolyMesh`]: formatting runs over fixed
//! 4096-line chunks that are concatenated in order.

use super::{FoamError, FoamOptions, FoamReport, FoamPatchType, PolyMesh, build_polymesh};
use crate::error::{MeshioError, Result};
use rayon::prelude::*;
use sm_mesh_api::MeshRead;
use std::io::Write;
use std::path::Path;

/// Lines per formatting task.
const LINES: usize = 4096;
/// Zone label lists up to this length are written inline (`List<label> 4(0 1 2 3)`).
const INLINE: usize = 10;

/// One polyMesh file: its name (`points`, `faces`, ...) and bytes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FoamFile {
    /// File name inside `constant/polyMesh`.
    pub name: String,
    /// ASCII contents.
    pub bytes: Vec<u8>,
}

const BANNER: &str = r"/*--------------------------------*- C++ -*----------------------------------*\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     | Written by sm-meshio (smart-mesh)
    \\  /    A nd           |
     \\/     M anipulation  |
\*---------------------------------------------------------------------------*/
";
const RULE: &str = "// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //\n";
const FOOTER: &str = "\n// ************************************************************************* //\n";

fn header(class: &str, object: &str, note: Option<&str>) -> Vec<u8> {
    let mut s = String::from(BANNER);
    s.push_str("FoamFile\n{\n    version     2.0;\n    format      ascii;\n    arch        \"LSB;label=32;scalar=64\";\n");
    s.push_str(&format!("    class       {class};\n"));
    if let Some(n) = note {
        s.push_str(&format!("    note        \"{n}\";\n"));
    }
    s.push_str(&format!("    location    \"constant/polyMesh\";\n    object      {object};\n}}\n"));
    s.push_str(RULE);
    s.push('\n');
    s.into_bytes()
}

/// `n` lines, line `i` produced by `f(i, buf)`, formatted in parallel chunks, concatenated in order.
fn lines(n: usize, f: impl Fn(usize, &mut Vec<u8>) + Sync) -> Vec<u8> {
    let parts: Vec<Vec<u8>> = (0..n.div_ceil(LINES))
        .into_par_iter()
        .map(|k| {
            let mut b = Vec::with_capacity(LINES * 24);
            for i in k * LINES..((k + 1) * LINES).min(n) {
                f(i, &mut b);
            }
            b
        })
        .collect();
    let mut out = Vec::with_capacity(parts.iter().map(Vec::len).sum());
    for p in parts {
        out.extend_from_slice(&p);
    }
    out
}

fn push_u(b: &mut Vec<u8>, mut v: u64) {
    let mut t = [0u8; 20];
    let mut i = t.len();
    loop {
        i -= 1;
        t[i] = b'0' + (v % 10) as u8;
        v /= 10;
        if v == 0 {
            break;
        }
    }
    b.extend_from_slice(&t[i..]);
}

/// Shortest round-trip text of a finite `f64` (Rust's `{:?}`).
fn push_f(b: &mut Vec<u8>, v: f64) {
    let _ = write!(b, "{v:?}");
}

fn list(items: usize, body: Vec<u8>) -> Vec<u8> {
    let mut out = Vec::with_capacity(body.len() + 32);
    out.extend_from_slice(format!("{items}\n(\n").as_bytes());
    out.extend_from_slice(&body);
    out.extend_from_slice(b")\n");
    out
}

fn labels_inline_or_block(v: &[u32], ty: &str, fmt: impl Fn(usize, &mut Vec<u8>) + Sync) -> Vec<u8> {
    if v.len() <= INLINE {
        let mut b = format!("List<{ty}> {}(", v.len()).into_bytes();
        for i in 0..v.len() {
            if i > 0 {
                b.push(b' ');
            }
            fmt(i, &mut b);
        }
        b.extend_from_slice(b")");
        b
    } else {
        let mut b = format!("List<{ty}> \n{}\n(\n", v.len()).into_bytes();
        b.extend_from_slice(&lines(v.len(), |i, b| {
            fmt(i, b);
            b.push(b'\n');
        }));
        b.extend_from_slice(b")\n;");
        b
    }
}

impl PolyMesh {
    /// The polyMesh files, in this order: `points`, `faces`, `owner`, `neighbour`, `boundary`,
    /// then `cellZones` and `faceZones` only when non-empty. Scalars are written in Rust's
    /// shortest round-trip form, so [`parse_polymesh`](super::parse_polymesh) restores the points
    /// bit for bit. A NaN or infinite coordinate is [`FoamError::NonFinite`]; an inconsistent
    /// mesh (CSR, counts) is [`MeshioError::Inconsistent`].
    pub fn files(&self) -> Result<Vec<FoamFile>> {
        let (nf, ni) = (self.n_faces(), self.n_internal());
        if self.face_offsets.len() != nf + 1
            || self.face_offsets.first() != Some(&0)
            || self.face_offsets.last().copied() != Some(self.face_points.len() as u64)
            || self.face_offsets.windows(2).any(|w| w[0] > w[1])
            || ni > nf
        {
            return Err(MeshioError::Inconsistent("polyMesh face arrays are inconsistent".into()));
        }
        if let Some(i) = self.points.par_iter().position_first(|p| !(p[0].is_finite() && p[1].is_finite() && p[2].is_finite())) {
            return Err(MeshioError::Foam(FoamError::NonFinite { point: i as u32 }));
        }
        let note = format!("nPoints:{}  nCells:{}  nFaces:{}  nInternalFaces:{}", self.points.len(), self.n_cells, nf, ni);
        let file = |name: &str, class: &str, note: Option<&str>, body: Vec<u8>| {
            let mut bytes = header(class, name, note);
            bytes.extend_from_slice(&body);
            bytes.extend_from_slice(FOOTER.as_bytes());
            FoamFile { name: name.to_string(), bytes }
        };
        let mut out = Vec::with_capacity(7);
        let pts = lines(self.points.len(), |i, b| {
            let p = self.points[i];
            b.push(b'(');
            push_f(b, p[0]);
            b.push(b' ');
            push_f(b, p[1]);
            b.push(b' ');
            push_f(b, p[2]);
            b.extend_from_slice(b")\n");
        });
        out.push(file("points", "vectorField", None, list(self.points.len(), pts)));
        let faces = lines(nf, |i, b| {
            let f = self.face(i);
            push_u(b, f.len() as u64);
            b.push(b'(');
            for (k, v) in f.iter().enumerate() {
                if k > 0 {
                    b.push(b' ');
                }
                push_u(b, u64::from(*v));
            }
            b.extend_from_slice(b")\n");
        });
        out.push(file("faces", "faceList", Some(&note), list(nf, faces)));
        let owner = lines(nf, |i, b| {
            push_u(b, u64::from(self.owner[i]));
            b.push(b'\n');
        });
        out.push(file("owner", "labelList", Some(&note), list(nf, owner)));
        let nb = lines(ni, |i, b| {
            push_u(b, u64::from(self.neighbour[i]));
            b.push(b'\n');
        });
        out.push(file("neighbour", "labelList", Some(&note), list(ni, nb)));

        let mut bd = format!("{}\n(\n", self.patches.len());
        for p in &self.patches {
            bd.push_str(&format!("    {}\n    {{\n        type            {};\n", p.name, p.patch_type.as_str()));
            if p.patch_type == FoamPatchType::Wall {
                bd.push_str("        inGroups        List<word> 1(wall);\n");
            }
            bd.push_str(&format!("        nFaces          {};\n        startFace       {};\n    }}\n", p.n_faces, p.start_face));
        }
        bd.push_str(")\n");
        out.push(file("boundary", "polyBoundaryMesh", None, bd.into_bytes()));

        if !self.cell_zones.is_empty() {
            let mut b = format!("{}\n(\n", self.cell_zones.len()).into_bytes();
            for z in &self.cell_zones {
                b.extend_from_slice(format!("{}\n{{\n    type            cellZone;\n    cellLabels      ", z.name).as_bytes());
                b.extend_from_slice(&labels_inline_or_block(&z.labels, "label", |i, b| push_u(b, u64::from(z.labels[i]))));
                b.extend_from_slice(b";\n}\n");
            }
            b.extend_from_slice(b")\n");
            out.push(file("cellZones", "regIOobject", None, b));
        }
        if !self.face_zones.is_empty() {
            let mut b = format!("{}\n(\n", self.face_zones.len()).into_bytes();
            for z in &self.face_zones {
                b.extend_from_slice(format!("{}\n{{\n    type            faceZone;\n    faceLabels      ", z.name).as_bytes());
                b.extend_from_slice(&labels_inline_or_block(&z.labels, "label", |i, b| push_u(b, u64::from(z.labels[i]))));
                b.extend_from_slice(b";\n    flipMap         ");
                b.extend_from_slice(&labels_inline_or_block(&z.labels, "bool", |i, b| b.push(if z.flips[i] { b'1' } else { b'0' })));
                b.extend_from_slice(b";\n}\n");
            }
            b.extend_from_slice(b")\n");
            out.push(file("faceZones", "regIOobject", None, b));
        }
        Ok(out)
    }

    /// Write [`PolyMesh::files`] into `dir` (normally `<case>/constant/polyMesh`), creating it.
    /// A `cellZones` or `faceZones` file left over from an earlier export is removed when this
    /// mesh has none.
    pub fn write_dir(&self, dir: &Path) -> Result<()> {
        let files = self.files()?;
        std::fs::create_dir_all(dir)?;
        for f in &files {
            std::fs::write(dir.join(&f.name), &f.bytes)?;
        }
        for stale in ["cellZones", "faceZones"] {
            if !files.iter().any(|f| f.name == stale) {
                match std::fs::remove_file(dir.join(stale)) {
                    Ok(()) => {}
                    Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                    Err(e) => return Err(e.into()),
                }
            }
        }
        Ok(())
    }
}

/// Build the polyMesh of `mesh` and write it to `dir` (normally `<case>/constant/polyMesh`).
/// Returns the report (counts, patches, losses).
pub fn write_polymesh(mesh: &dyn MeshRead, opts: &FoamOptions, dir: &Path) -> Result<FoamReport> {
    let b = build_polymesh(mesh, opts)?;
    b.mesh.write_dir(dir)?;
    Ok(b.report)
}
