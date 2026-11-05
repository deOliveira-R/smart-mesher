//! A parser for the msh4 files [`super::write_msh4`] produces (ASCII and binary), for round-trip
//! tests. It is NOT a general Gmsh reader: only the five sections the writer emits, only the
//! element types it writes, no `$Periodic`, `$NodeData` or partitioned entities.

use super::GmshError;

/// One entity record of `$Entities`.
#[derive(Clone, Debug, PartialEq)]
pub struct Msh4Entity {
    /// Entity tag.
    pub tag: i32,
    /// Bounding box minimum (the point itself for dimension 0).
    pub min: [f64; 3],
    /// Bounding box maximum.
    pub max: [f64; 3],
    /// Physical group tags.
    pub physical: Vec<i32>,
    /// Signed boundary entity tags (empty for points).
    pub boundary: Vec<i32>,
}

/// The nodes of one entity.
#[derive(Clone, Debug, PartialEq)]
pub struct Msh4NodeBlock {
    /// Entity dimension.
    pub dim: u8,
    /// Entity tag.
    pub entity: i32,
    /// Node tags.
    pub tags: Vec<u64>,
    /// Node coordinates, one per tag.
    pub coords: Vec<[f64; 3]>,
}

/// The elements of one entity and type.
#[derive(Clone, Debug, PartialEq)]
pub struct Msh4ElementBlock {
    /// Entity dimension.
    pub dim: u8,
    /// Entity tag.
    pub entity: i32,
    /// Gmsh element type code.
    pub etype: u8,
    /// Element tags.
    pub tags: Vec<u64>,
    /// Nodes per element of this type.
    pub nodes_per_element: usize,
    /// Node tags, `nodes_per_element` per element.
    pub nodes: Vec<u64>,
}

impl Msh4ElementBlock {
    /// The node tags of element `i`.
    pub fn element(&self, i: usize) -> &[u64] {
        &self.nodes[i * self.nodes_per_element..(i + 1) * self.nodes_per_element]
    }
}

/// A parsed msh4 file.
#[derive(Clone, Debug, PartialEq)]
pub struct Msh4Data {
    /// The file was binary.
    pub binary: bool,
    /// `(dimension, tag, name)` of `$PhysicalNames`.
    pub physical_names: Vec<(u8, u32, String)>,
    /// Entities by dimension.
    pub entities: [Vec<Msh4Entity>; 4],
    /// Node blocks in file order.
    pub node_blocks: Vec<Msh4NodeBlock>,
    /// Element blocks in file order.
    pub element_blocks: Vec<Msh4ElementBlock>,
}

impl Msh4Data {
    /// Total node count.
    pub fn node_count(&self) -> usize {
        self.node_blocks.iter().map(|b| b.tags.len()).sum()
    }
    /// Element counts by Gmsh type.
    pub fn elements_by_type(&self) -> std::collections::BTreeMap<u8, u64> {
        let mut m = std::collections::BTreeMap::new();
        for b in &self.element_blocks {
            *m.entry(b.etype).or_default() += b.tags.len() as u64;
        }
        m
    }
}

fn nodes_per(ty: u8) -> Option<usize> {
    Some(match ty {
        1 => 2,
        2 => 3,
        3 | 4 => 4,
        5 => 8,
        6 => 6,
        7 => 5,
        15 => 1,
        _ => return None,
    })
}

fn bad<T>(m: impl Into<String>) -> Result<T, GmshError> {
    Err(GmshError::Malformed(m.into()))
}

struct P<'a> {
    b: &'a [u8],
    p: usize,
    binary: bool,
}

impl<'a> P<'a> {
    fn rest(&self) -> usize {
        self.b.len() - self.p
    }
    fn take(&mut self, n: usize) -> Result<&'a [u8], GmshError> {
        if n > self.rest() {
            return bad("truncated");
        }
        let s = &self.b[self.p..self.p + n];
        self.p += n;
        Ok(s)
    }
    fn skip_ws(&mut self) {
        while self.p < self.b.len() && self.b[self.p].is_ascii_whitespace() {
            self.p += 1;
        }
    }
    fn line(&mut self) -> Result<&'a str, GmshError> {
        let start = self.p;
        while self.p < self.b.len() && self.b[self.p] != b'\n' {
            self.p += 1;
        }
        let s = std::str::from_utf8(&self.b[start..self.p]).map_err(|_| GmshError::Malformed("non-utf8 line".into()))?;
        if self.p < self.b.len() {
            self.p += 1;
        }
        Ok(s.trim_end_matches('\r'))
    }
    fn tok(&mut self) -> Result<&'a str, GmshError> {
        self.skip_ws();
        let start = self.p;
        while self.p < self.b.len() && !self.b[self.p].is_ascii_whitespace() {
            self.p += 1;
        }
        if start == self.p {
            return bad("unexpected end of data");
        }
        std::str::from_utf8(&self.b[start..self.p]).map_err(|_| GmshError::Malformed("non-utf8 token".into()))
    }
    fn i32(&mut self) -> Result<i32, GmshError> {
        if self.binary {
            let s = self.take(4)?;
            Ok(i32::from_le_bytes([s[0], s[1], s[2], s[3]]))
        } else {
            self.tok()?.parse().map_err(|_| GmshError::Malformed("bad integer".into()))
        }
    }
    fn sz(&mut self) -> Result<u64, GmshError> {
        if self.binary {
            let s = self.take(8)?;
            let mut w = [0u8; 8];
            w.copy_from_slice(s);
            Ok(u64::from_le_bytes(w))
        } else {
            self.tok()?.parse().map_err(|_| GmshError::Malformed("bad size".into()))
        }
    }
    fn f64(&mut self) -> Result<f64, GmshError> {
        if self.binary {
            let s = self.take(8)?;
            let mut w = [0u8; 8];
            w.copy_from_slice(s);
            Ok(f64::from_le_bytes(w))
        } else {
            self.tok()?.parse().map_err(|_| GmshError::Malformed("bad real".into()))
        }
    }
    /// A count whose items need at least `each` bytes in binary (2 in ASCII).
    fn count(&mut self, each: usize) -> Result<usize, GmshError> {
        let n = self.sz()?;
        let per = if self.binary { each.max(1) } else { 2 };
        match usize::try_from(n) {
            Ok(n) if n <= self.rest() / per => Ok(n),
            _ => bad("count exceeds the data"),
        }
    }
    fn end(&mut self, name: &str) -> Result<(), GmshError> {
        self.skip_ws();
        if self.line()? != format!("$End{name}") {
            return bad(format!("missing $End{name}"));
        }
        Ok(())
    }
}

/// Parse an msh4 file written by [`super::write_msh4`] (ASCII or binary).
///
/// Not a general reader (see the module docs). Corrupt input is an error, never a panic.
pub fn parse_msh4(bytes: &[u8]) -> Result<Msh4Data, GmshError> {
    let mut p = P { b: bytes, p: 0, binary: false };
    if p.line()? != "$MeshFormat" {
        return bad("missing $MeshFormat");
    }
    let head = p.line()?;
    let mut it = head.split_whitespace();
    if it.next() != Some("4.1") {
        return bad("only msh 4.1 is read");
    }
    let binary = match it.next() {
        Some("0") => false,
        Some("1") => true,
        _ => return bad("bad file-type flag"),
    };
    if it.next() != Some("8") || it.next().is_some() {
        return bad("only data-size 8 is read");
    }
    p.binary = binary;
    if binary {
        if p.i32()? != 1 {
            return bad("endianness check word is not 1");
        }
    }
    p.end("MeshFormat")?;
    let mut d = Msh4Data { binary, physical_names: Vec::new(), entities: Default::default(), node_blocks: Vec::new(), element_blocks: Vec::new() };
    loop {
        p.skip_ws();
        if p.rest() == 0 {
            break;
        }
        let name = p.line()?;
        match name {
            "$PhysicalNames" => {
                let n: usize = p.line()?.trim().parse().map_err(|_| GmshError::Malformed("bad name count".into()))?;
                for _ in 0..n {
                    let l = p.line()?;
                    let mut it = l.splitn(3, ' ');
                    let (Some(a), Some(b), Some(c)) = (it.next(), it.next(), it.next()) else { return bad("bad physical name line") };
                    let (dim, tag): (u8, u32) = (a.parse().map_err(|_| GmshError::Malformed("bad dim".into()))?, b.parse().map_err(|_| GmshError::Malformed("bad tag".into()))?);
                    let Some(nm) = c.strip_prefix('"').and_then(|c| c.strip_suffix('"')) else { return bad("physical name is not quoted") };
                    if dim > 3 {
                        return bad("physical group dimension");
                    }
                    d.physical_names.push((dim, tag, nm.to_string()));
                }
                p.end("PhysicalNames")?;
            }
            "$Entities" => {
                let mut n = [0usize; 4];
                for c in &mut n {
                    *c = p.count(8)?;
                }
                for (dim, &cnt) in n.iter().enumerate() {
                    for _ in 0..cnt {
                        let tag = p.i32()?;
                        let mut min = [0.0; 3];
                        let mut max = [0.0; 3];
                        for m in &mut min {
                            *m = p.f64()?;
                        }
                        if dim == 0 {
                            max = min;
                        } else {
                            for m in &mut max {
                                *m = p.f64()?;
                            }
                        }
                        let np = p.count(4)?;
                        let physical = (0..np).map(|_| p.i32()).collect::<Result<Vec<_>, _>>()?;
                        let boundary = if dim > 0 {
                            let nb = p.count(4)?;
                            (0..nb).map(|_| p.i32()).collect::<Result<Vec<_>, _>>()?
                        } else {
                            Vec::new()
                        };
                        d.entities[dim].push(Msh4Entity { tag, min, max, physical, boundary });
                    }
                }
                p.end("Entities")?;
            }
            "$Nodes" => {
                let blocks = p.count(20)?;
                let total = p.sz()?;
                let (_min, _max) = (p.sz()?, p.sz()?);
                let mut seen = 0u64;
                for _ in 0..blocks {
                    let dim = p.i32()?;
                    let entity = p.i32()?;
                    if !(0..=3).contains(&dim) || p.i32()? != 0 {
                        return bad("node block dimension or parametric flag");
                    }
                    let n = p.count(32)?;
                    let tags = (0..n).map(|_| p.sz()).collect::<Result<Vec<_>, _>>()?;
                    let mut coords = Vec::with_capacity(n);
                    for _ in 0..n {
                        coords.push([p.f64()?, p.f64()?, p.f64()?]);
                    }
                    seen += n as u64;
                    d.node_blocks.push(Msh4NodeBlock { dim: dim as u8, entity, tags, coords });
                }
                if seen != total {
                    return bad("node count does not match the blocks");
                }
                p.end("Nodes")?;
            }
            "$Elements" => {
                let blocks = p.count(20)?;
                let total = p.sz()?;
                let (_min, _max) = (p.sz()?, p.sz()?);
                let mut seen = 0u64;
                for _ in 0..blocks {
                    let dim = p.i32()?;
                    let entity = p.i32()?;
                    let ty = p.i32()?;
                    let Some(npe) = u8::try_from(ty).ok().and_then(nodes_per) else { return bad("unsupported element type") };
                    if !(0..=3).contains(&dim) {
                        return bad("element block dimension");
                    }
                    let n = p.count(8 * (1 + npe))?;
                    let mut tags = Vec::with_capacity(n);
                    let mut nodes = Vec::with_capacity(n * npe);
                    for _ in 0..n {
                        tags.push(p.sz()?);
                        for _ in 0..npe {
                            nodes.push(p.sz()?);
                        }
                    }
                    seen += n as u64;
                    d.element_blocks.push(Msh4ElementBlock { dim: dim as u8, entity, etype: ty as u8, tags, nodes_per_element: npe, nodes });
                }
                if seen != total {
                    return bad("element count does not match the blocks");
                }
                p.end("Elements")?;
            }
            other => return bad(format!("unexpected section {other:?}")),
        }
    }
    Ok(d)
}
