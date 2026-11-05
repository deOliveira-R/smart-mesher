//! A minimal pure-Rust HDF5 writer: exactly what the DAGMC `.h5m` exporter needs.
//!
//! No `sm-*` imports, so it can be lifted into its own crate unchanged. The subset:
//! superblock v2 (Jenkins lookup3 checksums), object headers v2 (`OHDR`, no timestamps, no
//! creation order), groups with compact link messages (children sorted by name), contiguous
//! datasets (layout v3), committed (named) datatypes, scalar and N-d attributes (message v3),
//! and the datatypes [`Dtype`] lists. No chunking, filters, variable-length data or timestamps.
//! The byte stream is a pure function of the tree: addresses come from one layout pass (an
//! object, its raw data, then its children in name order).
//!
//! The format facts used here are from the public HDF5 File Format Specification, version 3.0
//! (sections II.A superblock, IV.A.1 object header v2, IV.A.2 messages, III.B Jenkins lookup3).
//! Anything outside the subset is [`H5Error::Unsupported`].

use std::collections::BTreeMap;

#[cfg(any(test, feature = "testing"))]
pub mod read;

/// Errors from the HDF5 writer (and the test reader).
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum H5Error {
    /// The tree uses something outside the written subset (a name or message too long, ...).
    #[error("hdf5: unsupported: {0}")]
    Unsupported(String),
    /// A file read back is not valid for the subset.
    #[error("hdf5: malformed: {0}")]
    Malformed(String),
}

const UNDEF: u64 = u64::MAX;
const SIGNATURE: [u8; 8] = [0x89, b'H', b'D', b'F', b'\r', b'\n', 0x1a, b'\n'];

/// An element datatype of the subset.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Dtype {
    /// Unsigned 8-bit integer, little endian.
    #[allow(dead_code)] // constructed by tests and the reader only
    U8,
    /// Signed 32-bit integer, little endian.
    I32,
    /// Signed 64-bit integer, little endian.
    I64,
    /// Unsigned 64-bit integer, little endian.
    U64,
    /// IEEE binary64, little endian.
    F64,
    /// Fixed-size opaque bytes with an empty tag.
    Opaque(u32),
    /// Array of `n` little-endian `u64` (`H5T_ARRAY`).
    ArrayU64(u32),
    /// Enumeration over `u8` with `(name, value)` members.
    EnumU8(Vec<(String, u8)>),
}

impl Dtype {
    /// Size of one element in bytes.
    pub fn size(&self) -> usize {
        match self {
            Dtype::U8 | Dtype::EnumU8(_) => 1,
            Dtype::I32 => 4,
            Dtype::I64 | Dtype::U64 | Dtype::F64 => 8,
            Dtype::Opaque(n) => *n as usize,
            Dtype::ArrayU64(n) => 8 * *n as usize,
        }
    }

    fn fixed(size: u32, signed: bool) -> Vec<u8> {
        let mut m = vec![0x10, if signed { 0x08 } else { 0 }, 0, 0];
        m.extend_from_slice(&size.to_le_bytes());
        m.extend_from_slice(&0u16.to_le_bytes()); // bit offset
        m.extend_from_slice(&((size * 8) as u16).to_le_bytes()); // precision
        m
    }

    /// The datatype message body.
    pub fn encode(&self) -> Vec<u8> {
        match self {
            Dtype::U8 => Self::fixed(1, false),
            Dtype::I32 => Self::fixed(4, true),
            Dtype::I64 => Self::fixed(8, true),
            Dtype::U64 => Self::fixed(8, false),
            Dtype::F64 => {
                let mut m = vec![0x11, 0x20, 63, 0];
                m.extend_from_slice(&8u32.to_le_bytes());
                m.extend_from_slice(&0u16.to_le_bytes()); // bit offset
                m.extend_from_slice(&64u16.to_le_bytes()); // precision
                m.extend_from_slice(&[52, 11, 0, 52]); // exp loc, exp size, mant loc, mant size
                m.extend_from_slice(&1023u32.to_le_bytes());
                m
            }
            Dtype::Opaque(n) => {
                let mut m = vec![0x15, 0, 0, 0];
                m.extend_from_slice(&n.to_le_bytes());
                m
            }
            Dtype::ArrayU64(n) => {
                let mut m = vec![0x3a, 0, 0, 0];
                m.extend_from_slice(&(8 * n).to_le_bytes());
                m.push(1); // rank
                m.extend_from_slice(&n.to_le_bytes());
                m.extend_from_slice(&Self::fixed(8, false));
                m
            }
            Dtype::EnumU8(members) => {
                let mut m = vec![0x38];
                m.extend_from_slice(&(members.len() as u16).to_le_bytes());
                m.push(0);
                m.extend_from_slice(&1u32.to_le_bytes());
                m.extend_from_slice(&Self::fixed(1, false));
                for (name, _) in members {
                    m.extend_from_slice(name.as_bytes());
                    m.push(0);
                }
                for (_, v) in members {
                    m.push(*v);
                }
                m
            }
        }
    }
}

/// An attribute: scalar when `dims` is empty, else N-d.
#[derive(Clone, Debug, PartialEq)]
pub struct Attr {
    pub name: String,
    pub dtype: Dtype,
    pub dims: Vec<u64>,
    pub data: Vec<u8>,
}

impl Attr {
    /// A scalar `i32`.
    pub fn i32(name: &str, v: i32) -> Self {
        Self { name: name.into(), dtype: Dtype::I32, dims: vec![], data: v.to_le_bytes().to_vec() }
    }
    /// A scalar `i64`.
    pub fn i64(name: &str, v: i64) -> Self {
        Self { name: name.into(), dtype: Dtype::I64, dims: vec![], data: v.to_le_bytes().to_vec() }
    }
    /// A scalar `u64`.
    pub fn u64(name: &str, v: u64) -> Self {
        Self { name: name.into(), dtype: Dtype::U64, dims: vec![], data: v.to_le_bytes().to_vec() }
    }
    /// A scalar of any datatype from raw little-endian bytes.
    pub fn raw(name: &str, dtype: Dtype, data: Vec<u8>) -> Self {
        Self { name: name.into(), dtype, dims: vec![], data }
    }
}

/// A contiguous dataset.
#[derive(Clone, Debug, PartialEq)]
pub struct Dataset {
    pub dtype: Dtype,
    pub dims: Vec<u64>,
    pub data: Vec<u8>,
    pub attrs: Vec<Attr>,
}

impl Dataset {
    /// A dataset from raw bytes (`dims` product times element size must equal `data.len()`).
    pub fn raw(dtype: Dtype, dims: Vec<u64>, data: Vec<u8>) -> Self {
        Self { dtype, dims, data, attrs: Vec::new() }
    }
    /// `u64` values with the given shape.
    pub fn u64s(dims: Vec<u64>, v: &[u64]) -> Self {
        Self::raw(Dtype::U64, dims, v.iter().flat_map(|x| x.to_le_bytes()).collect())
    }
    /// `i64` values with the given shape.
    pub fn i64s(dims: Vec<u64>, v: &[i64]) -> Self {
        Self::raw(Dtype::I64, dims, v.iter().flat_map(|x| x.to_le_bytes()).collect())
    }
    /// `i32` values with the given shape.
    pub fn i32s(dims: Vec<u64>, v: &[i32]) -> Self {
        Self::raw(Dtype::I32, dims, v.iter().flat_map(|x| x.to_le_bytes()).collect())
    }
    /// `f64` values with the given shape.
    pub fn f64s(dims: Vec<u64>, v: &[f64]) -> Self {
        Self::raw(Dtype::F64, dims, v.iter().flat_map(|x| x.to_le_bytes()).collect())
    }
    /// The same dataset with an attribute added.
    pub fn with_attr(mut self, a: Attr) -> Self {
        self.attrs.push(a);
        self
    }
}

/// A tree object.
#[derive(Clone, Debug, PartialEq)]
pub enum Obj {
    /// A group.
    Group(Group),
    /// A dataset.
    Dataset(Dataset),
    /// A committed (named) datatype.
    Type(Dtype),
}

/// A group: attributes and children keyed by name (written in sorted order).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Group {
    pub attrs: Vec<Attr>,
    pub children: BTreeMap<String, Obj>,
}

impl Group {
    /// An empty group.
    pub fn new() -> Self {
        Self::default()
    }
    /// The child group `name`, created if absent (a non-group child of that name is replaced).
    pub fn group(&mut self, name: &str) -> &mut Group {
        let e = self.children.entry(name.to_string()).or_insert_with(|| Obj::Group(Group::new()));
        if !matches!(e, Obj::Group(_)) {
            *e = Obj::Group(Group::new());
        }
        match e {
            Obj::Group(g) => g,
            // Replaced by a group just above: statically dead, not reachable from any input.
            _ => unreachable!("normalised to a group above"),
        }
    }

    /// Add or replace a dataset.
    pub fn put_dataset(&mut self, name: &str, d: Dataset) {
        self.children.insert(name.to_string(), Obj::Dataset(d));
    }
    /// Add or replace a committed datatype.
    pub fn put_type(&mut self, name: &str, t: Dtype) {
        self.children.insert(name.to_string(), Obj::Type(t));
    }
    /// Add an attribute.
    pub fn put_attr(&mut self, a: Attr) {
        self.attrs.push(a);
    }
}

// --- Jenkins lookup3 (`hashlittle`, initval 0) ---

fn rot(x: u32, k: u32) -> u32 {
    x.rotate_left(k)
}

pub fn lookup3(key: &[u8]) -> u32 {
    let mut a = 0xdead_beefu32.wrapping_add(key.len() as u32);
    let mut b = a;
    let mut c = a;
    let mut k = key;
    let word = |s: &[u8]| -> u32 {
        let mut w = [0u8; 4];
        w[..s.len()].copy_from_slice(s);
        u32::from_le_bytes(w)
    };
    while k.len() > 12 {
        a = a.wrapping_add(word(&k[0..4]));
        b = b.wrapping_add(word(&k[4..8]));
        c = c.wrapping_add(word(&k[8..12]));
        a = a.wrapping_sub(c);
        a ^= rot(c, 4);
        c = c.wrapping_add(b);
        b = b.wrapping_sub(a);
        b ^= rot(a, 6);
        a = a.wrapping_add(c);
        c = c.wrapping_sub(b);
        c ^= rot(b, 8);
        b = b.wrapping_add(a);
        a = a.wrapping_sub(c);
        a ^= rot(c, 16);
        c = c.wrapping_add(b);
        b = b.wrapping_sub(a);
        b ^= rot(a, 19);
        a = a.wrapping_add(c);
        c = c.wrapping_sub(b);
        c ^= rot(b, 4);
        b = b.wrapping_add(a);
        k = &k[12..];
    }
    if k.is_empty() {
        return c;
    }
    let mut tail = [0u8; 12];
    tail[..k.len()].copy_from_slice(k);
    a = a.wrapping_add(word(&tail[0..4]));
    b = b.wrapping_add(word(&tail[4..8]));
    c = c.wrapping_add(word(&tail[8..12]));
    c ^= b;
    c = c.wrapping_sub(rot(b, 14));
    a ^= c;
    a = a.wrapping_sub(rot(c, 11));
    b ^= a;
    b = b.wrapping_sub(rot(a, 25));
    c ^= b;
    c = c.wrapping_sub(rot(b, 16));
    a ^= c;
    a = a.wrapping_sub(rot(c, 4));
    b ^= a;
    b = b.wrapping_sub(rot(a, 14));
    c ^= b;
    c = c.wrapping_sub(rot(b, 24));
    c
}

// --- message encoding ---

fn push_msg(out: &mut Vec<u8>, ty: u8, body: &[u8]) -> Result<(), H5Error> {
    let n = u16::try_from(body.len()).map_err(|_| H5Error::Unsupported("object header message over 64 KiB".into()))?;
    out.push(ty);
    out.extend_from_slice(&n.to_le_bytes());
    out.push(0); // flags
    out.extend_from_slice(body);
    Ok(())
}

fn dataspace(dims: &[u64]) -> Vec<u8> {
    let mut m = vec![2, dims.len() as u8, 0, if dims.is_empty() { 0 } else { 1 }];
    for d in dims {
        m.extend_from_slice(&d.to_le_bytes());
    }
    m
}

fn check_data(dtype: &Dtype, dims: &[u64], len: usize) -> Result<(), H5Error> {
    let n = dims.iter().try_fold(1u64, |a, d| a.checked_mul(*d)).and_then(|n| n.checked_mul(dtype.size() as u64));
    if n != Some(len as u64) {
        return Err(H5Error::Unsupported(format!("data length {len} does not match dims {dims:?}")));
    }
    Ok(())
}

fn attr_msg(a: &Attr) -> Result<Vec<u8>, H5Error> {
    check_data(&a.dtype, &a.dims, a.data.len())?;
    let (dt, ds) = (a.dtype.encode(), dataspace(&a.dims));
    let mut m = vec![3, 0];
    let name_len = u16::try_from(a.name.len() + 1).map_err(|_| H5Error::Unsupported("attribute name too long".into()))?;
    m.extend_from_slice(&name_len.to_le_bytes());
    m.extend_from_slice(&(dt.len() as u16).to_le_bytes());
    m.extend_from_slice(&(ds.len() as u16).to_le_bytes());
    m.push(0); // ASCII
    m.extend_from_slice(a.name.as_bytes());
    m.push(0);
    m.extend_from_slice(&dt);
    m.extend_from_slice(&ds);
    m.extend_from_slice(&a.data);
    Ok(m)
}

/// The complete object header for `obj`; `data_addr` and `children` (name, address) fill the
/// address fields. Lengths do not depend on the addresses.
fn header(obj: &Obj, data_addr: u64, children: &[(&str, u64)]) -> Result<Vec<u8>, H5Error> {
    let mut msgs = Vec::new();
    let attrs: &[Attr] = match obj {
        Obj::Group(g) => {
            let mut li = vec![0, 0];
            li.extend_from_slice(&UNDEF.to_le_bytes());
            li.extend_from_slice(&UNDEF.to_le_bytes());
            push_msg(&mut msgs, 0x02, &li)?;
            push_msg(&mut msgs, 0x0a, &[0, 0])?;
            for (name, addr) in children {
                if name.is_empty() || name.len() > 255 || name.contains('/') || name.contains('\0') {
                    return Err(H5Error::Unsupported(format!("link name {name:?}")));
                }
                let mut l = vec![1, 0, name.len() as u8];
                l.extend_from_slice(name.as_bytes());
                l.extend_from_slice(&addr.to_le_bytes());
                push_msg(&mut msgs, 0x06, &l)?;
            }
            &g.attrs
        }
        Obj::Dataset(d) => {
            check_data(&d.dtype, &d.dims, d.data.len())?;
            push_msg(&mut msgs, 0x01, &dataspace(&d.dims))?;
            push_msg(&mut msgs, 0x03, &d.dtype.encode())?;
            let mut l = vec![3, 1];
            l.extend_from_slice(&(if d.data.is_empty() { UNDEF } else { data_addr }).to_le_bytes());
            l.extend_from_slice(&(d.data.len() as u64).to_le_bytes());
            push_msg(&mut msgs, 0x08, &l)?;
            &d.attrs
        }
        Obj::Type(t) => {
            push_msg(&mut msgs, 0x03, &t.encode())?;
            &[]
        }
    };
    for a in attrs {
        push_msg(&mut msgs, 0x0c, &attr_msg(a)?)?;
    }
    let (code, w) = match msgs.len() {
        0..=255 => (0u8, 1usize),
        256..=65535 => (1, 2),
        65536..=0xffff_ffff => (2, 4),
        _ => (3, 8),
    };
    let mut out = Vec::with_capacity(msgs.len() + 16);
    out.extend_from_slice(b"OHDR");
    out.push(2);
    out.push(code);
    out.extend_from_slice(&(msgs.len() as u64).to_le_bytes()[..w]);
    out.extend_from_slice(&msgs);
    let ck = lookup3(&out);
    out.extend_from_slice(&ck.to_le_bytes());
    Ok(out)
}

fn data_of(obj: &Obj) -> &[u8] {
    match obj {
        Obj::Dataset(d) => &d.data,
        _ => &[],
    }
}

fn children_of(obj: &Obj) -> Vec<(&String, &Obj)> {
    match obj {
        Obj::Group(g) => g.children.iter().collect(),
        _ => Vec::new(),
    }
}

/// Bytes of `obj`, its data and everything below it.
fn total_len(obj: &Obj) -> Result<u64, H5Error> {
    let kids = children_of(obj);
    let names: Vec<(&str, u64)> = kids.iter().map(|(n, _)| (n.as_str(), 0)).collect();
    let mut n = header(obj, 0, &names)?.len() as u64 + data_of(obj).len() as u64;
    for (_, k) in kids {
        n += total_len(k)?;
    }
    Ok(n)
}

fn emit(obj: &Obj, at: u64, out: &mut Vec<u8>) -> Result<(), H5Error> {
    debug_assert_eq!(out.len() as u64, at);
    let kids = children_of(obj);
    let hlen = header(obj, 0, &kids.iter().map(|(n, _)| (n.as_str(), 0)).collect::<Vec<_>>())?.len() as u64;
    let data_addr = at + hlen;
    let mut next = data_addr + data_of(obj).len() as u64;
    let mut addrs = Vec::with_capacity(kids.len());
    for (name, k) in &kids {
        addrs.push((name.as_str(), next));
        next += total_len(k)?;
    }
    out.extend_from_slice(&header(obj, data_addr, &addrs)?);
    out.extend_from_slice(data_of(obj));
    for (_, k) in kids {
        let a = out.len() as u64;
        emit(k, a, out)?;
    }
    Ok(())
}

/// The bytes of an HDF5 file whose root group is `root`.
pub fn write_file(root: Group) -> Result<Vec<u8>, H5Error> {
    let root = Obj::Group(root);
    let root_addr = 48u64;
    let eof = root_addr + total_len(&root)?;
    let mut out = Vec::with_capacity(eof as usize);
    out.extend_from_slice(&SIGNATURE);
    out.extend_from_slice(&[2, 8, 8, 0]);
    out.extend_from_slice(&0u64.to_le_bytes()); // base address
    out.extend_from_slice(&UNDEF.to_le_bytes()); // superblock extension
    out.extend_from_slice(&eof.to_le_bytes());
    out.extend_from_slice(&root_addr.to_le_bytes());
    let ck = lookup3(&out);
    out.extend_from_slice(&ck.to_le_bytes());
    emit(&root, root_addr, &mut out)?;
    debug_assert_eq!(out.len() as u64, eof);
    Ok(out)
}

#[cfg(test)]
mod tests;
