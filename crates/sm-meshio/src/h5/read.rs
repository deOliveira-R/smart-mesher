//! A minimal reader of exactly the subset [`super::write_file`] produces, for round-trip tests.
//! Every access is bounds-checked: corrupt input is an `Err`, never a panic.

use super::{Attr, Dataset, Dtype, Group, H5Error, Obj, SIGNATURE, UNDEF, lookup3};

fn bad<T>(m: &str) -> Result<T, H5Error> {
    Err(H5Error::Malformed(m.into()))
}

struct Cur<'a> {
    b: &'a [u8],
    p: usize,
}

impl<'a> Cur<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], H5Error> {
        let end = self.p.checked_add(n).filter(|e| *e <= self.b.len());
        let Some(end) = end else { return bad("truncated") };
        let s = &self.b[self.p..end];
        self.p = end;
        Ok(s)
    }
    fn u8(&mut self) -> Result<u8, H5Error> {
        Ok(self.take(1)?[0])
    }
    fn u16(&mut self) -> Result<u16, H5Error> {
        let s = self.take(2)?;
        Ok(u16::from_le_bytes([s[0], s[1]]))
    }
    fn u32(&mut self) -> Result<u32, H5Error> {
        let s = self.take(4)?;
        Ok(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
    }
    fn u64(&mut self) -> Result<u64, H5Error> {
        let s = self.take(8)?;
        let mut w = [0u8; 8];
        w.copy_from_slice(s);
        Ok(u64::from_le_bytes(w))
    }
}

impl Dtype {
    /// Decode a datatype message body of the subset; the whole slice must be consumed.
    pub fn decode(b: &[u8]) -> Result<Dtype, H5Error> {
        let t = [Dtype::U8, Dtype::I32, Dtype::I64, Dtype::U64, Dtype::F64];
        for c in &t {
            if c.encode() == b {
                return Ok(c.clone());
            }
        }
        let mut c = Cur { b, p: 0 };
        let head = c.u8()?;
        let flags = c.take(3)?;
        let size = c.u32()?;
        match head {
            0x15 if flags == [0, 0, 0] && c.p == b.len() => Ok(Dtype::Opaque(size)),
            0x3a if flags == [0, 0, 0] => {
                if c.u8()? != 1 {
                    return bad("array rank");
                }
                let n = c.u32()?;
                let base = Dtype::decode(&b[c.p..])?;
                if base != Dtype::U64 || size != 8 * n {
                    return bad("array base");
                }
                Ok(Dtype::ArrayU64(n))
            }
            0x38 => {
                let n = u16::from_le_bytes([flags[0], flags[1]]) as usize;
                let base = Self::fixed(1, false);
                if size != 1 || c.take(base.len())? != base.as_slice() {
                    return bad("enum base");
                }
                let mut names = Vec::new();
                for _ in 0..n {
                    let rest = &b[c.p..];
                    let Some(z) = rest.iter().position(|x| *x == 0) else { return bad("enum name") };
                    let name = std::str::from_utf8(&rest[..z]).map_err(|_| H5Error::Malformed("enum name utf8".into()))?;
                    names.push(name.to_string());
                    c.take(z + 1)?;
                }
                let vals = c.take(n)?;
                if c.p != b.len() {
                    return bad("enum trailing");
                }
                Ok(Dtype::EnumU8(names.into_iter().zip(vals.iter().copied()).collect()))
            }
            _ => bad("unsupported datatype"),
        }
    }
}

fn dims_of(body: &[u8]) -> Result<Vec<u64>, H5Error> {
    let mut c = Cur { b: body, p: 0 };
    if c.u8()? != 2 {
        return bad("dataspace version");
    }
    let rank = c.u8()? as usize;
    let _flags = c.u8()?;
    let ty = c.u8()?;
    if (ty == 0) != (rank == 0) {
        return bad("dataspace type");
    }
    (0..rank).map(|_| c.u64()).collect()
}

fn attr_of(body: &[u8]) -> Result<Attr, H5Error> {
    let mut c = Cur { b: body, p: 0 };
    if c.u8()? != 3 || c.u8()? != 0 {
        return bad("attribute version");
    }
    let (nl, tl, sl) = (c.u16()? as usize, c.u16()? as usize, c.u16()? as usize);
    if c.u8()? != 0 {
        return bad("attribute encoding");
    }
    let name = c.take(nl)?;
    let Some((0, name)) = name.split_last().map(|(z, n)| (*z, n)) else { return bad("attribute name") };
    let name = std::str::from_utf8(name).map_err(|_| H5Error::Malformed("attribute name utf8".into()))?.to_string();
    let dtype = Dtype::decode(c.take(tl)?)?;
    let dims = dims_of(c.take(sl)?)?;
    let data = body[c.p..].to_vec();
    super::check_data(&dtype, &dims, data.len())?;
    Ok(Attr { name, dtype, dims, data })
}

struct Reader<'a> {
    b: &'a [u8],
}

impl Reader<'_> {
    fn object(&self, addr: u64, depth: u32) -> Result<Obj, H5Error> {
        if depth > 32 {
            return bad("tree too deep");
        }
        let Ok(at) = usize::try_from(addr) else { return bad("address") };
        if at >= self.b.len() {
            return bad("address out of file");
        }
        let mut c = Cur { b: self.b, p: at };
        if c.take(4)? != b"OHDR" || c.u8()? != 2 {
            return bad("object header signature");
        }
        let flags = c.u8()?;
        if flags & !3 != 0 {
            return bad("object header flags");
        }
        let w = 1usize << (flags & 3);
        let mut sz = [0u8; 8];
        sz[..w].copy_from_slice(c.take(w)?);
        let size = usize::try_from(u64::from_le_bytes(sz)).map_err(|_| H5Error::Malformed("chunk size".into()))?;
        let msgs = c.take(size)?;
        let end = c.p;
        let ck = c.u32()?;
        if lookup3(&self.b[at..end]) != ck {
            return bad("object header checksum");
        }
        let mut m = Cur { b: msgs, p: 0 };
        let (mut dims, mut dtype, mut layout) = (None, None, None);
        let (mut links, mut is_group, mut attrs) = (Vec::new(), false, Vec::new());
        while m.p < msgs.len() {
            let ty = m.u8()?;
            let n = m.u16()? as usize;
            if m.u8()? != 0 {
                return bad("message flags");
            }
            let body = m.take(n)?;
            match ty {
                0x01 => dims = Some(dims_of(body)?),
                0x03 => dtype = Some(Dtype::decode(body)?),
                0x08 => layout = Some(body),
                0x02 | 0x0a => is_group = true,
                0x06 => {
                    let mut l = Cur { b: body, p: 0 };
                    if l.u8()? != 1 || l.u8()? != 0 {
                        return bad("link version");
                    }
                    let nl = l.u8()? as usize;
                    let name = std::str::from_utf8(l.take(nl)?).map_err(|_| H5Error::Malformed("link name utf8".into()))?.to_string();
                    links.push((name, l.u64()?));
                    is_group = true;
                }
                0x0c => attrs.push(attr_of(body)?),
                _ => return bad("unsupported message"),
            }
        }
        if is_group {
            let mut g = Group { attrs, ..Group::default() };
            for (name, a) in links {
                g.children.insert(name, self.object(a, depth + 1)?);
            }
            return Ok(Obj::Group(g));
        }
        match (dtype, dims, layout) {
            (Some(t), None, None) => Ok(Obj::Type(t)),
            (Some(dtype), Some(dims), Some(l)) => {
                let mut c = Cur { b: l, p: 0 };
                if c.u8()? != 3 || c.u8()? != 1 {
                    return bad("layout");
                }
                let (a, n) = (c.u64()?, c.u64()?);
                let data = if a == UNDEF {
                    if n != 0 {
                        return bad("undefined data address");
                    }
                    Vec::new()
                } else {
                    let (Ok(a), Ok(n)) = (usize::try_from(a), usize::try_from(n)) else { return bad("data address") };
                    let Some(e) = a.checked_add(n).filter(|e| *e <= self.b.len()) else { return bad("data out of file") };
                    self.b[a..e].to_vec()
                };
                super::check_data(&dtype, &dims, data.len())?;
                Ok(Obj::Dataset(Dataset { dtype, dims, data, attrs }))
            }
            _ => bad("object kind"),
        }
    }
}

/// Parse a file of the written subset into its tree.
pub fn read_file(b: &[u8]) -> Result<Group, H5Error> {
    let mut c = Cur { b, p: 0 };
    if c.take(8)? != SIGNATURE || c.take(4)? != [2, 8, 8, 0] {
        return bad("superblock");
    }
    if c.u64()? != 0 || c.u64()? != UNDEF {
        return bad("superblock addresses");
    }
    let eof = c.u64()?;
    let root = c.u64()?;
    let end = c.p;
    if lookup3(&b[..end]) != c.u32()? {
        return bad("superblock checksum");
    }
    if eof != b.len() as u64 {
        return bad("end-of-file address");
    }
    match (Reader { b }).object(root, 0)? {
        Obj::Group(g) => Ok(g),
        _ => bad("root is not a group"),
    }
}

impl Group {
    /// The object at `path` (`/`-separated, no leading slash).
    pub fn find(&self, path: &str) -> Option<&Obj> {
        let mut cur: &Group = self;
        let parts: Vec<&str> = path.split('/').filter(|s| !s.is_empty()).collect();
        for (i, p) in parts.iter().enumerate() {
            let o = cur.children.get(*p)?;
            if i + 1 == parts.len() {
                return Some(o);
            }
            match o {
                Obj::Group(g) => cur = g,
                _ => return None,
            }
        }
        None
    }

    /// The dataset at `path`.
    pub fn dataset(&self, path: &str) -> Option<&Dataset> {
        match self.find(path)? {
            Obj::Dataset(d) => Some(d),
            _ => None,
        }
    }
}

impl Dataset {
    /// Values as `u64` (the dataset must be `U64`).
    pub fn as_u64(&self) -> Vec<u64> {
        self.data.chunks_exact(8).map(|c| u64::from_le_bytes([c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]])).collect()
    }
    /// Values as `i64`.
    pub fn as_i64(&self) -> Vec<i64> {
        self.as_u64().into_iter().map(|x| x as i64).collect()
    }
    /// Values as `i32`.
    pub fn as_i32(&self) -> Vec<i32> {
        self.data.chunks_exact(4).map(|c| i32::from_le_bytes([c[0], c[1], c[2], c[3]])).collect()
    }
    /// Values as `f64`.
    pub fn as_f64(&self) -> Vec<f64> {
        self.as_u64().into_iter().map(f64::from_bits).collect()
    }
}
