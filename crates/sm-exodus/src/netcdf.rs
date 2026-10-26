//! A minimal classic-netCDF model and codec: CDF-1 (`Classic`), CDF-2 (`Offset64`) and CDF-5
//! (`Data64`).
//!
//! Only what Exodus II needs: fixed-size variables, text and numeric attributes, one optional
//! unlimited (record) dimension that always holds zero records (`numrecs = 0`). Everything is
//! big-endian; names, attribute values and non-record variables are padded to 4 bytes with zeros.
//!
//! Widths (netCDF file-format BNF): list tags and `nc_type` are 4 bytes; `numrecs`, every element
//! count, name length, `dim_length`, `ndims`, dimension ids and `vsize` are 4 bytes in CDF-1/2 and
//! 8 in CDF-5; `begin` is 4 bytes in CDF-1 and 8 in CDF-2/5.

use crate::error::{ExodusError, Result};
use rayon::prelude::*;
use std::borrow::Cow;
use std::fmt;
use std::io::Write;

const TAG_DIMENSION: u32 = 0x0A;
const TAG_VARIABLE: u32 = 0x0B;
const TAG_ATTRIBUTE: u32 = 0x0C;
const PAR_CHUNK: usize = 1 << 15;

/// On-disk flavour of a classic netCDF file.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum NcFormat {
    /// CDF-1: 32-bit offsets and counts (magic `CDF\x01`).
    Classic,
    /// CDF-2: 64-bit offsets (magic `CDF\x02`).
    Offset64,
    /// CDF-5: 64-bit offsets, counts and sizes, and `Int64` (magic `CDF\x05`).
    Data64,
}

impl NcFormat {
    /// The four magic bytes.
    pub const fn magic(self) -> [u8; 4] {
        let v = match self {
            NcFormat::Classic => 1,
            NcFormat::Offset64 => 2,
            NcFormat::Data64 => 5,
        };
        [b'C', b'D', b'F', v]
    }

    /// Width in bytes of `numrecs`, element counts, dimension ids and `vsize`.
    const fn count_width(self) -> usize {
        match self {
            NcFormat::Data64 => 8,
            _ => 4,
        }
    }

    /// Width in bytes of a variable's `begin` offset.
    const fn offset_width(self) -> usize {
        match self {
            NcFormat::Classic => 4,
            _ => 8,
        }
    }
}

/// External type of a variable or attribute.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum NcType {
    /// 8-bit signed (`NC_BYTE`, code 1).
    Byte,
    /// 8-bit character (`NC_CHAR`, code 2).
    Char,
    /// 16-bit signed (`NC_SHORT`, code 3).
    Short,
    /// 32-bit signed (`NC_INT`, code 4).
    Int,
    /// 32-bit float (`NC_FLOAT`, code 5).
    Float,
    /// 64-bit float (`NC_DOUBLE`, code 6).
    Double,
    /// 64-bit signed (`NC_INT64`, code 10; CDF-5 only).
    Int64,
}

impl NcType {
    /// The `nc_type` code.
    pub const fn code(self) -> u32 {
        match self {
            NcType::Byte => 1,
            NcType::Char => 2,
            NcType::Short => 3,
            NcType::Int => 4,
            NcType::Float => 5,
            NcType::Double => 6,
            NcType::Int64 => 10,
        }
    }

    /// Inverse of [`NcType::code`].
    pub const fn from_code(c: u32) -> Option<Self> {
        Some(match c {
            1 => NcType::Byte,
            2 => NcType::Char,
            3 => NcType::Short,
            4 => NcType::Int,
            5 => NcType::Float,
            6 => NcType::Double,
            10 => NcType::Int64,
            _ => return None,
        })
    }

    /// Size of one element in bytes.
    pub const fn size(self) -> u64 {
        match self {
            NcType::Byte | NcType::Char => 1,
            NcType::Short => 2,
            NcType::Int | NcType::Float => 4,
            NcType::Double | NcType::Int64 => 8,
        }
    }

    /// The CDL spelling (`int`, `double`, ...).
    pub const fn cdl_name(self) -> &'static str {
        match self {
            NcType::Byte => "byte",
            NcType::Char => "char",
            NcType::Short => "short",
            NcType::Int => "int",
            NcType::Float => "float",
            NcType::Double => "double",
            NcType::Int64 => "int64",
        }
    }
}

/// An attribute value.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum NcValue {
    /// `NC_CHAR` text (no terminating NUL is added).
    Text(String),
    /// `NC_BYTE` values.
    Byte(Vec<i8>),
    /// `NC_SHORT` values.
    Short(Vec<i16>),
    /// `NC_INT` values.
    Int(Vec<i32>),
    /// `NC_FLOAT` values.
    Float(Vec<f32>),
    /// `NC_DOUBLE` values.
    Double(Vec<f64>),
    /// `NC_INT64` values (CDF-5 only).
    Int64(Vec<i64>),
}

impl NcValue {
    /// External type of the value.
    pub fn ty(&self) -> NcType {
        match self {
            NcValue::Text(_) => NcType::Char,
            NcValue::Byte(_) => NcType::Byte,
            NcValue::Short(_) => NcType::Short,
            NcValue::Int(_) => NcType::Int,
            NcValue::Float(_) => NcType::Float,
            NcValue::Double(_) => NcType::Double,
            NcValue::Int64(_) => NcType::Int64,
        }
    }

    /// Number of elements.
    pub fn len(&self) -> usize {
        match self {
            NcValue::Text(v) => v.len(),
            NcValue::Byte(v) => v.len(),
            NcValue::Short(v) => v.len(),
            NcValue::Int(v) => v.len(),
            NcValue::Float(v) => v.len(),
            NcValue::Double(v) => v.len(),
            NcValue::Int64(v) => v.len(),
        }
    }

    /// True if the value has no elements.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The text of a [`NcValue::Text`], `None` otherwise.
    pub fn as_text(&self) -> Option<&str> {
        match self {
            NcValue::Text(s) => Some(s),
            _ => None,
        }
    }

    fn encode(&self) -> Vec<u8> {
        match self {
            NcValue::Text(s) => s.as_bytes().to_vec(),
            NcValue::Byte(v) => v.iter().map(|x| *x as u8).collect(),
            NcValue::Short(v) => v.iter().flat_map(|x| x.to_be_bytes()).collect(),
            NcValue::Int(v) => v.iter().flat_map(|x| x.to_be_bytes()).collect(),
            NcValue::Float(v) => v.iter().flat_map(|x| x.to_be_bytes()).collect(),
            NcValue::Double(v) => v.iter().flat_map(|x| x.to_be_bytes()).collect(),
            NcValue::Int64(v) => v.iter().flat_map(|x| x.to_be_bytes()).collect(),
        }
    }

    fn decode(ty: NcType, raw: &[u8]) -> NcValue {
        fn chunks<const N: usize, T>(raw: &[u8], f: impl Fn([u8; N]) -> T) -> Vec<T> {
            raw.as_chunks::<N>().0.iter().map(|c| f(*c)).collect()
        }
        match ty {
            NcType::Char => NcValue::Text(String::from_utf8_lossy(raw).into_owned()),
            NcType::Byte => NcValue::Byte(raw.iter().map(|b| *b as i8).collect()),
            NcType::Short => NcValue::Short(chunks(raw, i16::from_be_bytes)),
            NcType::Int => NcValue::Int(chunks(raw, i32::from_be_bytes)),
            NcType::Float => NcValue::Float(chunks(raw, f32::from_be_bytes)),
            NcType::Double => NcValue::Double(chunks(raw, f64::from_be_bytes)),
            NcType::Int64 => NcValue::Int64(chunks(raw, i64::from_be_bytes)),
        }
    }
}

/// Variable data: big-endian bytes, or a provider called once per write so the writer holds
/// about one variable at a time. The lifetime lets a provider borrow the mesh.
pub enum NcData<'a> {
    /// Big-endian bytes, exactly `elements * size` long (the writer adds the padding).
    Bytes(Vec<u8>),
    /// Called when the variable is written (and by [`NcFile::to_cdl`]).
    Lazy(Box<dyn Fn() -> Vec<u8> + Send + Sync + 'a>),
}

impl fmt::Debug for NcData<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NcData::Bytes(b) => write!(f, "Bytes({} bytes)", b.len()),
            NcData::Lazy(_) => f.write_str("Lazy(..)"),
        }
    }
}

pub(crate) fn par_pack<const N: usize, T: Copy + Send + Sync>(xs: &[T], f: impl Fn(T) -> [u8; N] + Sync) -> Vec<u8> {
    let mut out = vec![0u8; xs.len() * N];
    out.par_chunks_mut(PAR_CHUNK * N).zip(xs.par_chunks(PAR_CHUNK)).for_each(|(o, x)| {
        for (dst, v) in o.as_chunks_mut::<N>().0.iter_mut().zip(x) {
            *dst = f(*v);
        }
    });
    out
}

impl NcData<'_> {
    /// Big-endian `NC_INT` bytes (packed in parallel, order independent).
    pub fn from_i32s(xs: &[i32]) -> NcData<'static> {
        NcData::Bytes(par_pack(xs, i32::to_be_bytes))
    }
    /// Big-endian `NC_INT64` bytes.
    pub fn from_i64s(xs: &[i64]) -> NcData<'static> {
        NcData::Bytes(par_pack(xs, i64::to_be_bytes))
    }
    /// Big-endian `NC_FLOAT` bytes.
    pub fn from_f32s(xs: &[f32]) -> NcData<'static> {
        NcData::Bytes(par_pack(xs, f32::to_be_bytes))
    }
    /// Big-endian `NC_DOUBLE` bytes.
    pub fn from_f64s(xs: &[f64]) -> NcData<'static> {
        NcData::Bytes(par_pack(xs, f64::to_be_bytes))
    }
    /// `NC_CHAR` bytes of a string.
    pub fn from_text(s: &str) -> NcData<'static> {
        NcData::Bytes(s.as_bytes().to_vec())
    }

    fn bytes(&self) -> Cow<'_, [u8]> {
        match self {
            NcData::Bytes(b) => Cow::Borrowed(b),
            NcData::Lazy(f) => Cow::Owned(f()),
        }
    }
}

/// Handle of a dimension of an [`NcFile`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct DimId(usize);

/// Handle of a variable of an [`NcFile`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct VarId(usize);

impl DimId {
    /// Position in declaration order.
    pub fn index(self) -> usize {
        self.0
    }
}

impl VarId {
    /// Position in declaration order.
    pub fn index(self) -> usize {
        self.0
    }
}

/// A declared dimension.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NcDim {
    /// Name.
    pub name: String,
    /// Length; `None` for the unlimited (record) dimension.
    pub len: Option<u64>,
}

/// A declared variable with its attributes and data.
#[derive(Debug)]
pub struct NcVar<'a> {
    /// Name.
    pub name: String,
    /// External type.
    pub ty: NcType,
    /// Dimensions, slowest varying first.
    pub dims: Vec<DimId>,
    /// Attributes in declaration order.
    pub attrs: Vec<(String, NcValue)>,
    data: Option<NcData<'a>>,
}

impl NcVar<'_> {
    /// The data, if set.
    pub fn data(&self) -> Option<&NcData<'_>> {
        self.data.as_ref()
    }
}

/// A classic netCDF file under construction (or just decoded by [`NcFile::read`]).
#[derive(Debug)]
pub struct NcFile<'a> {
    format: NcFormat,
    dims: Vec<NcDim>,
    gattrs: Vec<(String, NcValue)>,
    vars: Vec<NcVar<'a>>,
}

/// Placement of one variable in the file.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NcVarLayout {
    /// Variable name.
    pub name: String,
    /// True if the first dimension is the unlimited one.
    pub is_record: bool,
    /// The `vsize` header field (padded size in bytes; per record for a record variable;
    /// `0xFFFF_FFFF` when clamped in CDF-2).
    pub vsize: u64,
    /// True padded size of the data in bytes (differs from `vsize` only when clamped).
    pub size: u64,
    /// Absolute file offset of the first data byte.
    pub begin: u64,
}

/// Header size and variable placement, computed without any data.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NcLayout {
    /// Format the layout is for.
    pub format: NcFormat,
    /// Header length in bytes.
    pub header_size: u64,
    /// One entry per variable, in declaration order.
    pub vars: Vec<NcVarLayout>,
    /// Total file length with zero records.
    pub file_size: u64,
}

fn pad4(n: u64) -> u64 {
    (n + 3) & !3
}

fn too_large(what: impl Into<String>, value: u64, limit: u64) -> ExodusError {
    ExodusError::TooLarge { what: what.into(), value, limit }
}

impl<'a> NcFile<'a> {
    /// An empty file of the given format.
    pub fn new(format: NcFormat) -> Self {
        Self { format, dims: Vec::new(), gattrs: Vec::new(), vars: Vec::new() }
    }

    /// The format.
    pub fn format(&self) -> NcFormat {
        self.format
    }

    /// Declare a dimension; `None` is the unlimited record dimension (at most one, checked by
    /// [`NcFile::layout`]; a length of `Some(0)` is rejected there too because the header encodes
    /// the record dimension as length 0).
    pub fn dim(&mut self, name: &str, len: Option<u64>) -> DimId {
        self.dims.push(NcDim { name: name.to_string(), len });
        DimId(self.dims.len() - 1)
    }

    /// Set a global attribute.
    pub fn global_attr(&mut self, name: &str, value: NcValue) {
        self.gattrs.push((name.to_string(), value));
    }

    /// Declare a variable over `dims` (slowest varying first).
    pub fn var(&mut self, name: &str, ty: NcType, dims: &[DimId]) -> VarId {
        self.vars.push(NcVar { name: name.to_string(), ty, dims: dims.to_vec(), attrs: Vec::new(), data: None });
        VarId(self.vars.len() - 1)
    }

    /// Set an attribute of a variable. An id from another file is ignored.
    pub fn var_attr(&mut self, v: VarId, name: &str, value: NcValue) {
        if let Some(var) = self.vars.get_mut(v.0) {
            var.attrs.push((name.to_string(), value));
        }
    }

    /// Set (or replace) the data of a variable. An id from another file is ignored.
    pub fn set_data(&mut self, v: VarId, data: NcData<'a>) {
        if let Some(var) = self.vars.get_mut(v.0) {
            var.data = Some(data);
        }
    }

    /// Declared dimensions.
    pub fn dims(&self) -> &[NcDim] {
        &self.dims
    }

    /// Declared variables.
    pub fn vars(&self) -> &[NcVar<'a>] {
        &self.vars
    }

    /// Global attributes.
    pub fn global_attrs(&self) -> &[(String, NcValue)] {
        &self.gattrs
    }

    /// Dimension by name.
    pub fn dim_by_name(&self, name: &str) -> Option<&NcDim> {
        self.dims.iter().find(|d| d.name == name)
    }

    /// Variable by name.
    pub fn var_by_name(&self, name: &str) -> Option<&NcVar<'a>> {
        self.vars.iter().find(|v| v.name == name)
    }

    /// Global attribute by name.
    pub fn global_attr_by_name(&self, name: &str) -> Option<&NcValue> {
        self.gattrs.iter().find(|(n, _)| n == name).map(|(_, v)| v)
    }

    /// Bytes of a variable (calls a lazy provider).
    pub fn var_bytes(&self, name: &str) -> Option<Cow<'_, [u8]>> {
        self.var_by_name(name)?.data.as_ref().map(NcData::bytes)
    }

    /// Length of a dimension by name (the unlimited dimension reports 0).
    pub fn dim_len(&self, name: &str) -> Option<u64> {
        self.dim_by_name(name).map(|d| d.len.unwrap_or(0))
    }

    fn validate(&self) -> Result<()> {
        let mut seen = std::collections::BTreeSet::new();
        let mut unlimited = 0;
        for d in &self.dims {
            if d.name.is_empty() || !seen.insert(d.name.as_str()) {
                return Err(ExodusError::Malformed(format!("dimension name '{}' is empty or repeated", d.name)));
            }
            match d.len {
                None => unlimited += 1,
                Some(0) => {
                    return Err(ExodusError::Malformed(format!(
                        "dimension '{}' has length 0 (the header encodes the record dimension that way)",
                        d.name
                    )));
                }
                Some(n) if self.format != NcFormat::Data64 && n > u64::from(u32::MAX) => {
                    return Err(too_large(format!("dimension '{}'", d.name), n, u64::from(u32::MAX)));
                }
                Some(_) => {}
            }
        }
        if unlimited > 1 {
            return Err(ExodusError::Malformed("more than one unlimited dimension".into()));
        }
        let mut vseen = std::collections::BTreeSet::new();
        for v in &self.vars {
            if v.name.is_empty() || !vseen.insert(v.name.as_str()) {
                return Err(ExodusError::Malformed(format!("variable name '{}' is empty or repeated", v.name)));
            }
            if v.ty == NcType::Int64 && self.format != NcFormat::Data64 {
                return Err(ExodusError::Unsupported(format!("variable '{}': NC_INT64 needs Data64", v.name)));
            }
            for (i, d) in v.dims.iter().enumerate() {
                let dim = self
                    .dims
                    .get(d.0)
                    .ok_or_else(|| ExodusError::Malformed(format!("variable '{}' uses an unknown dimension", v.name)))?;
                if dim.len.is_none() && i != 0 {
                    return Err(ExodusError::Malformed(format!(
                        "variable '{}': the unlimited dimension must come first",
                        v.name
                    )));
                }
            }
        }
        let attr_ok = |owner: &str, attrs: &[(String, NcValue)]| -> Result<()> {
            let mut s = std::collections::BTreeSet::new();
            for (n, v) in attrs {
                if n.is_empty() || !s.insert(n.as_str()) {
                    return Err(ExodusError::Malformed(format!("{owner}: attribute '{n}' is empty or repeated")));
                }
                if v.ty() == NcType::Int64 && self.format != NcFormat::Data64 {
                    return Err(ExodusError::Unsupported(format!("{owner}: NC_INT64 attribute needs Data64")));
                }
            }
            Ok(())
        };
        attr_ok("global", &self.gattrs)?;
        for v in &self.vars {
            attr_ok(&v.name, &v.attrs)?;
        }
        Ok(())
    }

    /// Elements per record (or in total) of a variable, with overflow checks.
    fn elements(&self, v: &NcVar<'_>) -> Result<(u64, bool)> {
        let mut n: u64 = 1;
        let mut record = false;
        for d in &v.dims {
            match self.dims[d.0].len {
                None => record = true,
                Some(l) => {
                    n = n.checked_mul(l).ok_or_else(|| too_large(format!("variable '{}'", v.name), u64::MAX, u64::MAX))?;
                }
            }
        }
        Ok((n, record))
    }

    fn header(&self, begins: &[u64], vsizes: &[u64]) -> Vec<u8> {
        let mut h = Header { out: Vec::new(), cw: self.format.count_width(), ow: self.format.offset_width() };
        h.out.extend_from_slice(&self.format.magic());
        h.count(0); // numrecs
        if self.dims.is_empty() {
            h.absent();
        } else {
            h.tag(TAG_DIMENSION);
            h.count(self.dims.len() as u64);
            for d in &self.dims {
                h.name(&d.name);
                h.count(d.len.unwrap_or(0));
            }
        }
        h.attrs(&self.gattrs);
        if self.vars.is_empty() {
            h.absent();
        } else {
            h.tag(TAG_VARIABLE);
            h.count(self.vars.len() as u64);
            for (i, v) in self.vars.iter().enumerate() {
                h.name(&v.name);
                h.count(v.dims.len() as u64);
                for d in &v.dims {
                    h.count(d.0 as u64);
                }
                h.attrs(&v.attrs);
                h.out.extend_from_slice(&v.ty.code().to_be_bytes());
                h.count(vsizes[i]);
                h.offset(begins[i]);
            }
        }
        h.out
    }

    /// Header size, and `vsize` / `begin` of every variable, without touching any data (a
    /// `2^33`-element dimension is fine). Non-record variables follow the header in declaration
    /// order, record variables (zero records) after them.
    ///
    /// Errors: [`ExodusError::TooLarge`] when CDF-1 needs an offset or size above `u32::MAX`, or
    /// when a CDF-2 variable above 4 GiB is not the last non-record variable;
    /// [`ExodusError::Malformed`] for inconsistent models (length-0 dimension, two unlimited
    /// dimensions, repeated names).
    pub fn layout(&self) -> Result<NcLayout> {
        self.validate()?;
        let n = self.vars.len();
        let header_size = self.header(&vec![0; n], &vec![0; n]).len() as u64;
        let mut sizes = Vec::with_capacity(n);
        let mut rec = Vec::with_capacity(n);
        for v in &self.vars {
            let (elems, record) = self.elements(v)?;
            let bytes = elems.checked_mul(v.ty.size()).ok_or_else(|| too_large(format!("variable '{}'", v.name), u64::MAX, u64::MAX))?;
            sizes.push(pad4(bytes));
            rec.push(record);
        }
        let last_fixed = (0..n).rev().find(|&i| !rec[i]);
        let mut begins = vec![0u64; n];
        let mut vsizes = vec![0u64; n];
        let mut pos = header_size;
        let classic = self.format == NcFormat::Classic;
        let max32 = u64::from(u32::MAX);
        for pass_record in [false, true] {
            for i in (0..n).filter(|&i| rec[i] == pass_record) {
                begins[i] = pos;
                vsizes[i] = sizes[i];
                if classic && (pos > max32 || sizes[i] > max32) {
                    return Err(too_large(format!("variable '{}' (CDF-1 offset/size)", self.vars[i].name), pos.max(sizes[i]), max32));
                }
                if self.format == NcFormat::Offset64 && sizes[i] > max32 {
                    if Some(i) != last_fixed {
                        return Err(too_large(format!("variable '{}' (CDF-2 allows one >4 GiB variable, as the last)", self.vars[i].name), sizes[i], max32));
                    }
                    vsizes[i] = max32;
                }
                if !pass_record {
                    pos = pos.checked_add(sizes[i]).ok_or_else(|| too_large("file", u64::MAX, u64::MAX))?;
                }
            }
        }
        if classic && pos > max32 {
            return Err(too_large("file (CDF-1)", pos, max32));
        }
        let vars = (0..n)
            .map(|i| NcVarLayout {
                name: self.vars[i].name.clone(),
                is_record: rec[i],
                vsize: vsizes[i],
                size: sizes[i],
                begin: begins[i],
            })
            .collect();
        Ok(NcLayout { format: self.format, header_size, vars, file_size: pos })
    }

    /// Write the file; returns the number of bytes written. Lazy providers are called one at a
    /// time. Every non-record variable needs data of exactly `elements * size` bytes.
    pub fn write(&self, w: &mut dyn Write) -> Result<u64> {
        let lay = self.layout()?;
        let begins: Vec<u64> = lay.vars.iter().map(|v| v.begin).collect();
        let vsizes: Vec<u64> = lay.vars.iter().map(|v| v.vsize).collect();
        let header = self.header(&begins, &vsizes);
        w.write_all(&header)?;
        let mut written = header.len() as u64;
        let zeros = [0u8; 4];
        for (v, l) in self.vars.iter().zip(&lay.vars) {
            if l.is_record {
                if v.data.as_ref().is_some_and(|d| !d.bytes().is_empty()) {
                    return Err(ExodusError::Unsupported(format!("variable '{}': record data", v.name)));
                }
                continue;
            }
            let (elems, _) = self.elements(v)?;
            let want = elems * v.ty.size();
            let data = v
                .data
                .as_ref()
                .ok_or_else(|| ExodusError::Malformed(format!("variable '{}' has no data", v.name)))?
                .bytes();
            if data.len() as u64 != want {
                return Err(ExodusError::Malformed(format!("variable '{}': data is {} bytes, expected {want}", v.name, data.len())));
            }
            w.write_all(&data)?;
            let pad = (l.size - want) as usize;
            w.write_all(&zeros[..pad])?;
            written += l.size;
        }
        Ok(written)
    }

    /// Render `ncdump`-style CDL: header plus a `data:` section (doubles with `%.17g`, integers
    /// plain, floats shortest round-trip with an `f` suffix, char variables as quoted rows with
    /// trailing NULs trimmed). Variables without data, and the record variables, are skipped in
    /// `data:`. Calls lazy providers.
    pub fn to_cdl(&self) -> String {
        let mut s = String::from("netcdf file {\ndimensions:\n");
        for d in &self.dims {
            match d.len {
                Some(n) => s.push_str(&format!("\t{} = {} ;\n", d.name, n)),
                None => s.push_str(&format!("\t{} = UNLIMITED ; // (0 currently)\n", d.name)),
            }
        }
        s.push_str("variables:\n");
        for v in &self.vars {
            let dims: Vec<&str> = v.dims.iter().map(|d| self.dims[d.0].name.as_str()).collect();
            if dims.is_empty() {
                s.push_str(&format!("\t{} {} ;\n", v.ty.cdl_name(), v.name));
            } else {
                s.push_str(&format!("\t{} {}({}) ;\n", v.ty.cdl_name(), v.name, dims.join(", ")));
            }
            for (n, a) in &v.attrs {
                s.push_str(&format!("\t\t{}:{} = {} ;\n", v.name, n, cdl_value(a)));
            }
        }
        if !self.gattrs.is_empty() {
            s.push_str("\n// global attributes:\n");
            for (n, a) in &self.gattrs {
                s.push_str(&format!("\t\t:{} = {} ;\n", n, cdl_value(a)));
            }
        }
        s.push_str("data:\n");
        for v in &self.vars {
            let Some(d) = &v.data else { continue };
            let is_record = v.dims.first().is_some_and(|d| self.dims[d.0].len.is_none());
            let bytes = d.bytes();
            if is_record || bytes.is_empty() {
                continue;
            }
            s.push_str(&format!("\n {} =", v.name));
            if v.ty == NcType::Char {
                let row = v.dims.last().and_then(|d| self.dims[d.0].len).unwrap_or(bytes.len() as u64).max(1) as usize;
                let rows: Vec<String> = bytes.chunks(row).map(cdl_string_bytes).collect();
                if rows.len() == 1 {
                    s.push_str(&format!(" {} ;\n", rows[0]));
                } else {
                    s.push_str(&format!("\n  {} ;\n", rows.join(",\n  ")));
                }
            } else {
                let vals = cdl_numbers(v.ty, &bytes);
                s.push_str(&format!(" {} ;\n", vals.join(", ")));
            }
        }
        s.push_str("}\n");
        s
    }
}

impl NcFile<'static> {
    /// Parse a CDF-1/2/5 file. Rejects NetCDF-4/HDF5 (`\x89HDF`) with `Unsupported`, anything
    /// else invalid (truncation, bad tags, offsets past the end) with `Malformed`; never panics.
    /// Files with record data (`numrecs > 0`) are `Unsupported`. Data are copied.
    pub fn read(bytes: &[u8]) -> Result<NcFile<'static>> {
        if bytes.starts_with(b"\x89HDF") {
            return Err(ExodusError::Unsupported("netCDF-4/HDF5 input".into()));
        }
        let mut c = Cursor { b: bytes, pos: 0, cw: 4, ow: 4 };
        let magic = c.take(4)?;
        let format = match magic {
            [b'C', b'D', b'F', 1] => NcFormat::Classic,
            [b'C', b'D', b'F', 2] => NcFormat::Offset64,
            [b'C', b'D', b'F', 5] => NcFormat::Data64,
            _ => return Err(ExodusError::Malformed("bad magic".into())),
        };
        c.cw = format.count_width();
        c.ow = format.offset_width();
        let numrecs = c.count()?;
        let mut f = NcFile::new(format);
        let mut record_dim_seen = false;
        // dimensions
        let n = c.list(TAG_DIMENSION)?;
        for _ in 0..n {
            let name = c.name()?;
            let len = c.count()?;
            if len == 0 {
                if record_dim_seen {
                    return Err(ExodusError::Malformed("two unlimited dimensions".into()));
                }
                record_dim_seen = true;
                f.dims.push(NcDim { name, len: None });
            } else {
                f.dims.push(NcDim { name, len: Some(len) });
            }
        }
        f.gattrs = c.attrs()?;
        let nv = c.list(TAG_VARIABLE)?;
        let mut placement = Vec::new();
        for _ in 0..nv {
            let name = c.name()?;
            let nd = c.count()?;
            let mut dims = Vec::new();
            for _ in 0..nd {
                let d = c.count()?;
                if d >= f.dims.len() as u64 {
                    return Err(ExodusError::Malformed(format!("variable '{name}': bad dimension id {d}")));
                }
                dims.push(DimId(d as usize));
            }
            let attrs = c.attrs()?;
            let ty = NcType::from_code(c.u32()?).ok_or_else(|| ExodusError::Malformed(format!("variable '{name}': bad nc_type")))?;
            if ty == NcType::Int64 && format != NcFormat::Data64 {
                return Err(ExodusError::Malformed(format!("variable '{name}': NC_INT64 outside CDF-5")));
            }
            let _vsize = c.count()?;
            let begin = c.offset()?;
            f.vars.push(NcVar { name, ty, dims, attrs, data: None });
            placement.push(begin);
        }
        for (i, begin) in placement.into_iter().enumerate() {
            let v = &f.vars[i];
            let is_record = v.dims.first().is_some_and(|d| f.dims[d.0].len.is_none());
            if is_record {
                if numrecs != 0 {
                    return Err(ExodusError::Unsupported("record data".into()));
                }
                continue;
            }
            let mut elems: u64 = 1;
            for d in &v.dims {
                let l = f.dims[d.0].len.unwrap_or(1);
                elems = elems.checked_mul(l).ok_or_else(|| ExodusError::Malformed("variable size overflow".into()))?;
            }
            let size = elems.checked_mul(v.ty.size()).ok_or_else(|| ExodusError::Malformed("variable size overflow".into()))?;
            let end = begin.checked_add(size).ok_or_else(|| ExodusError::Malformed("variable range overflow".into()))?;
            if end > bytes.len() as u64 {
                return Err(ExodusError::Malformed(format!("variable '{}' extends past the end of the file", v.name)));
            }
            let data = bytes[begin as usize..end as usize].to_vec();
            f.vars[i].data = Some(NcData::Bytes(data));
        }
        Ok(f)
    }
}

struct Header {
    out: Vec<u8>,
    cw: usize,
    ow: usize,
}

impl Header {
    fn tag(&mut self, t: u32) {
        self.out.extend_from_slice(&t.to_be_bytes());
    }
    fn count(&mut self, v: u64) {
        if self.cw == 8 {
            self.out.extend_from_slice(&v.to_be_bytes());
        } else {
            self.out.extend_from_slice(&(v as u32).to_be_bytes());
        }
    }
    fn offset(&mut self, v: u64) {
        if self.ow == 8 {
            self.out.extend_from_slice(&v.to_be_bytes());
        } else {
            self.out.extend_from_slice(&(v as u32).to_be_bytes());
        }
    }
    fn absent(&mut self) {
        self.tag(0);
        self.count(0);
    }
    fn pad(&mut self) {
        while !self.out.len().is_multiple_of(4) {
            self.out.push(0);
        }
    }
    fn name(&mut self, s: &str) {
        self.count(s.len() as u64);
        self.out.extend_from_slice(s.as_bytes());
        self.pad();
    }
    fn attrs(&mut self, attrs: &[(String, NcValue)]) {
        if attrs.is_empty() {
            self.absent();
            return;
        }
        self.tag(TAG_ATTRIBUTE);
        self.count(attrs.len() as u64);
        for (n, v) in attrs {
            self.name(n);
            self.out.extend_from_slice(&v.ty().code().to_be_bytes());
            self.count(v.len() as u64);
            self.out.extend_from_slice(&v.encode());
            self.pad();
        }
    }
}

struct Cursor<'b> {
    b: &'b [u8],
    pos: usize,
    cw: usize,
    ow: usize,
}

impl<'b> Cursor<'b> {
    fn take(&mut self, n: usize) -> Result<&'b [u8]> {
        let end = self.pos.checked_add(n).filter(|e| *e <= self.b.len()).ok_or_else(|| ExodusError::Malformed("truncated header".into()))?;
        let s = &self.b[self.pos..end];
        self.pos = end;
        Ok(s)
    }
    fn u32(&mut self) -> Result<u32> {
        let s = self.take(4)?;
        Ok(u32::from_be_bytes([s[0], s[1], s[2], s[3]]))
    }
    fn wide(&mut self, w: usize) -> Result<u64> {
        if w == 8 {
            let s = self.take(8)?;
            Ok(u64::from_be_bytes([s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7]]))
        } else {
            Ok(u64::from(self.u32()?))
        }
    }
    fn count(&mut self) -> Result<u64> {
        self.wide(self.cw)
    }
    fn offset(&mut self) -> Result<u64> {
        self.wide(self.ow)
    }
    fn skip_pad(&mut self, n: usize) -> Result<()> {
        self.take((4 - n % 4) % 4).map(|_| ())
    }
    fn name(&mut self) -> Result<String> {
        let n = self.count()?;
        let n = usize::try_from(n).map_err(|_| ExodusError::Malformed("name length".into()))?;
        let s = String::from_utf8_lossy(self.take(n)?).into_owned();
        self.skip_pad(n)?;
        Ok(s)
    }
    /// Read a list header; returns the element count (0 for ABSENT).
    fn list(&mut self, want: u32) -> Result<u64> {
        let tag = self.u32()?;
        let n = self.count()?;
        match (tag, n) {
            (0, 0) => Ok(0),
            (t, n) if t == want => {
                if n > self.b.len() as u64 {
                    return Err(ExodusError::Malformed("list longer than the file".into()));
                }
                Ok(n)
            }
            _ => Err(ExodusError::Malformed(format!("bad list tag {tag:#x}"))),
        }
    }
    fn attrs(&mut self) -> Result<Vec<(String, NcValue)>> {
        let n = self.list(TAG_ATTRIBUTE)?;
        let mut out = Vec::new();
        for _ in 0..n {
            let name = self.name()?;
            let ty = NcType::from_code(self.u32()?).ok_or_else(|| ExodusError::Malformed(format!("attribute '{name}': bad nc_type")))?;
            let cnt = self.count()?;
            let bytes = cnt.checked_mul(ty.size()).and_then(|b| usize::try_from(b).ok()).ok_or_else(|| ExodusError::Malformed("attribute size".into()))?;
            let raw = self.take(bytes)?;
            self.skip_pad(bytes)?;
            out.push((name, NcValue::decode(ty, raw)));
        }
        Ok(out)
    }
}

// ---- CDL helpers ---------------------------------------------------------------------------

/// C `%.{prec}g` for finite `v`; `NaN` / `Infinity` spelled as `ncdump` does.
pub(crate) fn fmt_g(v: f64, prec: usize) -> String {
    if v.is_nan() {
        return "NaN".into();
    }
    if v.is_infinite() {
        return if v > 0.0 { "Infinity".into() } else { "-Infinity".into() };
    }
    if v == 0.0 {
        return if v.is_sign_negative() { "-0".into() } else { "0".into() };
    }
    let p = prec.max(1);
    let e = format!("{:.*e}", p - 1, v);
    let (mant, exp) = e.split_once('e').unwrap_or((&e, "0"));
    let exp: i32 = exp.parse().unwrap_or(0);
    let trim = |s: &str| -> String {
        if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.').to_string() } else { s.to_string() }
    };
    if exp < -4 || exp >= p as i32 {
        let sign = if exp < 0 { '-' } else { '+' };
        format!("{}e{}{:02}", trim(mant), sign, exp.abs())
    } else {
        trim(&format!("{:.*}", (p as i32 - 1 - exp) as usize, v))
    }
}

fn cdl_string_bytes(b: &[u8]) -> String {
    let end = b.iter().rposition(|c| *c != 0).map_or(0, |p| p + 1);
    let mut s = String::from("\"");
    for &c in &b[..end] {
        match c {
            b'"' => s.push_str("\\\""),
            b'\\' => s.push_str("\\\\"),
            b'\n' => s.push_str("\\n"),
            b'\t' => s.push_str("\\t"),
            0x20..=0x7e => s.push(c as char),
            0x80..=0xff => s.push(c as char),
            _ => s.push_str(&format!("\\{c:03o}")),
        }
    }
    s.push('"');
    s
}

fn cdl_numbers(ty: NcType, b: &[u8]) -> Vec<String> {
    match NcValue::decode(ty, b) {
        NcValue::Text(_) => Vec::new(),
        NcValue::Byte(v) => v.iter().map(|x| format!("{x}b")).collect(),
        NcValue::Short(v) => v.iter().map(|x| format!("{x}s")).collect(),
        NcValue::Int(v) => v.iter().map(|x| x.to_string()).collect(),
        NcValue::Float(v) => v.iter().map(|x| format!("{x}f")).collect(),
        NcValue::Double(v) => v.iter().map(|x| fmt_g(*x, 17)).collect(),
        NcValue::Int64(v) => v.iter().map(|x| x.to_string()).collect(),
    }
}

fn cdl_value(v: &NcValue) -> String {
    match v {
        NcValue::Text(s) => cdl_string_bytes(s.as_bytes()),
        other => cdl_numbers(other.ty(), &other.encode()).join(", "),
    }
}

#[cfg(test)]
mod tests {
    use super::fmt_g;

    #[test]
    fn g_format_matches_c() {
        assert_eq!(fmt_g(0.0, 17), "0");
        assert_eq!(fmt_g(1.0, 17), "1");
        assert_eq!(fmt_g(0.5, 17), "0.5");
        assert_eq!(fmt_g(0.1, 17), "0.10000000000000001");
        assert_eq!(fmt_g(-2.5, 17), "-2.5");
        assert_eq!(fmt_g(1e20, 17), "1e+20");
        assert_eq!(fmt_g(1.5e-7, 17), "1.4999999999999999e-07");
        assert_eq!(fmt_g(123456789.0, 17), "123456789");
        assert_eq!(fmt_g(0.0001, 6), "0.0001");
        assert_eq!(fmt_g(0.00001, 6), "1e-05");
    }
}
