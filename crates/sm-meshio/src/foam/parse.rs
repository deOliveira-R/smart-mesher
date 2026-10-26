//! ASCII polyMesh reader. Iterative (no recursion), allocation proportional to the tokens read
//! (declared counts are only compared, never pre-allocated), `Err` and never a panic on any input.

use super::{FoamError, FoamFile, FoamPatchType, PatchOrigin, PolyFaceZone, PolyMesh, PolyPatch, PolyZone};
use crate::error::{MeshioError, Result};
use std::path::Path;

fn fe(e: FoamError) -> MeshioError {
    MeshioError::Foam(e)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Tok<'a> {
    Word(&'a str),
    Str(&'a str),
    LParen,
    RParen,
    LBrace,
    RBrace,
    Semi,
}

struct Lexer<'a> {
    file: &'a str,
    src: &'a [u8],
    pos: usize,
    line: usize,
    peeked: Option<Tok<'a>>,
}

impl<'a> Lexer<'a> {
    fn new(file: &'a str, src: &'a [u8]) -> Self {
        Self { file, src, pos: 0, line: 1, peeked: None }
    }

    fn err(&self, detail: impl Into<String>) -> MeshioError {
        fe(FoamError::Parse { file: self.file.to_string(), line: self.line, detail: detail.into() })
    }

    fn malformed<T>(&self, detail: impl Into<String>) -> Result<T> {
        Err(fe(FoamError::Malformed(format!("{}: line {}: {}", self.file, self.line, detail.into()))))
    }

    fn skip_space(&mut self) -> Result<()> {
        loop {
            match self.src.get(self.pos) {
                Some(b'\n') => {
                    self.line += 1;
                    self.pos += 1;
                }
                Some(b' ' | b'\t' | b'\r' | 0x0b | 0x0c) => self.pos += 1,
                Some(b'/') if self.src.get(self.pos + 1) == Some(&b'/') => {
                    while self.src.get(self.pos).is_some_and(|c| *c != b'\n') {
                        self.pos += 1;
                    }
                }
                Some(b'/') if self.src.get(self.pos + 1) == Some(&b'*') => {
                    let start_line = self.line;
                    self.pos += 2;
                    loop {
                        match self.src.get(self.pos) {
                            None => {
                                self.line = start_line;
                                return Err(self.err("unterminated /* comment"));
                            }
                            Some(b'*') if self.src.get(self.pos + 1) == Some(&b'/') => {
                                self.pos += 2;
                                break;
                            }
                            Some(b'\n') => {
                                self.line += 1;
                                self.pos += 1;
                            }
                            Some(_) => self.pos += 1,
                        }
                    }
                }
                _ => return Ok(()),
            }
        }
    }

    /// Next token, `None` at end of input.
    fn next(&mut self) -> Result<Option<Tok<'a>>> {
        if let Some(t) = self.peeked.take() {
            return Ok(Some(t));
        }
        self.lex()
    }

    fn lex(&mut self) -> Result<Option<Tok<'a>>> {
        self.skip_space()?;
        let Some(&c) = self.src.get(self.pos) else { return Ok(None) };
        let tok = match c {
            b'(' => Some(Tok::LParen),
            b')' => Some(Tok::RParen),
            b'{' => Some(Tok::LBrace),
            b'}' => Some(Tok::RBrace),
            b';' => Some(Tok::Semi),
            _ => None,
        };
        if let Some(t) = tok {
            self.pos += 1;
            return Ok(Some(t));
        }
        if c == b'"' {
            let start = self.pos + 1;
            let mut i = start;
            loop {
                match self.src.get(i) {
                    None => return Err(self.err("unterminated string")),
                    Some(b'\\') => i += 2,
                    Some(b'"') => break,
                    Some(b'\n') => {
                        self.line += 1;
                        i += 1;
                    }
                    Some(_) => i += 1,
                }
            }
            let s = std::str::from_utf8(&self.src[start..i]).map_err(|_| self.err("string is not UTF-8"))?;
            self.pos = i + 1;
            return Ok(Some(Tok::Str(s)));
        }
        let start = self.pos;
        while self.src.get(self.pos).is_some_and(|c| !matches!(c, b' ' | b'\t' | b'\r' | b'\n' | 0x0b | 0x0c | b'(' | b')' | b'{' | b'}' | b';' | b'"')) {
            self.pos += 1;
        }
        let w = std::str::from_utf8(&self.src[start..self.pos]).map_err(|_| self.err("token is not UTF-8"))?;
        Ok(Some(Tok::Word(w)))
    }

    fn peek(&mut self) -> Result<Option<Tok<'a>>> {
        if self.peeked.is_none() {
            self.peeked = self.lex()?;
        }
        Ok(self.peeked)
    }

    fn need(&mut self, what: &str) -> Result<Tok<'a>> {
        self.next()?.ok_or_else(|| self.err(format!("unexpected end of file, expected {what}")))
    }

    fn expect(&mut self, t: Tok<'a>, what: &str) -> Result<()> {
        let got = self.need(what)?;
        if got == t { Ok(()) } else { Err(self.err(format!("expected {what}, found {got:?}"))) }
    }

    fn word(&mut self, what: &str) -> Result<&'a str> {
        match self.need(what)? {
            Tok::Word(w) | Tok::Str(w) => Ok(w),
            t => Err(self.err(format!("expected {what}, found {t:?}"))),
        }
    }

    /// Skip the value of an entry up to its `;` (parentheses and braces nest).
    fn skip_to_semi(&mut self) -> Result<()> {
        let mut depth = 0usize;
        loop {
            match self.need("';'")? {
                Tok::LParen | Tok::LBrace => depth += 1,
                Tok::RParen | Tok::RBrace => {
                    depth = depth.checked_sub(1).ok_or_else(|| self.err("unbalanced bracket"))?;
                }
                Tok::Semi if depth == 0 => return Ok(()),
                _ => {}
            }
        }
    }

    fn count(&mut self) -> Result<usize> {
        let w = self.word("a count")?;
        if w.starts_with('-') {
            return self.malformed(format!("negative count {w}"));
        }
        w.parse::<usize>().map_err(|_| self.err(format!("'{w}' is not a count")))
    }

    fn label(&mut self) -> Result<u32> {
        let w = self.word("a label")?;
        if w.starts_with('-') {
            return self.malformed(format!("negative label {w}"));
        }
        w.parse::<u32>().or_else(|_| self.malformed(format!("'{w}' is not a 32-bit label")))
    }

    fn scalar(&mut self) -> Result<f64> {
        let w = self.word("a number")?;
        match w.parse::<f64>() {
            Ok(v) if v.is_finite() => Ok(v),
            Ok(_) => self.malformed(format!("non-finite coordinate '{w}'")),
            Err(_) => Err(self.err(format!("'{w}' is not a number"))),
        }
    }

    fn boolean(&mut self) -> Result<bool> {
        let w = self.word("a bool")?;
        match w {
            "1" | "true" | "on" | "yes" | "y" | "t" => Ok(true),
            "0" | "false" | "off" | "no" | "n" | "f" | "none" => Ok(false),
            _ => Err(self.err(format!("'{w}' is not a bool"))),
        }
    }

    /// `[List<T>] N ( elem ... )` or the uniform form `N { elem }`. The count is compared with
    /// the elements actually read; it is never used to size an allocation.
    fn list<T: Clone>(&mut self, mut elem: impl FnMut(&mut Self) -> Result<T>) -> Result<Vec<T>> {
        if let Some(Tok::Word(w)) = self.peek()?
            && w.starts_with("List<")
        {
            self.next()?;
        }
        let n = self.count()?;
        let mut out = Vec::new();
        match self.need("'(' or '{'")? {
            Tok::LParen => {
                loop {
                    if self.peek()? == Some(Tok::RParen) {
                        self.next()?;
                        break;
                    }
                    out.push(elem(self)?);
                }
                if out.len() != n {
                    return self.malformed(format!("list declares {n} entries but holds {}", out.len()));
                }
            }
            Tok::LBrace => {
                if n > self.src.len() {
                    return self.malformed(format!("uniform list of {n} entries is larger than the input"));
                }
                let start = self.pos;
                let v = elem(self)?;
                self.expect(Tok::RBrace, "'}'")?;
                // n copies of an element that took `size` bytes to write must fit in the input,
                // so the expansion stays proportional to it whatever the element's own size.
                let size = (self.pos - start).max(1);
                if n.saturating_mul(size) > self.src.len() {
                    return self.malformed(format!("uniform list of {n} entries of {size} bytes is larger than the input"));
                }
                out = vec![v; n];
            }
            t => return Err(self.err(format!("expected '(' or '{{' after the count, found {t:?}"))),
        }
        Ok(out)
    }
}

/// `FoamFile { ... }` header; returns after the closing brace.
fn header(lx: &mut Lexer) -> Result<()> {
    let w = lx.word("'FoamFile'")?;
    if w != "FoamFile" {
        return Err(lx.err(format!("expected the FoamFile header, found '{w}'")));
    }
    lx.expect(Tok::LBrace, "'{'")?;
    loop {
        match lx.need("a header entry or '}'")? {
            Tok::RBrace => return Ok(()),
            Tok::Word(key) => {
                let value = match lx.need("a value")? {
                    Tok::Word(v) | Tok::Str(v) => v,
                    t => return Err(lx.err(format!("expected a value for '{key}', found {t:?}"))),
                };
                lx.skip_to_semi()?;
                match key {
                    "format" if value != "ascii" => {
                        return Err(fe(FoamError::Unsupported(format!("{}: format '{value}' (only ascii is read)", lx.file))));
                    }
                    "class" if value == "faceCompactList" => {
                        return Err(fe(FoamError::Unsupported(format!("{}: class faceCompactList", lx.file))));
                    }
                    _ => {}
                }
            }
            t => return Err(lx.err(format!("unexpected {t:?} in the FoamFile header"))),
        }
    }
}

fn find<'a>(files: &'a [FoamFile], name: &str) -> Result<Option<&'a FoamFile>> {
    let mut hits = files.iter().filter(|f| f.name == name);
    let first = hits.next();
    if hits.next().is_some() {
        return Err(fe(FoamError::Malformed(format!("file '{name}' given twice"))));
    }
    Ok(first)
}

fn require<'a>(files: &'a [FoamFile], name: &str) -> Result<&'a FoamFile> {
    find(files, name)?.ok_or_else(|| fe(FoamError::Malformed(format!("missing file '{name}'"))))
}

fn only_end(lx: &mut Lexer) -> Result<()> {
    match lx.next()? {
        None => Ok(()),
        Some(t) => Err(lx.err(format!("unexpected {t:?} after the list"))),
    }
}

fn label_file(f: &FoamFile) -> Result<Vec<u32>> {
    let mut lx = Lexer::new(&f.name, &f.bytes);
    header(&mut lx)?;
    let v = lx.list(|l| l.label())?;
    only_end(&mut lx)?;
    Ok(v)
}

/// One boundary or zone dictionary list: `N ( name { entries } ... )`. `entry` gets the key and
/// the lexer positioned after it and must consume the value including its `;`.
fn dict_list<T>(
    lx: &mut Lexer,
    mut start: impl FnMut(&str) -> T,
    mut entry: impl FnMut(&mut T, &str, &mut Lexer) -> Result<()>,
) -> Result<Vec<T>> {
    let n = lx.count()?;
    lx.expect(Tok::LParen, "'('")?;
    let mut out = Vec::new();
    loop {
        match lx.need("a name or ')'")? {
            Tok::RParen => break,
            Tok::Word(name) | Tok::Str(name) => {
                lx.expect(Tok::LBrace, "'{'")?;
                let mut item = start(name);
                loop {
                    match lx.need("an entry or '}'")? {
                        Tok::RBrace => break,
                        Tok::Word(key) => entry(&mut item, key, lx)?,
                        t => return Err(lx.err(format!("unexpected {t:?} in a dictionary"))),
                    }
                }
                out.push(item);
            }
            t => return Err(lx.err(format!("unexpected {t:?}, expected a name"))),
        }
    }
    if out.len() != n {
        return lx.malformed(format!("list declares {n} entries but holds {}", out.len()));
    }
    Ok(out)
}

/// The `faces` list straight into CSR (offsets, concatenated labels), no per-face allocation.
/// The uniform `N{face}` spelling is rejected: OpenFOAM never writes it for faces, and expanding
/// it would multiply the input.
fn faces_csr(lx: &mut Lexer) -> Result<(Vec<u64>, Vec<u32>)> {
    if let Some(Tok::Word(w)) = lx.peek()?
        && w.starts_with("List<")
    {
        lx.next()?;
    }
    let n = lx.count()?;
    match lx.need("'('")? {
        Tok::LParen => {}
        Tok::LBrace => return Err(fe(FoamError::Unsupported(format!("{}: uniform N{{face}} list for faces", lx.file)))),
        t => return Err(lx.err(format!("expected '(' after the count, found {t:?}"))),
    }
    let mut offsets = vec![0u64];
    let mut pts = Vec::new();
    while lx.peek()? != Some(Tok::RParen) {
        let k = lx.count()?;
        lx.expect(Tok::LParen, "'('")?;
        let before = pts.len();
        while lx.peek()? != Some(Tok::RParen) {
            pts.push(lx.label()?);
        }
        lx.next()?;
        let held = pts.len() - before;
        if held != k {
            return lx.malformed(format!("face declares {k} points but holds {held}"));
        }
        if k < 3 {
            return lx.malformed(format!("a face with {k} points"));
        }
        offsets.push(pts.len() as u64);
    }
    lx.next()?;
    if offsets.len() - 1 != n {
        return lx.malformed(format!("list declares {n} entries but holds {}", offsets.len() - 1));
    }
    Ok((offsets, pts))
}

/// Parse polyMesh files (`points faces owner neighbour boundary`, optional `cellZones`,
/// `faceZones`) into a [`PolyMesh`]. ASCII only: `format binary` is [`FoamError::Unsupported`].
/// Handles `//` and `/* */` comments, the `FoamFile` header and the list spellings `N(...)`,
/// `N\n(\n...\n)`, `List<label> N(...)` and `N{value}`. Hostile input gives an error, never a
/// panic, and memory use is proportional to the input.
pub fn parse_polymesh(files: &[FoamFile]) -> Result<PolyMesh> {
    let pf = require(files, "points")?;
    let mut lx = Lexer::new(&pf.name, &pf.bytes);
    header(&mut lx)?;
    let points: Vec<[f64; 3]> = lx.list(|l| {
        l.expect(Tok::LParen, "'('")?;
        let p = [l.scalar()?, l.scalar()?, l.scalar()?];
        l.expect(Tok::RParen, "')'")?;
        Ok(p)
    })?;
    only_end(&mut lx)?;

    let ff = require(files, "faces")?;
    let mut lx = Lexer::new(&ff.name, &ff.bytes);
    header(&mut lx)?;
    let (face_offsets, face_points) = faces_csr(&mut lx)?;
    only_end(&mut lx)?;

    let owner = label_file(require(files, "owner")?)?;
    let neighbour = label_file(require(files, "neighbour")?)?;
    let nf_total = face_offsets.len() - 1;
    let mal = |s: String| fe(FoamError::Malformed(s));
    if owner.len() != nf_total {
        return Err(mal(format!("{} faces but {} owners", nf_total, owner.len())));
    }
    if neighbour.len() > owner.len() {
        return Err(mal(format!("{} neighbours for {} faces", neighbour.len(), owner.len())));
    }
    let np = points.len();
    if let Some(v) = face_points.iter().find(|v| **v as usize >= np) {
        return Err(mal(format!("face point index {v} is past the {np} points")));
    }
    let max_cell = owner.iter().chain(&neighbour).max().copied();
    let n_cells = match max_cell {
        None => 0,
        Some(m) => m.checked_add(1).ok_or_else(|| mal("cell label overflows".into()))?,
    };

    let bf = require(files, "boundary")?;
    let mut lx = Lexer::new(&bf.name, &bf.bytes);
    header(&mut lx)?;
    struct P {
        name: String,
        ty: Option<String>,
        n: Option<u32>,
        start: Option<u32>,
    }
    let raw = dict_list(
        &mut lx,
        |n| P { name: n.to_string(), ty: None, n: None, start: None },
        |p, key, lx| {
            match key {
                "type" => p.ty = Some(lx.word("a patch type")?.to_string()),
                "nFaces" => p.n = Some(lx.label()?),
                "startFace" => p.start = Some(lx.label()?),
                _ => return lx.skip_to_semi(),
            }
            lx.skip_to_semi()
        },
    )?;
    only_end(&mut lx)?;
    let n_faces = nf_total as u64;
    let mut patches = Vec::new();
    for p in raw {
        let ty = p.ty.ok_or_else(|| mal(format!("patch '{}' has no type", p.name)))?;
        let patch_type = FoamPatchType::from_word(&ty).ok_or_else(|| fe(FoamError::Unsupported(format!("patch type '{ty}' (patch '{}')", p.name))))?;
        let (Some(n), Some(start)) = (p.n, p.start) else { return Err(mal(format!("patch '{}' needs nFaces and startFace", p.name))) };
        if u64::from(n) + u64::from(start) > n_faces {
            return Err(mal(format!("patch '{}' runs past the {n_faces} faces", p.name)));
        }
        patches.push(PolyPatch { name: p.name, patch_type, start_face: start, n_faces: n, origin: PatchOrigin::Parsed });
    }

    let mut cell_zones = Vec::new();
    if let Some(f) = find(files, "cellZones")? {
        let mut lx = Lexer::new(&f.name, &f.bytes);
        header(&mut lx)?;
        let z = dict_list(
            &mut lx,
            |n| PolyZone { name: n.to_string(), labels: Vec::new() },
            |z, key, lx| {
                if key == "cellLabels" {
                    z.labels = lx.list(|l| l.label())?;
                }
                lx.skip_to_semi()
            },
        )?;
        only_end(&mut lx)?;
        if let Some(l) = z.iter().flat_map(|z| &z.labels).find(|l| **l >= n_cells) {
            return Err(mal(format!("cellZones label {l} is past the {n_cells} cells")));
        }
        cell_zones = z;
    }
    let mut face_zones = Vec::new();
    if let Some(f) = find(files, "faceZones")? {
        let mut lx = Lexer::new(&f.name, &f.bytes);
        header(&mut lx)?;
        let z = dict_list(
            &mut lx,
            |n| PolyFaceZone { name: n.to_string(), labels: Vec::new(), flips: Vec::new() },
            |z, key, lx| {
                match key {
                    "faceLabels" => z.labels = lx.list(|l| l.label())?,
                    "flipMap" => z.flips = lx.list(|l| l.boolean())?,
                    _ => {}
                }
                lx.skip_to_semi()
            },
        )?;
        only_end(&mut lx)?;
        for z in &z {
            if z.labels.len() != z.flips.len() {
                return Err(mal(format!("faceZone '{}' has {} labels and {} flips", z.name, z.labels.len(), z.flips.len())));
            }
            if let Some(l) = z.labels.iter().find(|l| **l as u64 >= n_faces) {
                return Err(mal(format!("faceZones label {l} is past the {n_faces} faces")));
            }
        }
        face_zones = z;
    }

    Ok(PolyMesh { points, face_offsets, face_points, owner, neighbour, n_cells, patches, cell_zones, face_zones })
}

/// Read the polyMesh files of directory `dir` (normally `<case>/constant/polyMesh`) and parse
/// them with [`parse_polymesh`]. `cellZones` and `faceZones` are read when present.
pub fn read_polymesh_dir(dir: &Path) -> Result<PolyMesh> {
    let mut files = Vec::new();
    for name in ["points", "faces", "owner", "neighbour", "boundary"] {
        files.push(FoamFile { name: name.to_string(), bytes: std::fs::read(dir.join(name))? });
    }
    for name in ["cellZones", "faceZones"] {
        match std::fs::read(dir.join(name)) {
            Ok(bytes) => files.push(FoamFile { name: name.to_string(), bytes }),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => return Err(e.into()),
        }
    }
    parse_polymesh(&files)
}
