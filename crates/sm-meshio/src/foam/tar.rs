//! A deterministic ustar writer, so the multi-file polyMesh fits a `Write`-based export.

use super::PolyMesh;
use crate::error::{MeshioError, Result};
use std::io::Write;

fn octal(field: &mut [u8], v: u64) {
    // Zero-padded octal filling all but the last byte, which stays NUL.
    let n = field.len() - 1;
    let s = format!("{v:0n$o}");
    field[..n].copy_from_slice(s.as_bytes());
}

fn header(name: &str, size: u64) -> Result<[u8; 512]> {
    let (prefix, base) = if name.len() <= 100 {
        ("", name)
    } else {
        // ustar splits a long path at a '/' into prefix (<= 155) and name (<= 100).
        let cut = name.char_indices().filter(|(_, c)| *c == '/').map(|(i, _)| i).rev().find(|&i| i <= 155 && name.len() - i - 1 <= 100 && name.len() - i - 1 > 0);
        match cut {
            Some(i) => (&name[..i], &name[i + 1..]),
            None => return Err(MeshioError::Unsupported(format!("tar path '{name}' does not fit a ustar header"))),
        }
    };
    let mut h = [0u8; 512];
    h[..base.len()].copy_from_slice(base.as_bytes());
    octal(&mut h[100..108], 0o644);
    octal(&mut h[108..116], 0);
    octal(&mut h[116..124], 0);
    octal(&mut h[124..136], size);
    octal(&mut h[136..148], 0);
    h[156] = b'0';
    h[257..263].copy_from_slice(b"ustar\0");
    h[263..265].copy_from_slice(b"00");
    octal(&mut h[329..337], 0);
    octal(&mut h[337..345], 0);
    h[345..345 + prefix.len()].copy_from_slice(prefix.as_bytes());
    h[148..156].copy_from_slice(b"        ");
    let sum: u32 = h.iter().map(|b| u32::from(*b)).sum();
    let s = format!("{sum:06o}");
    h[148..154].copy_from_slice(s.as_bytes());
    h[154] = 0;
    h[155] = b' ';
    Ok(h)
}

impl PolyMesh {
    /// Write [`PolyMesh::files`] as a deterministic ustar archive: entries `<prefix><name>` in
    /// sorted order, mode 0644, uid and gid 0, mtime 0, empty user and group names, two zero
    /// blocks at the end. The bytes are a pure function of the mesh. `prefix` is usually
    /// `"constant/polyMesh/"` (include the trailing slash) or `""`. Paths longer than a ustar
    /// header holds are [`MeshioError::Unsupported`].
    pub fn write_tar<W: Write>(&self, w: &mut W, prefix: &str) -> Result<()> {
        let mut files = self.files()?;
        files.sort_by(|a, b| a.name.cmp(&b.name));
        for f in &files {
            w.write_all(&header(&format!("{prefix}{}", f.name), f.bytes.len() as u64)?)?;
            w.write_all(&f.bytes)?;
            let pad = (512 - f.bytes.len() % 512) % 512;
            w.write_all(&[0u8; 512][..pad])?;
        }
        w.write_all(&[0u8; 1024])?;
        Ok(())
    }
}
