//! CRC-32 (IEEE 802.3, the polynomial of zlib/PNG/gzip), slicing-by-8. `zlib.crc32` in Python
//! gives the same value, so the reference reader can verify arrays.

const POLY: u32 = 0xEDB8_8320;

const fn make_tables() -> [[u32; 256]; 8] {
    let mut t = [[0u32; 256]; 8];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 { (c >> 1) ^ POLY } else { c >> 1 };
            k += 1;
        }
        t[0][i] = c;
        i += 1;
    }
    let mut i = 0;
    while i < 256 {
        let mut s = 1;
        while s < 8 {
            let p = t[s - 1][i];
            t[s][i] = (p >> 8) ^ t[0][(p & 0xFF) as usize];
            s += 1;
        }
        i += 1;
    }
    t
}

static TABLES: [[u32; 256]; 8] = make_tables();

/// CRC-32 of `data`.
pub(crate) fn crc32(data: &[u8]) -> u32 {
    let t = &TABLES;
    let mut c = !0u32;
    let (chunks, rest) = data.as_chunks::<8>();
    for ch in chunks {
        let lo = u32::from_le_bytes([ch[0], ch[1], ch[2], ch[3]]) ^ c;
        let hi = u32::from_le_bytes([ch[4], ch[5], ch[6], ch[7]]);
        c = t[7][(lo & 0xFF) as usize]
            ^ t[6][((lo >> 8) & 0xFF) as usize]
            ^ t[5][((lo >> 16) & 0xFF) as usize]
            ^ t[4][(lo >> 24) as usize]
            ^ t[3][(hi & 0xFF) as usize]
            ^ t[2][((hi >> 8) & 0xFF) as usize]
            ^ t[1][((hi >> 16) & 0xFF) as usize]
            ^ t[0][(hi >> 24) as usize];
    }
    for &b in rest {
        c = (c >> 8) ^ t[0][((c ^ b as u32) & 0xFF) as usize];
    }
    !c
}

#[cfg(test)]
mod tests {
    use super::crc32;

    #[test]
    fn known_vectors() {
        assert_eq!(crc32(b""), 0);
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        let long: Vec<u8> = (0..1000u32).map(|i| (i * 7 + 3) as u8).collect();
        // Slicing-by-8 must agree with the byte-wise definition.
        let mut c = !0u32;
        for &b in &long {
            c = (c >> 8) ^ super::TABLES[0][((c ^ b as u32) & 0xFF) as usize];
        }
        assert_eq!(crc32(&long), !c);
    }
}
