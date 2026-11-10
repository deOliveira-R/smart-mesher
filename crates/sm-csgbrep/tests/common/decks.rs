//! Inline OpenMC decks built from text (cylinder and plane surfaces only).
#![allow(dead_code)]

/// `n x n` lattice of one pin universe (fuel r1, void gap to r2, clad to r3, moderator) with
/// pitch `p` and height `h`, vacuum on the container box. With `outer` the container is the
/// array box plus a margin of `p / 2` on x and y, and the rest is material 4.
pub fn pin_lattice(n: usize, p: f64, radii: (f64, f64, f64), h: f64, outer: bool) -> String {
    pin_lattice_ex(n, p, radii, h, outer, 0, false)
}

/// [`pin_lattice`] with every surface, cell and universe id shifted by `shift` and, with
/// `reverse`, the elements written in the opposite order. The lattice id stays 20 (the frame is
/// named after it).
pub fn pin_lattice_ex(n: usize, p: f64, (r1, r2, r3): (f64, f64, f64), h: f64, outer: bool, shift: u32, reverse: bool) -> String {
    let sh = |i: u32| i + shift;
    let half = n as f64 * p / 2.0;
    let m = if outer { half + p / 2.0 } else { half };
    let uni = vec![sh(10).to_string(); n * n].chunks(n).map(|c| c.join(" ")).collect::<Vec<_>>().join("\n");
    let mut cells = vec![
        format!("  <cell id=\"{}\" fill=\"20\" region=\"{} -{} {} -{} {} -{}\" universe=\"0\"/>\n", sh(1), sh(11), sh(12), sh(13), sh(14), sh(15), sh(16)),
        format!("  <cell id=\"{}\" material=\"1\" region=\"-{}\" universe=\"{}\"/>\n", sh(100), sh(1), sh(10)),
        format!("  <cell id=\"{}\" material=\"void\" region=\"{} -{}\" universe=\"{}\"/>\n", sh(101), sh(1), sh(2), sh(10)),
        format!("  <cell id=\"{}\" material=\"2\" region=\"{} -{}\" universe=\"{}\"/>\n", sh(102), sh(2), sh(3), sh(10)),
        format!("  <cell id=\"{}\" material=\"3\" region=\"{}\" universe=\"{}\"/>\n", sh(103), sh(3), sh(10)),
    ];
    if outer {
        cells.push(format!("  <cell id=\"{}\" material=\"4\" universe=\"{}\"/>\n", sh(900), sh(30)));
    }
    let mut surfs = Vec::new();
    for (id, r) in [(1, r1), (2, r2), (3, r3)] {
        surfs.push(format!("  <surface id=\"{}\" type=\"z-cylinder\" coeffs=\"0.0 0.0 {r}\"/>\n", sh(id)));
    }
    for (id, ty, c) in [(11, "x-plane", -m), (12, "x-plane", m), (13, "y-plane", -m), (14, "y-plane", m), (15, "z-plane", -h / 2.0), (16, "z-plane", h / 2.0)] {
        surfs.push(format!("  <surface id=\"{}\" type=\"{ty}\" boundary=\"vacuum\" coeffs=\"{c}\"/>\n", sh(id)));
    }
    if reverse {
        cells.reverse();
        surfs.reverse();
    }
    let mut s = String::from("<?xml version='1.0' encoding='UTF-8'?>\n<geometry>\n");
    s.extend(cells);
    s.push_str(&format!("  <lattice id=\"20\">\n    <pitch>{p} {p}</pitch>\n"));
    if outer {
        s.push_str(&format!("    <outer>{}</outer>\n", sh(30)));
    }
    s.push_str(&format!("    <dimension>{n} {n}</dimension>\n    <lower_left>{} {}</lower_left>\n    <universes>\n{uni}\n</universes>\n  </lattice>\n", -half, -half));
    s.extend(surfs);
    s.push_str("</geometry>\n");
    s
}

/// A single z-cylinder (`r`, centre `(cx, cy)`) in a box `[-b, b]^2 x [0, h]`, split by the plane
/// `x = 0` into two moderator halves, so the moderator of the cylinder's cell is two pieces
/// only when the cylinder cuts it (here the cylinder lies in `x > 0`, so the moderator stays one
/// cell that the plane `x = 0` would split in a naive partition).
pub fn disconnected_moderator(b: f64, h: f64) -> String {
    // Cell 2: moderator = box minus the slab |x| < 0.25 (two disconnected parts); cell 3: the slab.
    let mut s = String::from("<?xml version='1.0' encoding='UTF-8'?>\n<geometry>\n");
    s.push_str("  <cell id=\"1\" material=\"1\" region=\"21 -22 11 -12 13 -14 15 -16\" universe=\"1\"/>\n");
    s.push_str("  <cell id=\"2\" material=\"2\" region=\"(-21 | 22) 11 -12 13 -14 15 -16\" universe=\"1\"/>\n");
    s.push_str("  <surface id=\"21\" type=\"x-plane\" coeffs=\"-0.25\"/>\n  <surface id=\"22\" type=\"x-plane\" coeffs=\"0.25\"/>\n");
    for (id, ty, c) in [(11, "x-plane", -b), (12, "x-plane", b), (13, "y-plane", -b), (14, "y-plane", b), (15, "z-plane", 0.0), (16, "z-plane", h)] {
        s.push_str(&format!("  <surface id=\"{id}\" type=\"{ty}\" boundary=\"vacuum\" coeffs=\"{c}\"/>\n"));
    }
    s.push_str("</geometry>\n");
    s
}

/// A z-cylinder written as a general `quadric` (`(x-cx)^2 + (y-cy)^2 - r^2`) in a vacuum box.
pub fn quadric_cylinder(r: f64, (cx, cy): (f64, f64), b: f64, h: f64) -> String {
    let k = cx * cx + cy * cy - r * r;
    let mut s = String::from("<?xml version='1.0' encoding='UTF-8'?>\n<geometry>\n");
    s.push_str("  <cell id=\"1\" material=\"1\" region=\"-1 11 -12 13 -14 15 -16\" universe=\"1\"/>\n");
    s.push_str("  <cell id=\"2\" material=\"2\" region=\"1 11 -12 13 -14 15 -16\" universe=\"1\"/>\n");
    s.push_str(&format!("  <surface id=\"1\" type=\"quadric\" coeffs=\"1 1 0 0 0 0 {} {} 0 {k}\"/>\n", -2.0 * cx, -2.0 * cy));
    for (id, ty, c) in [(11, "x-plane", -b), (12, "x-plane", b), (13, "y-plane", -b), (14, "y-plane", b), (15, "z-plane", 0.0), (16, "z-plane", h)] {
        s.push_str(&format!("  <surface id=\"{id}\" type=\"{ty}\" boundary=\"vacuum\" coeffs=\"{c}\"/>\n"));
    }
    s.push_str("</geometry>\n");
    s
}

/// A universe 5 (cylinder `r` at `(0.2, 0.1)` in material 1, the rest of the box material 2)
/// placed by one filling cell with a translation and a rotation (`rot` is the OpenMC `rotation`
/// text), in a vacuum box.
pub fn placed_cylinder(t: [f64; 3], rot: Option<&str>, r: f64) -> String {
    let mut s = String::from("<?xml version='1.0' encoding='UTF-8'?>\n<geometry>\n");
    let rot = rot.map(|r| format!(" rotation=\"{r}\"")).unwrap_or_default();
    s.push_str(&format!("  <cell id=\"1\" fill=\"5\" region=\"54 -55 56 -57 58 -59\" translation=\"{} {} {}\"{rot} universe=\"0\"/>\n", t[0], t[1], t[2]));
    s.push_str("  <cell id=\"101\" material=\"1\" region=\"-53\" universe=\"5\"/>\n");
    s.push_str("  <cell id=\"102\" material=\"2\" region=\"53\" universe=\"5\"/>\n");
    s.push_str(&format!("  <surface id=\"53\" type=\"z-cylinder\" coeffs=\"0.2 0.1 {r}\"/>\n"));
    // The filled box is [-1.5, 1.5]^2 x [-1, 1] about the translation, in the cell frame.
    for (id, ty, c) in [(54, "x-plane", t[0] - 1.5), (55, "x-plane", t[0] + 1.5), (56, "y-plane", t[1] - 1.5), (57, "y-plane", t[1] + 1.5), (58, "z-plane", t[2] - 1.0), (59, "z-plane", t[2] + 1.0)] {
        s.push_str(&format!("  <surface id=\"{id}\" type=\"{ty}\" boundary=\"vacuum\" coeffs=\"{c}\"/>\n"));
    }
    s.push_str("</geometry>\n");
    s
}

/// `n x n` lattice of two pin universes in a checkerboard (universe 10 where `i + j` is even,
/// 11 where it is odd), each with fuel, gap and clad circles (radii `ra` and `rb`) and a
/// moderator; pitch `p`, height `h`, vacuum on the container box; `outer` as in [`pin_lattice`].
pub fn mixed_lattice(n: usize, p: f64, ra: (f64, f64, f64), rb: (f64, f64, f64), h: f64, outer: bool) -> String {
    let half = n as f64 * p / 2.0;
    let m = if outer { half + p / 2.0 } else { half };
    // Rows are written top to bottom (the reader reverses OpenMC's row order); a checkerboard
    // is symmetric enough that the parity of `i + j` is what matters for the volumes.
    let rows: Vec<String> = (0..n).map(|r| (0..n).map(|c| if (r + c) % 2 == 0 { "10" } else { "11" }).collect::<Vec<_>>().join(" ")).collect();
    let mut s = String::from("<?xml version='1.0' encoding='UTF-8'?>\n<geometry>\n");
    s.push_str("  <cell id=\"1\" fill=\"20\" region=\"11 -12 13 -14 15 -16\" universe=\"0\"/>\n");
    for (u, base) in [(10, 1), (11, 4)] {
        s.push_str(&format!("  <cell id=\"{}\" material=\"1\" region=\"-{}\" universe=\"{u}\"/>\n", u * 10, base));
        s.push_str(&format!("  <cell id=\"{}\" material=\"void\" region=\"{} -{}\" universe=\"{u}\"/>\n", u * 10 + 1, base, base + 1));
        s.push_str(&format!("  <cell id=\"{}\" material=\"2\" region=\"{} -{}\" universe=\"{u}\"/>\n", u * 10 + 2, base + 1, base + 2));
        s.push_str(&format!("  <cell id=\"{}\" material=\"3\" region=\"{}\" universe=\"{u}\"/>\n", u * 10 + 3, base + 2));
    }
    if outer {
        s.push_str("  <cell id=\"900\" material=\"4\" universe=\"30\"/>\n");
    }
    s.push_str(&format!("  <lattice id=\"20\">\n    <pitch>{p} {p}</pitch>\n"));
    if outer {
        s.push_str("    <outer>30</outer>\n");
    }
    s.push_str(&format!("    <dimension>{n} {n}</dimension>\n    <lower_left>{} {}</lower_left>\n    <universes>\n{}\n</universes>\n  </lattice>\n", -half, -half, rows.join("\n")));
    for (id, r) in [(1, ra.0), (2, ra.1), (3, ra.2), (4, rb.0), (5, rb.1), (6, rb.2)] {
        s.push_str(&format!("  <surface id=\"{id}\" type=\"z-cylinder\" coeffs=\"0.0 0.0 {r}\"/>\n"));
    }
    for (id, ty, c) in [(11, "x-plane", -m), (12, "x-plane", m), (13, "y-plane", -m), (14, "y-plane", m), (15, "z-plane", -h / 2.0), (16, "z-plane", h / 2.0)] {
        s.push_str(&format!("  <surface id=\"{id}\" type=\"{ty}\" boundary=\"vacuum\" coeffs=\"{c}\"/>\n"));
    }
    s.push_str("</geometry>\n");
    s
}
