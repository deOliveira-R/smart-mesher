//! STEP files written by Open CASCADE 7.8 (committed under `data/step/`) with their analytic
//! volumes. They are inputs for `sm-cadio` acceptance tests; the bytes are pinned by sha256.

use core::f64::consts::PI;

/// One committed STEP file.
#[derive(Clone, Copy, Debug)]
pub struct StepFixture {
    /// File stem.
    pub name: &'static str,
    /// The file contents.
    pub bytes: &'static [u8],
    /// Lowercase hex sha256 of `bytes`.
    pub sha256: &'static str,
    /// Number of solids in the file.
    pub solids: usize,
    /// Total volume of the solids (closed form, except `loft_bspline`, which is the Open CASCADE
    /// value).
    pub volume: f64,
    /// Relative tolerance to compare a computed volume with `volume`.
    pub rel_tol: f64,
    /// A text that must occur in the file: the defining quadric or surface entity.
    pub marker: &'static str,
}

/// Cylinder, radius 2, height 5: volume `20 pi`.
pub const CYLINDER_R2_H5: StepFixture = StepFixture {
    name: "cylinder_r2_h5",
    bytes: include_bytes!("../data/step/cylinder_r2_h5.step"),
    sha256: "2bab00083ec845fd101da0da9b57c5ed503a56c4b8b67d2ce1d8e148be062420",
    solids: 1,
    volume: 20.0 * PI,
    rel_tol: 1e-9,
    marker: "CYLINDRICAL_SURFACE('',#32,2.)",
};

/// Sphere, radius 3: volume `36 pi`.
pub const SPHERE_R3: StepFixture = StepFixture {
    name: "sphere_r3",
    bytes: include_bytes!("../data/step/sphere_r3.step"),
    sha256: "f5a5cd1024a15ffa17e9ebebf45c9c191c37660da649b6e06507eeb6441a899d",
    solids: 1,
    volume: 36.0 * PI,
    rel_tol: 1e-9,
    marker: "SPHERICAL_SURFACE('',#23,3.)",
};

/// Torus, major radius 5, minor radius 1: volume `2 pi^2 R r^2 = 10 pi^2`.
pub const TORUS_R5_R1: StepFixture = StepFixture {
    name: "torus_R5_r1",
    bytes: include_bytes!("../data/step/torus_R5_r1.step"),
    sha256: "6322f5879a909de5f1a68019e9768657bdd0263bac1dafd68cfcb9f0782b0f7a",
    solids: 1,
    volume: 10.0 * PI * PI,
    rel_tol: 1e-9,
    marker: "TOROIDAL_SURFACE('',#32,5.,1.)",
};

/// Cone with base radius 2 and height 3: volume `pi r^2 h / 3 = 4 pi`.
pub const CONE_R2_H3: StepFixture = StepFixture {
    name: "cone_r2_h3",
    bytes: include_bytes!("../data/step/cone_r2_h3.step"),
    sha256: "0f96efee0e3d2f3c203e5f554aab150f9f5ee5a297ff7d2d645b54b566ac6704",
    solids: 1,
    volume: 4.0 * PI,
    rel_tol: 1e-9,
    marker: "CONICAL_SURFACE('',#33,2.,0.588002603548)",
};

/// The cube `[0,10]^3` with one edge (length 10) filleted by radius 1: the fillet removes
/// `(1 - pi/4) r^2` of cross-section along the edge, so the volume is `1000 - (1 - pi/4) 10`.
pub const BOX_FILLET1: StepFixture = StepFixture {
    name: "box_fillet1",
    bytes: include_bytes!("../data/step/box_fillet1.step"),
    sha256: "d9a4357ee641cb0167f3d8cb345d8d5f70700c20679c98fe676f97dda692ef57",
    solids: 1,
    volume: 1000.0 - (1.0 - PI / 4.0) * 10.0,
    rel_tol: 1e-9,
    marker: "CYLINDRICAL_SURFACE('',#73,1.)",
};

/// A B-spline loft (no closed form); the volume is the Open CASCADE value, to 1e-6 relative.
pub const LOFT_BSPLINE: StepFixture = StepFixture {
    name: "loft_bspline",
    bytes: include_bytes!("../data/step/loft_bspline.step"),
    sha256: "306ca9a5040664dc46f3692aed12b74acba702fa403076d5f98c4e4adaf0689a",
    solids: 1,
    volume: 54.454271953,
    rel_tol: 1e-6,
    marker: "B_SPLINE_SURFACE",
};

/// The three-solid W3a milestone part (see [`crate::w3a`]): total volume 2000.
pub const W3A_PART: StepFixture = StepFixture {
    name: "w3a_part",
    bytes: crate::w3a::STEP,
    sha256: "252eb1e6377ebd4090287def170c3c6442b0f0c9f286308cf75b037606bb463d",
    solids: 3,
    volume: crate::w3a::TOTAL_VOLUME,
    rel_tol: 1e-9,
    marker: "MANIFOLD_SOLID_BREP",
};

/// Every committed STEP file, in a fixed order.
pub const ALL: [StepFixture; 7] = [CYLINDER_R2_H5, SPHERE_R3, TORUS_R5_R1, CONE_R2_H3, BOX_FILLET1, LOFT_BSPLINE, W3A_PART];
