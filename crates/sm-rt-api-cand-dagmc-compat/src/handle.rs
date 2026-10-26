//! Dense query handles (the analogue of DAGMC's `EntityHandle`).

/// A volume in a [`crate::World`]. Dense: `0..volume_count()`, ascending in the source
/// `EntityId` order. The implicit complement is a normal volume (see
/// [`crate::World::implicit_complement`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VolHandle(pub u32);

/// A surface (one B-rep face, or one group of coplanar-by-classification mesh faces) in a
/// [`crate::World`]. Dense: `0..surface_count()`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SurfHandle(pub u32);

impl VolHandle {
    /// Index as `usize`.
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

impl SurfHandle {
    /// Index as `usize`.
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// One facet (triangle or analytic patch) of a surface. Used in [`crate::RayHistory`] so a ray
/// restarted on a surface cannot re-hit the facet it just crossed. Analytic backends use
/// `index = 0` for a whole analytic surface.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FacetId {
    /// Owning surface.
    pub surface: SurfHandle,
    /// Facet index within the surface.
    pub index: u32,
}

/// Sense of a surface with respect to a volume (DAGMC: `+1` forward, `-1` reverse).
///
/// The surface normal points **out of** its forward volume and **into** its reverse volume
/// (the same convention as `sm_topo_api::Side::Left` = forward).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SurfSense {
    /// The volume is on the side the normal points away from.
    Forward,
    /// The volume is on the side the normal points to.
    Reverse,
}

impl SurfSense {
    /// DAGMC's integer encoding: `+1` forward, `-1` reverse.
    pub const fn as_i32(self) -> i32 {
        match self {
            SurfSense::Forward => 1,
            SurfSense::Reverse => -1,
        }
    }
    /// `+1.0` or `-1.0`.
    pub const fn sign(self) -> f64 {
        match self {
            SurfSense::Forward => 1.0,
            SurfSense::Reverse => -1.0,
        }
    }
}
