//! Dense index newtypes. All are `u32` with a `NONE` sentinel so they pack into SoA columns
//! and cross the C ABI as plain integers.

macro_rules! rt_index {
    ($(#[$m:meta])* $name:ident) => {
        $(#[$m])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
        #[repr(transparent)]
        pub struct $name(pub u32);
        impl $name {
            /// Sentinel meaning "none" / "outside".
            pub const NONE: Self = Self(u32::MAX);
            /// Build from a `usize` (debug-asserts it is below the sentinel).
            pub const fn new(i: usize) -> Self {
                debug_assert!(i < u32::MAX as usize);
                Self(i as u32)
            }
            /// The index as `usize`, for slicing.
            pub const fn get(self) -> usize {
                self.0 as usize
            }
            /// True if this is the sentinel.
            pub const fn is_none(self) -> bool {
                self.0 == u32::MAX
            }
        }
    };
}

rt_index! {
    /// A material region: what the physics sees (a B-rep volume, a block). Several tracking
    /// cells share one region. `RegionId::NONE` is "outside the model".
    RegionId
}
rt_index! {
    /// A tracking cell: the finest unit a particle walks through and a tally can bin by.
    /// In a mesh-native tracker it is the mesh cell index (`CellIx`) in the flattened,
    /// instance-numbered mesh; in a facet-only tracker it equals the region.
    CellId
}
rt_index! {
    /// A boundary surface: a set of faces (a B-rep face, a mesh boundary set). This is the
    /// DAGMC "surface"; it is what reflect/vacuum/periodic decks refer to.
    SurfaceId
}
rt_index! {
    /// One oriented face of the tracking structure (a mesh face, or a facet triangle).
    /// Crossing it is exact: the neighbour cell is looked up, not searched for.
    FaceId
}
rt_index! {
    /// A component (prototype) of a hierarchical world.
    ComponentId
}
rt_index! {
    /// A placed instance of a component. Cell numbering is dense over instances in
    /// ascending `InstanceId`, so global `CellId`s never collide.
    InstanceId
}

impl CellId {
    /// The implicit complement: space inside the domain that no cell covers (DAGMC's implicit
    /// complement). It has no mesh cell; the tracker uses the boundary-face BVH there.
    pub const COMPLEMENT: Self = Self(u32::MAX - 1);
}
