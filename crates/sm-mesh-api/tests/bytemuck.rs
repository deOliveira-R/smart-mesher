//! `bytemuck` feature: the index newtypes and `SignedFace` are `Pod` over `u32`.
#![cfg(feature = "bytemuck")]

use sm_mesh_api::{CellIx, EdgeIx, FaceIx, PointIx, Side, SignedFace, VertexIx};

#[test]
fn index_types_cast_to_u32_columns_and_back() {
    let v = [VertexIx(7), VertexIx(0), VertexIx(u32::MAX - 1)];
    assert_eq!(bytemuck::cast_slice::<VertexIx, u32>(&v), &[7, 0, u32::MAX - 1]);
    let back: &[VertexIx] = bytemuck::cast_slice(&[1u32, 2, 3]);
    assert_eq!(back, &[VertexIx(1), VertexIx(2), VertexIx(3)]);
    assert_eq!(bytemuck::cast_slice::<CellIx, u32>(&[CellIx(4)]), &[4]);
    assert_eq!(bytemuck::cast_slice::<FaceIx, u32>(&[FaceIx(5)]), &[5]);
    assert_eq!(bytemuck::cast_slice::<EdgeIx, u32>(&[EdgeIx(6)]), &[6]);
    assert_eq!(bytemuck::cast_slice::<PointIx, u32>(&[PointIx(8)]), &[8]);
    assert_eq!(<VertexIx as bytemuck::Zeroable>::zeroed(), VertexIx(0));
}

#[test]
fn signed_face_casts_to_its_raw_word() {
    let sf = [SignedFace::new(FaceIx(5), Side::Right), SignedFace::new(FaceIx(9), Side::Left)];
    let raw: &[u32] = bytemuck::cast_slice(&sf);
    assert_eq!(raw, &[sf[0].to_raw(), sf[1].to_raw()]);
    assert_eq!(raw, &[11, 18]);
    let back: &[SignedFace] = bytemuck::cast_slice(raw);
    assert_eq!(back, &sf);
    // Arrays of indices (edge endpoints) cast too.
    let e = [[VertexIx(1), VertexIx(2)]];
    assert_eq!(bytemuck::cast_slice::<[VertexIx; 2], u32>(&e), &[1, 2]);
}
