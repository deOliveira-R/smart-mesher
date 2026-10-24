//! Wall-clock timing of building and querying a structured hex block: `cargo run --release -p
//! sm-mesh --example timing -- 100 100 100` (default 100^3 = 10^6 hexahedra).

use sm_mesh::generate::{hex_block, hex_block_bulk};
use sm_mesh::MeshBuilder;
use sm_mesh_api::{MeshRead, SerialAlloc, connectivity_hash, geometry_hash, topology_hash, validate};
use std::time::Instant;

fn main() {
    let a: Vec<u32> = std::env::args().skip(1).filter_map(|s| s.parse().ok()).collect();
    let n = if a.len() == 3 { [a[0], a[1], a[2]] } else { [100, 100, 100] };
    println!("threads: {}", rayon::current_num_threads());
    let t = Instant::now();
    let bulk = hex_block_bulk(n, [1.0; 3], &mut SerialAlloc::new()).map(|f| f.mesh);
    println!("bulk build + finish (with validate): {:?}", t.elapsed());
    let Ok(m) = bulk else { return };
    println!("cells {} faces {} vertices {}", m.cell_count(), m.face_count(), m.vertex_count());
    let t = Instant::now();
    let v = validate(&m);
    println!("validate (sequential contract validator): {:?} ({} violations)", t.elapsed(), v.len());
    let t = Instant::now();
    let h = (connectivity_hash(&m), topology_hash(&m), geometry_hash(&m));
    println!("hashes: {:?} {h:x?}", t.elapsed());
    let t = Instant::now();
    let inc = hex_block(MeshBuilder::new(), n, [1.0; 3], false, &mut SerialAlloc::new());
    println!("incremental build + finish: {:?} ok={}", t.elapsed(), inc.is_ok());
    let t = Instant::now();
    let vc = m.vertex_cells().indices().len();
    println!("vertex_cells: {:?} ({vc} entries)", t.elapsed());
    let t = Instant::now();
    let nb = m.cell_neighbors().indices().len();
    println!("cell_neighbors: {:?} ({nb} entries)", t.elapsed());
}
