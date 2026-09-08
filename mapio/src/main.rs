use mapio::StaticMap;
use std::path::PathBuf;

fn main() -> anyhow::Result<()> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| format!("{}/tb3_map.yaml", std::env::var("HOME").unwrap()));

    println!("Loading map from: {path}");
    let map = StaticMap::load(&PathBuf::from(path))?;

    println!("Map loaded:");
    println!("  Size: {} x {} cells", map.width, map.height);
    println!("  Resolution: {:.3} m/cell", map.resolution);
    println!("  Origin: ({:.3}, {:.3})", map.origin_x, map.origin_y);
    println!("  Occupied cells: {}", map.occupied_count());
    println!("  Free cells: {}", map.free_count());
    println!("  Unknown cells: {}", map.unknown_count());

    // ASCII dump of the whole map, y flipped so it prints "upright"
    // (row 0 of our output = top = highest y)
    println!();
    println!("Map ASCII (# occupied, . free, ? unknown):");
    for y in (0..map.height).rev() {
        let mut row = String::new();
        for x in 0..map.width {
            let c = if map.is_occupied(x, y) { '#' }
                else if map.is_free(x, y) { '.' }
                else { '?' };
            row.push(c);
        }
        println!("{row}");
    }
    println!();

    // Sanity check: convert a world point to a cell and back
    if let Some((cx, cy)) = map.world_to_cell(0.0, 0.0) {
        let (wx, wy) = map.cell_to_world(cx, cy);
        println!(
            "  World (0.0, 0.0) -> cell ({cx}, {cy}) -> world ({:.3}, {:.3})",
            wx, wy
        );
        println!("  Cell occupied: {}", map.is_occupied(cx, cy));
    } else {
        println!("  World (0.0, 0.0) is out of map bounds");
    }

    Ok(())
}
