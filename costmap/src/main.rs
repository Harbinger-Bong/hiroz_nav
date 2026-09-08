use clap::Parser;
use hiroz::{
    Builder, Result,
    context::ZContextBuilder,
};
use hiroz_msgs::sensor_msgs::LaserScan;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

#[derive(Debug, Parser)]
struct Args {
    #[arg(short, long, default_value = "peer")]
    mode: String,
    #[arg(short, long)]
    endpoint: Option<String>,
    /// Grid resolution in meters per cell
    #[arg(short, long, default_value = "0.05")]
    resolution: f32,
    /// Grid size in meters (square, centered on robot)
    #[arg(short, long, default_value = "10.0")]
    size: f32,
}

/// Minimal 2D occupancy grid. 0 = free, 1 = occupied, -1 = unknown.
struct OccupancyGrid {
    resolution: f32,
    width_cells: usize,
    height_cells: usize,
    cells: Vec<i8>,
}

impl OccupancyGrid {
    fn new(size_m: f32, resolution: f32) -> Self {
        let n = (size_m / resolution).ceil() as usize;
        Self {
            resolution,
            width_cells: n,
            height_cells: n,
            cells: vec![-1; n * n],
        }
    }

    fn world_to_cell(&self, x: f32, y: f32) -> Option<(usize, usize)> {
        let half = (self.width_cells as f32 * self.resolution) / 2.0;
        let cx = ((x + half) / self.resolution) as isize;
        let cy = ((y + half) / self.resolution) as isize;
        if cx < 0 || cy < 0 || cx as usize >= self.width_cells || cy as usize >= self.height_cells {
            return None;
        }
        Some((cx as usize, cy as usize))
    }

    fn mark_occupied(&mut self, x: f32, y: f32) {
        if let Some((cx, cy)) = self.world_to_cell(x, y) {
            self.cells[cy * self.width_cells + cx] = 1;
        }
    }

    fn occupied_count(&self) -> usize {
        self.cells.iter().filter(|&&c| c == 1).count()
    }
}

/// Convert a LaserScan into occupied cells, assuming the scan origin is at (0,0)
/// in its own frame (robot-centered costmap — no odom fusion yet).
fn integrate_scan(grid: &mut OccupancyGrid, scan: &LaserScan) {
    let mut angle = scan.angle_min;
    for &range in scan.ranges.iter() {
        if range.is_finite() && range >= scan.range_min && range <= scan.range_max {
            let x = range * angle.cos();
            let y = range * angle.sin();
            grid.mark_occupied(x, y);
        }
        angle += scan.angle_increment;
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    zenoh::init_log_from_env_or("error");

    let format = hiroz_protocol::KeyExprFormat::RmwZenoh;
    let ctx = if let Some(e) = args.endpoint {
        ZContextBuilder::default()
            .with_mode(args.mode)
            .with_connect_endpoints([e])
            .keyexpr_format(format)
            .build()?
    } else {
        ZContextBuilder::default()
            .with_mode(args.mode)
            .keyexpr_format(format)
            .build()?
    };

    println!("=== Costmap Node (Hiroz) ===");

    let node = ctx.create_node("costmap").build()?;
    println!("Node created: /costmap");

    let scan_sub = node.create_sub::<LaserScan>("/scan").build()?;
    println!("Subscribed to /scan");
    println!("Grid: {:.2}m x {:.2}m @ {:.2}m/cell", args.size, args.size, args.resolution);
    println!();

    let count = Arc::new(AtomicUsize::new(0));
    let count_c = count.clone();

    while let Ok(msg) = scan_sub.async_recv().await {
        let n = count_c.fetch_add(1, Ordering::Relaxed);
        let mut grid = OccupancyGrid::new(args.size, args.resolution);
        integrate_scan(&mut grid, &msg);

        if n % 5 == 0 {
            println!(
                "[COSTMAP #{n}] grid={}x{} cells, occupied={}, scan_ranges={}",
                grid.width_cells,
                grid.height_cells,
                grid.occupied_count(),
                msg.ranges.len()
            );
        }
    }

    Ok(())
}
