use clap::Parser;
use hiroz::{Builder, Result, context::ZContextBuilder};
use hiroz_msgs::geometry_msgs::PoseStamped;
use hiroz_msgs::nav_msgs::{Odometry, Path};
use mapio::StaticMap;
use std::collections::{BinaryHeap, HashMap};
use std::cmp::Ordering;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

#[derive(Debug, Parser)]
struct Args {
    #[arg(short, long, default_value = "peer")]
    mode: String,
    #[arg(short, long)]
    endpoint: Option<String>,
    #[arg(long)]
    map: Option<String>,
}

#[derive(Copy, Clone, Eq, PartialEq)]
struct Node {
    cost: u32,
    cell: (usize, usize),
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}
impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn nearest_passable(map: &StaticMap, from: (usize, usize), max_radius: i64) -> Option<(usize, usize)> {
    if map.is_free(from.0, from.1) {
        return Some(from);
    }
    for r in 1..=max_radius {
        for dx in -r..=r {
            for dy in -r..=r {
                if dx.abs() != r && dy.abs() != r { continue; }
                let nx = from.0 as i64 + dx;
                let ny = from.1 as i64 + dy;
                if nx < 0 || ny < 0 || nx as usize >= map.width || ny as usize >= map.height {
                    continue;
                }
                let cell = (nx as usize, ny as usize);
                if map.is_free(cell.0, cell.1) {
                    return Some(cell);
                }
            }
        }
    }
    None
}

fn astar(map: &StaticMap, start: (usize, usize), goal: (usize, usize)) -> Option<Vec<(f32, f32)>> {
    let w = map.width;
    let h = map.height;
    let idx = |c: (usize, usize)| c.1 * w + c.0;

    let heuristic = |a: (usize, usize), b: (usize, usize)| -> u32 {
        let dx = (a.0 as i64 - b.0 as i64).abs();
        let dy = (a.1 as i64 - b.1 as i64).abs();
        (10 * (dx + dy) - 6 * dx.min(dy)) as u32
    };

    let mut open = BinaryHeap::new();
    let mut g_score: HashMap<usize, u32> = HashMap::new();
    let mut came_from: HashMap<usize, (usize, usize)> = HashMap::new();

    g_score.insert(idx(start), 0);
    open.push(Node { cost: heuristic(start, goal), cell: start });

    let neighbors = [
        (-1i64, 0i64, 10u32), (1, 0, 10), (0, -1, 10), (0, 1, 10),
        (-1, -1, 14), (-1, 1, 14), (1, -1, 14), (1, 1, 14),
    ];

    while let Some(Node { cell, .. }) = open.pop() {
        if cell == goal {
            let mut path = vec![cell];
            let mut cur = cell;
            while let Some(&prev) = came_from.get(&idx(cur)) {
                path.push(prev);
                cur = prev;
            }
            path.reverse();
            return Some(path.into_iter().map(|c| map.cell_to_world(c.0, c.1)).collect());
        }

        let current_g = *g_score.get(&idx(cell)).unwrap_or(&u32::MAX);

        for (dx, dy, step_cost) in neighbors.iter() {
            let nx = cell.0 as i64 + dx;
            let ny = cell.1 as i64 + dy;
            if nx < 0 || ny < 0 || nx as usize >= w || ny as usize >= h { continue; }
            let ncell = (nx as usize, ny as usize);
            if map.is_occupied(ncell.0, ncell.1) { continue; }

            let tentative_g = current_g.saturating_add(*step_cost);
            let neighbor_g = *g_score.get(&idx(ncell)).unwrap_or(&u32::MAX);
            if tentative_g < neighbor_g {
                g_score.insert(idx(ncell), tentative_g);
                came_from.insert(idx(ncell), cell);
                open.push(Node {
                    cost: tentative_g + heuristic(ncell, goal),
                    cell: ncell,
                });
            }
        }
    }
    None
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    zenoh::init_log_from_env_or("error");

    let map_path = args
        .map
        .unwrap_or_else(|| format!("{}/tb3_map.yaml", std::env::var("HOME").unwrap()));
    let map = Arc::new(StaticMap::load(&PathBuf::from(map_path)).expect("failed to load map"));

    let format = hiroz_protocol::KeyExprFormat::RmwZenoh;
    let ctx = if let Some(e) = args.endpoint {
        ZContextBuilder::default().with_mode(args.mode).with_connect_endpoints([e]).keyexpr_format(format).build()?
    } else {
        ZContextBuilder::default().with_mode(args.mode).keyexpr_format(format).build()?
    };

    let node = ctx.create_node("planner").build()?;
    println!("Node created: /planner");

    let goal_sub = node.create_sub::<PoseStamped>("/goal_pose").build()?;
    let odom_sub = node.create_sub::<Odometry>("/odom").build()?;
    let path_pub = node.create_pub::<Path>("/planned_path").build()?;

    let current_pos = Arc::new(Mutex::new((0.0f32, 0.0f32)));
    let current_pos_c = current_pos.clone();

    tokio::spawn(async move {
        while let Ok(msg) = odom_sub.async_recv().await {
            let mut pos = current_pos_c.lock().unwrap();
            *pos = (msg.pose.pose.position.x as f32, msg.pose.pose.position.y as f32);
        }
    });

    println!("Planner node ready. Waiting for /goal_pose...");

    while let Ok(goal_msg) = goal_sub.async_recv().await {
        let goal_x = goal_msg.pose.position.x as f32;
        let goal_y = goal_msg.pose.position.y as f32;
        let (start_x, start_y) = *current_pos.lock().unwrap();

        let start_cell = map.world_to_cell(start_x, start_y);
        let goal_cell = map.world_to_cell(goal_x, goal_y);

        if let (Some(s), Some(g)) = (start_cell, goal_cell) {
            let s_snap = nearest_passable(&map, s, 5);
            let g_snap = nearest_passable(&map, g, 5);

            if let (Some(s2), Some(g2)) = (s_snap, g_snap) {
                if let Some(path_pts) = astar(&map, s2, g2) {
                    println!("Path generated ({} waypoints). Publishing to /planned_path...", path_pts.len());
                    
                    let mut path_msg = Path::default();
                    for &(wx, wy) in path_pts.iter() {
                        let mut pose_stamped = PoseStamped::default();
                        pose_stamped.pose.position.x = wx as f64;
                        pose_stamped.pose.position.y = wy as f64;
                        pose_stamped.pose.orientation.w = 1.0;
                        path_msg.poses.push(pose_stamped);
                    }
                    let _ = path_pub.async_publish(&path_msg).await;
                } else {
                    println!("No valid path found.");
                }
            }
        }
    }

    Ok(())
}
