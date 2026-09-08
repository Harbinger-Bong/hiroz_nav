use clap::Parser;
use hiroz::{Builder, Result, context::ZContextBuilder};
use hiroz_msgs::geometry_msgs::{TwistStamped, Twist};
use hiroz_msgs::nav_msgs::{Odometry, Path};
use std::sync::{Arc, Mutex};
use tokio::time::{interval, Duration};

#[derive(Debug, Parser)]
struct Args {
    #[arg(short, long, default_value = "peer")]
    mode: String,
    #[arg(short, long)]
    endpoint: Option<String>,
}

#[derive(Default, Clone)]
struct BotState {
    x: f32,
    y: f32,
    yaw: f32,
}

fn normalize_angle(mut angle: f32) -> f32 {
    while angle > std::f32::consts::PI { angle -= 2.0 * std::f32::consts::PI; }
    while angle < -std::f32::consts::PI { angle += 2.0 * std::f32::consts::PI; }
    angle
}

fn yaw_from_quat(x: f32, y: f32, z: f32, w: f32) -> f32 {
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    siny_cosp.atan2(cosy_cosp)
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    zenoh::init_log_from_env_or("error");

    let format = hiroz_protocol::KeyExprFormat::RmwZenoh;
    let ctx = if let Some(e) = args.endpoint {
        ZContextBuilder::default().with_mode(args.mode).with_connect_endpoints([e]).keyexpr_format(format).build()?
    } else {
        ZContextBuilder::default().with_mode(args.mode).keyexpr_format(format).build()?
    };

    let node = ctx.create_node("controller").build()?;
    println!("Node created: /controller");

    let odom_sub = node.create_sub::<Odometry>("/odom").build()?;
    let path_sub = node.create_sub::<Path>("/planned_path").build()?;
    let cmd_vel_pub = node.create_pub::<TwistStamped>("/cmd_vel").build()?;

    let bot_state = Arc::new(Mutex::new(BotState::default()));
    let current_path = Arc::new(Mutex::new(Vec::<(f32, f32)>::new()));

    // 1. Odometry Tracker
    let bot_state_odom = bot_state.clone();
    tokio::spawn(async move {
        while let Ok(msg) = odom_sub.async_recv().await {
            let q = msg.pose.pose.orientation;
            let yaw = yaw_from_quat(q.x as f32, q.y as f32, q.z as f32, q.w as f32);
            let mut state = bot_state_odom.lock().unwrap();
            state.x = msg.pose.pose.position.x as f32;
            state.y = msg.pose.pose.position.y as f32;
            state.yaw = yaw;
        }
    });

    // 2. Path Subscriber
    let path_sub_c = current_path.clone();
    tokio::spawn(async move {
        while let Ok(path_msg) = path_sub.async_recv().await {
            let mut waypoints: Vec<(f32, f32)> = path_msg
                .poses
                .iter()
                .map(|p| (p.pose.position.x as f32, p.pose.position.y as f32))
                .collect();

            if waypoints.len() > 1 {
                waypoints.remove(0);
            }
            println!("Received new path with {} target points", waypoints.len());
            *path_sub_c.lock().unwrap() = waypoints;
        }
    });

    // 3. Control Loop (10 Hz)
    println!("Controller node ready. Awaiting path...");
    let mut ticker = interval(Duration::from_millis(100));
    loop {
        ticker.tick().await;
        let mut twist_stamped = TwistStamped::default();
        // Optionally set header frame_id if needed by your message bindings
        // twist_stamped.header.frame_id = "base_link".to_string();

        let mut path = current_path.lock().unwrap();
        if !path.is_empty() {
            let state = bot_state.lock().unwrap();
            let target = path[0];

            let dx = target.0 - state.x;
            let dy = target.1 - state.y;
            let dist = (dx * dx + dy * dy).sqrt();

            if dist < 0.15 {
                path.remove(0);
            } else {
                let target_yaw = dy.atan2(dx);
                let yaw_err = normalize_angle(target_yaw - state.yaw);

                if yaw_err.abs() > 0.2 {
                    twist_stamped.twist.angular.z = (1.5 * yaw_err as f64).clamp(-1.0, 1.0);
                } else {
                    twist_stamped.twist.linear.x = (0.5 * dist as f64).clamp(0.0, 0.25);
                    twist_stamped.twist.angular.z = (1.0 * yaw_err as f64).clamp(-0.5, 0.5);
                }
            }
        }
        let _ = cmd_vel_pub.async_publish(&twist_stamped).await;
    }
}
