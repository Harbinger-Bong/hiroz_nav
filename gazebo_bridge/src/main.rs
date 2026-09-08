use clap::Parser;
use hiroz::{
    Builder, Result,
    context::ZContextBuilder,
};
use hiroz_msgs::geometry_msgs::Twist;
use hiroz_msgs::nav_msgs::Odometry;
use hiroz_msgs::sensor_msgs::LaserScan;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

#[derive(Debug, Parser)]
struct Args {
    #[arg(short, long, default_value = "peer")]
    mode: String,
    #[arg(short, long)]
    endpoint: Option<String>,
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

    println!("=== Gazebo Bridge (Hiroz) ===");

    let node = ctx.create_node("gazebo_bridge").build()?;
    println!("Node created: /gazebo_bridge");

    let odom_sub = node.create_sub::<Odometry>("/odom").build()?;
    println!("Subscribed to /odom");

    let scan_sub = node.create_sub::<LaserScan>("/scan").build()?;
    println!("Subscribed to /scan");

    let cmd_vel_pub = node.create_pub::<Twist>("/cmd_vel").build()?;
    println!("Created publisher on /cmd_vel");

    println!();
    println!("Bridge is live. Gazebo (ROS 2) <-> Zenoh <-> Hiroz");
    println!();

    let odom_count = Arc::new(AtomicUsize::new(0));
    let scan_count = Arc::new(AtomicUsize::new(0));

    let odom_count_c = odom_count.clone();
    let odom_task = tokio::spawn(async move {
        while let Ok(msg) = odom_sub.async_recv().await {
            let count = odom_count_c.fetch_add(1, Ordering::Relaxed);
            if true {
                println!(
                    "[ODOM #{count}] pos=({:.4}, {:.4}) vx={:.3}",
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.twist.twist.linear.x
                );
            }
        }
    });

    let scan_count_c = scan_count.clone();
    let scan_task = tokio::spawn(async move {
        while let Ok(msg) = scan_sub.async_recv().await {
            let count = scan_count_c.fetch_add(1, Ordering::Relaxed);
            if true {
                let min_range = msg
                    .ranges
                    .iter()
                    .filter(|r| r.is_finite())
                    .cloned()
                    .fold(f32::INFINITY, f32::min);
                println!(
                    "[SCAN #{count}] n={} min_dist={:.2}m",
                    msg.ranges.len(),
                    min_range
                );
            }
        }
    });

    let _ = cmd_vel_pub.async_publish(&Twist::default()).await;

    let _ = tokio::join!(odom_task, scan_task);

    Ok(())
}
