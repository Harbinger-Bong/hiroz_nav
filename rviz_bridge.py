#!/usr/bin/env python3
"""
rviz_bridge.py  —  ROS 2 ↔ Zenoh/Hiroz bridge for RViz interaction.

What it does
────────────
  RViz  →  ROS 2 /goal_pose           (PoseStamped, from "2D Goal Pose" tool)
         →  re-published on ROS 2      (passthrough; Zenoh rmw picks it up)

  Zenoh  → ROS 2 /odom                (so RViz robot model moves)
  Zenoh  → ROS 2 /plan                (so RViz draws the path)
  Zenoh  → ROS 2 /scan                (optional laser overlay in RViz)

  TF     → map → odom  (static, identity — adjust if your map origin differs)

Usage
─────
  python3 rviz_bridge.py          # in a sourced ROS 2 Jazzy shell
  python3 rviz_bridge.py --help

The node uses rmw_zenoh_cpp as the RMW, so ROS 2 topics are automatically
tunnelled through Zenoh — no explicit re-publishing needed for /goal_pose.
The bridge's main job is therefore TF + RViz config generation.
"""

import subprocess
import sys
import os
import argparse
import time

# ──────────────────────────────────────────────────────────────────
# Argument parsing (before rclpy import so --help works without ROS)
# ──────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="RViz ↔ Hiroz bridge")
parser.add_argument("--map-yaml", default=os.path.expanduser("~/tb3_map.yaml"),
                    help="Path to the map YAML (for map_server)")
parser.add_argument("--map-frame",   default="map")
parser.add_argument("--odom-frame",  default="odom")
parser.add_argument("--robot-frame", default="base_footprint")
parser.add_argument("--no-map-server", action="store_true",
                    help="Skip launching map_server (if already running)")
args = parser.parse_args()

# ──────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import math

# ──────────────────────────────────────────────────────────────────
LATCHING_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

# ──────────────────────────────────────────────────────────────────

class RvizBridge(Node):
    """
    Thin ROS 2 node.  Because rmw_zenoh_cpp is the RMW, all ROS 2 pub/sub
    here are automatically available to the Hiroz nodes — no manual
    serialise/deserialise needed.

    Responsibilities:
      1. Broadcast  map → odom  static TF  (identity, z-planar robot)
      2. Broadcast  odom → base_footprint  dynamic TF from /odom
      3. Echo /goal_pose back so RViz sees its own click acknowledged
      4. Subscribe /plan and republish (path already on ROS 2 via zenoh rmw)
    """

    def __init__(self):
        super().__init__("rviz_bridge")

        # ── TF broadcasters ──────────────────────────────────────
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_map_to_odom()

        self.dynamic_tf_broadcaster = TransformBroadcaster(self)

        # ── Subscriptions ────────────────────────────────────────
        self.create_subscription(Odometry, "/odom",
                                 self._odom_cb, 10)
        self.create_subscription(PoseStamped, "/goal_pose",
                                 self._goal_cb, 10)
        self.create_subscription(Path, "/planned_path",
                                 self._path_cb, 10)

        # ── Publishers ───────────────────────────────────────────
        # /plan  — standard RViz path display topic
        self.plan_pub = self.create_publisher(Path, "/plan", LATCHING_QOS)

        self.get_logger().info("rviz_bridge ready")
        self.get_logger().info(f"  map frame   : {args.map_frame}")
        self.get_logger().info(f"  odom frame  : {args.odom_frame}")
        self.get_logger().info(f"  robot frame : {args.robot_frame}")
        self.get_logger().info("  Click '2D Goal Pose' in RViz to send goals")

    # ── Static TF: map → odom ────────────────────────────────────
    def _publish_static_map_to_odom(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = args.map_frame
        t.child_frame_id  = args.odom_frame
        # Identity transform — assumes robot starts at map origin.
        # If your map origin is non-zero, translate here:
        #   t.transform.translation.x = -origin_x
        #   t.transform.translation.y = -origin_y
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info("Static TF published: map → odom (identity)")

    # ── Dynamic TF: odom → base_footprint ───────────────────────
    def _odom_cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = args.odom_frame
        t.child_frame_id  = args.robot_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation      = msg.pose.pose.orientation
        self.dynamic_tf_broadcaster.sendTransform(t)

    # ── Goal echo (informational) ────────────────────────────────
    def _goal_cb(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(f"Goal received from RViz → ({x:.3f}, {y:.3f})")

    # ── Path relay: /planned_path → /plan ───────────────────────
    def _path_cb(self, msg: Path):
        msg.header.frame_id = args.map_frame   # ensure correct frame
        self.plan_pub.publish(msg)
        self.get_logger().info(f"Path relayed: {len(msg.poses)} waypoints → /plan")


# ──────────────────────────────────────────────────────────────────
# map_server launcher (optional helper)
# ──────────────────────────────────────────────────────────────────

def launch_map_server(yaml_path: str):
    """Spawn map_server and lifecycle_manager as subprocess."""
    if not os.path.exists(yaml_path):
        print(f"[bridge] WARNING: map YAML not found: {yaml_path}")
        print("[bridge] Skipping map_server launch — load map manually in RViz.")
        return None

    print(f"[bridge] Launching map_server with {yaml_path} ...")
    cmd = [
        "ros2", "run", "nav2_map_server", "map_server",
        "--ros-args", "-p", f"yaml_filename:={yaml_path}",
                     "-p", "use_sim_time:=false",
    ]
    proc = subprocess.Popen(cmd)

    # Give map_server a moment, then activate it via lifecycle
    time.sleep(1.5)
    subprocess.run([
        "ros2", "lifecycle", "set", "/map_server", "configure"
    ], capture_output=True)
    time.sleep(0.5)
    subprocess.run([
        "ros2", "lifecycle", "set", "/map_server", "activate"
    ], capture_output=True)
    print("[bridge] map_server active.")
    return proc


# ──────────────────────────────────────────────────────────────────
# RViz config generator
# ──────────────────────────────────────────────────────────────────

RVIZ_CONFIG = """\
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Tools
    Name: Tools
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Name: Map
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
    - Alpha: 1.0
      Axes Length: 0.3
      Class: rviz_default_plugins/Odometry
      Name: Odometry
      Topic:
        Depth: 10
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /odom
    - Class: rviz_default_plugins/LaserScan
      Color: 255; 50; 50
      Name: LaserScan
      Size (m): 0.04
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
    - Class: rviz_default_plugins/Path
      Color: 0; 200; 50
      Name: PlannedPath
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /plan
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
  Global Options:
    Fixed Frame: map
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
  Value: Orbit (rviz)
"""

# ──────────────────────────────────────────────────────────────────

def main():
    # Write RViz config
    rviz_path = os.path.expanduser("~/tb3_hiroz.rviz")
    with open(rviz_path, "w") as f:
        f.write(RVIZ_CONFIG)
    print(f"[bridge] RViz config written → {rviz_path}")

    # Optionally launch map_server
    map_proc = None
    if not args.no_map_server:
        map_proc = launch_map_server(args.map_yaml)

    # Start the bridge node
    rclpy.init()
    node = RvizBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if map_proc:
            map_proc.terminate()


if __name__ == "__main__":
    main()
