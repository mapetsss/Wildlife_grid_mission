#!/usr/bin/env python3
"""Record MAVROS local pose and plot position/yaw after a manual test."""

import argparse
import csv
import math
import os
import signal
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


CSV_FIELDS = [
    "time_s",
    "ros_stamp_s",
    "x_m",
    "y_m",
    "z_m",
    "qx",
    "qy",
    "qz",
    "qw",
    "yaw_rad",
    "yaw_deg",
]


def yaw_from_quaternion(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def stamp_to_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def unwrap_degrees(values):
    if not values:
        return []
    output = [values[0]]
    offset = 0.0
    previous = values[0]
    for value in values[1:]:
        delta = value - previous
        if delta > 180.0:
            offset -= 360.0
        elif delta < -180.0:
            offset += 360.0
        output.append(value + offset)
        previous = value
    return output


def make_qos_profile(reliable):
    return QoSProfile(
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def read_csv(csv_path):
    rows = []
    with open(csv_path, "r", newline="") as csv_file:
        for row in csv.DictReader(csv_file):
            rows.append({key: float(row[key]) for key in CSV_FIELDS})
    return rows


def plot_rows(rows, png_path, title):
    if not rows:
        print("No pose samples recorded; skip plot.")
        return

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed; CSV was saved but plot was skipped.")
        return

    time_s = [row["time_s"] for row in rows]
    x_m = [row["x_m"] for row in rows]
    y_m = [row["y_m"] for row in rows]
    z_m = [row["z_m"] for row in rows]
    yaw_deg = [row["yaw_deg"] for row in rows]
    yaw_unwrapped = unwrap_degrees(yaw_deg)

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(title)

    path_ax = axes[0][0]
    path_ax.plot(x_m, y_m, linewidth=1.8, label="XY path")
    path_ax.scatter(x_m[0], y_m[0], marker="o", s=70, label="start")
    path_ax.scatter(x_m[-1], y_m[-1], marker="x", s=90, label="end")

    stride = max(1, len(rows) // 25)
    arrow_len = max(0.15, min(0.6, max(max(x_m) - min(x_m), max(y_m) - min(y_m), 0.3) * 0.12))
    for index in range(0, len(rows), stride):
        yaw = math.radians(yaw_deg[index])
        path_ax.arrow(
            x_m[index],
            y_m[index],
            arrow_len * math.cos(yaw),
            arrow_len * math.sin(yaw),
            width=0.006,
            head_width=0.05,
            length_includes_head=True,
            alpha=0.65,
        )

    path_ax.set_xlabel("x / m")
    path_ax.set_ylabel("y / m")
    path_ax.set_title("Local XY path with yaw arrows")
    path_ax.axis("equal")
    path_ax.grid(True)
    path_ax.legend()

    axes[0][1].plot(time_s, z_m, linewidth=1.5)
    axes[0][1].set_xlabel("time / s")
    axes[0][1].set_ylabel("z / m")
    axes[0][1].set_title("Altitude in MAVROS local ENU")
    axes[0][1].grid(True)

    axes[1][0].plot(time_s, yaw_deg, linewidth=1.2, label="wrapped")
    axes[1][0].plot(time_s, yaw_unwrapped, linewidth=1.2, label="unwrapped")
    axes[1][0].set_xlabel("time / s")
    axes[1][0].set_ylabel("yaw / deg")
    axes[1][0].set_title("Vehicle yaw")
    axes[1][0].grid(True)
    axes[1][0].legend()

    axes[1][1].plot(time_s, x_m, linewidth=1.2, label="x")
    axes[1][1].plot(time_s, y_m, linewidth=1.2, label="y")
    axes[1][1].plot(time_s, z_m, linewidth=1.2, label="z")
    axes[1][1].set_xlabel("time / s")
    axes[1][1].set_ylabel("position / m")
    axes[1][1].set_title("Position over time")
    axes[1][1].grid(True)
    axes[1][1].legend()

    fig.tight_layout()
    fig.savefig(png_path, dpi=160)
    plt.close(fig)
    print(f"Saved plot: {png_path}")


class PoseRecorder(Node):
    def __init__(self, args):
        super().__init__("mavros_pose_recorder")
        self.args = args
        self.rows = []
        self.start_time = None
        self.last_sample_time = None
        self.sample_period = 0.0 if args.max_rate <= 0.0 else 1.0 / args.max_rate

        self.create_subscription(
            PoseStamped,
            args.topic,
            self.pose_callback,
            make_qos_profile(args.reliable),
        )
        self.get_logger().info(f"Recording {args.topic}; press Ctrl+C to stop.")

    def pose_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if self.start_time is None:
            self.start_time = now
        if self.last_sample_time is not None and now - self.last_sample_time < self.sample_period:
            return
        self.last_sample_time = now

        p = msg.pose.position
        q = msg.pose.orientation
        yaw_rad = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.rows.append({
            "time_s": now - self.start_time,
            "ros_stamp_s": stamp_to_seconds(msg.header.stamp),
            "x_m": p.x,
            "y_m": p.y,
            "z_m": p.z,
            "qx": q.x,
            "qy": q.y,
            "qz": q.z,
            "qw": q.w,
            "yaw_rad": yaw_rad,
            "yaw_deg": math.degrees(yaw_rad),
        })

    def save_csv(self, csv_path):
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with open(csv_path, "w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(self.rows)
        self.get_logger().info(f"Saved {len(self.rows)} samples: {csv_path}")


def make_output_paths(args):
    output_dir = Path(os.path.expanduser(args.output_dir))
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = args.name or datetime.now().strftime("mavros_pose_%Y%m%d_%H%M%S")
    return output_dir / f"{stem}.csv", output_dir / f"{stem}.png"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record /mavros/local_position/pose to CSV and generate a path/yaw plot."
    )
    parser.add_argument("--topic", default="/mavros/local_position/pose")
    parser.add_argument("--output-dir", default="~/detect_ws/flight_logs")
    parser.add_argument("--name", default="", help="Output file stem; default uses timestamp.")
    parser.add_argument("--max-rate", type=float, default=30.0, help="Maximum saved samples per second; <=0 saves every message.")
    parser.add_argument("--reliable", action="store_true", help="Use reliable QoS instead of best effort.")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--from-csv", default="", help="Only generate a plot from an existing CSV.")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.from_csv:
        csv_path = Path(os.path.expanduser(args.from_csv))
        png_path = csv_path.with_suffix(".png")
        rows = read_csv(csv_path)
        plot_rows(rows, png_path, csv_path.stem)
        return

    csv_path, png_path = make_output_paths(args)

    rclpy.init()
    recorder = PoseRecorder(args)

    stop_requested = {"value": False}

    def request_stop(_signum, _frame):
        stop_requested["value"] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        while rclpy.ok() and not stop_requested["value"]:
            rclpy.spin_once(recorder, timeout_sec=0.2)
    finally:
        recorder.save_csv(csv_path)
        rows = list(recorder.rows)
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not args.no_plot:
        plot_rows(rows, png_path, csv_path.stem)


if __name__ == "__main__":
    main()
