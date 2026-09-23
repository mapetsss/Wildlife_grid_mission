#!/usr/bin/env python3
"""从 ROS 2 图像话题手动保存棋盘格标定图片。

用法：
  python3 tools/capture_calibration_images.py --topic /camera/color/image_raw

程序运行后：
  - 每次按 Enter 保存当前最新图像
  - 输入 q 后回车退出
"""

import argparse
import os
import sys
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class CalibrationImageCapture:
    def __init__(self, topic):
        self.node = rclpy.create_node("calibration_image_capture")
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_stamp = None
        self.subscription = self.node.create_subscription(
            Image,
            topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.node.get_logger().warn(f"图像转换失败: {exc}")
            return

        with self.lock:
            self.latest_frame = frame.copy()
            self.latest_stamp = msg.header.stamp

    def get_latest_frame(self):
        with self.lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()


def main():
    parser = argparse.ArgumentParser(description="从 ROS 2 图像话题保存相机标定图片")
    parser.add_argument(
        "--topic", default="/camera/color/image_raw", help="D435 彩色图像话题"
    )
    parser.add_argument(
        "--output-dir",
        default=os.path.join(PROJECT_ROOT, "calibration", "images"),
        help="图片保存目录",
    )
    parser.add_argument("--prefix", default="chessboard", help="图片文件名前缀")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    rclpy.init()
    capture = CalibrationImageCapture(args.topic)

    spin_thread = threading.Thread(target=rclpy.spin, args=(capture.node,), daemon=True)
    spin_thread.start()

    print(f"正在订阅图像话题: {args.topic}")
    print(f"图片保存目录: {args.output_dir}")
    print("把棋盘格摆到不同位置和角度后，按 Enter 保存一张；输入 q 后回车退出。")

    saved_count = 0
    try:
        while True:
            user_input = input(f"[{saved_count:03d}] Enter 保存，q 退出 > ").strip().lower()
            if user_input == "q":
                break

            frame = capture.get_latest_frame()
            if frame is None:
                print("还没有收到图像，请确认 /camera/color/image_raw 正在发布。")
                continue

            filename = os.path.join(args.output_dir, f"{args.prefix}_{saved_count:04d}.png")
            if not cv2.imwrite(filename, frame):
                print(f"保存失败: {filename}", file=sys.stderr)
                continue

            print(f"已保存: {filename}")
            saved_count += 1
    finally:
        capture.node.destroy_node()
        rclpy.shutdown()

    print(f"共保存 {saved_count} 张图片。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
