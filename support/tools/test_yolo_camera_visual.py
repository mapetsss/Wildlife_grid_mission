#!/usr/bin/env python3
"""Visual test helper for robocup YOLO detection on a monocular camera.

This script does not run inference itself. It subscribes to the camera image,
periodically calls the ros2_yolos_cpp detection service, subscribes to the
published Detection2DArray, and draws boxes on the live camera frame.
"""

import argparse
import os
import select
import sys
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

try:
    from ros2_yolos_cpp.srv import DetectImage
except ImportError as exc:
    print(
        "无法导入 ros2_yolos_cpp.srv.DetectImage，请先 source install/setup.bash",
        file=sys.stderr,
    )
    raise exc


class YoloCameraVisualTester(Node):
    def __init__(self, args):
        super().__init__("yolo_camera_visual_tester")
        self.args = args
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.latest_image_msg = None
        self.latest_frame = None
        self.latest_detections = Detection2DArray()
        self.last_service_call_time = 0.0
        self.last_detection_time = 0.0
        self.frame_count = 0
        self.saved_count = 0
        self.window_available = not args.no_window

        self.image_sub = self.create_subscription(
            Image,
            args.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            args.detections_topic,
            self.detections_callback,
            10,
        )
        self.detect_client = self.create_client(DetectImage, args.service)
        self.timer = self.create_timer(1.0 / args.trigger_hz, self.timer_callback)

        if args.save_dir:
            os.makedirs(args.save_dir, exist_ok=True)

        self.get_logger().info(f"订阅图像: {args.image_topic}")
        self.get_logger().info(f"订阅检测结果: {args.detections_topic}")
        self.get_logger().info(f"调用检测服务: {args.service}")
        self.get_logger().info("模型输入尺寸按 320x320 记录；实际预处理由 ros2_yolos_cpp/yolos-cpp 完成")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"图像转换失败: {exc}")
            return

        with self.lock:
            self.latest_image_msg = msg
            self.latest_frame = frame.copy()

    def detections_callback(self, msg):
        with self.lock:
            self.latest_detections = msg
            self.last_detection_time = time.time()

    def timer_callback(self):
        now = time.time()
        with self.lock:
            image_msg = self.latest_image_msg

        if image_msg is None:
            self.get_logger().warn("还没有收到相机图像")
            return

        if not self.detect_client.service_is_ready():
            self.get_logger().warn(f"检测服务未就绪: {self.args.service}")
            return

        if now - self.last_service_call_time < 1.0 / self.args.trigger_hz:
            return

        request = DetectImage.Request()
        request.image = image_msg
        self.detect_client.call_async(request)
        self.last_service_call_time = now

    def make_visual_frame(self):
        with self.lock:
            if self.latest_frame is None:
                return None, 0
            frame = self.latest_frame.copy()
            detections = list(self.latest_detections.detections)
            age = time.time() - self.last_detection_time if self.last_detection_time > 0 else None

        valid_count = 0
        for det in detections:
            label, score = self.get_detection_label_score(det)
            if self.args.class_name and label and label != self.args.class_name:
                continue
            if score is not None and score < self.args.min_score:
                continue

            bbox = det.bbox
            cx = float(bbox.center.position.x)
            cy = float(bbox.center.position.y)
            w = float(bbox.size_x)
            h = float(bbox.size_y)
            x1 = int(round(cx - w / 2.0))
            y1 = int(round(cy - h / 2.0))
            x2 = int(round(cx + w / 2.0))
            y2 = int(round(cy + h / 2.0))

            x1 = max(0, min(frame.shape[1] - 1, x1))
            y1 = max(0, min(frame.shape[0] - 1, y1))
            x2 = max(0, min(frame.shape[1] - 1, x2))
            y2 = max(0, min(frame.shape[0] - 1, y2))

            text = label or self.args.class_name or "robocup"
            if score is not None:
                text = f"{text} {score:.2f}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 0), 2)
            self.draw_label(frame, text, x1, y1)
            valid_count += 1

        status = f"input=320x320 detections={valid_count} trigger={self.args.trigger_hz:.1f}Hz"
        if age is not None:
            status += f" age={age:.1f}s"
        cv2.putText(frame, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)
        return frame, valid_count

    @staticmethod
    def get_detection_label_score(det):
        if not det.results:
            return "", None

        hyp = det.results[0].hypothesis
        label = str(getattr(hyp, "class_id", ""))
        score = getattr(hyp, "score", None)
        return label, score

    @staticmethod
    def draw_label(frame, text, x, y):
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.55
        thickness = 1
        (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
        y_text = max(th + 6, y)
        cv2.rectangle(frame, (x, y_text - th - 6), (x + tw + 8, y_text + baseline), (0, 120, 0), -1)
        cv2.putText(frame, text, (x + 4, y_text - 4), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)

    def spin_visual(self):
        self.get_logger().info("窗口模式按 q 或 Esc 退出；终端模式输入 q 后回车退出")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.should_quit_from_terminal():
                break

            frame, count = self.make_visual_frame()
            if frame is None:
                continue

            self.frame_count += 1
            if self.args.save_dir and self.should_save(count):
                filename = os.path.join(self.args.save_dir, f"robocup_yolo_{self.saved_count:04d}.jpg")
                if cv2.imwrite(filename, frame):
                    self.saved_count += 1
                    self.get_logger().info(f"已保存: {filename}")

            if self.window_available:
                try:
                    cv2.imshow(self.args.window_name, frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
                except cv2.error as exc:
                    self.window_available = False
                    self.get_logger().warn(
                        "OpenCV 图形窗口不可用，已切换为无窗口模式。"
                        "请使用 --save-dir 保存带框图片查看效果。"
                    )
                    self.get_logger().warn(f"OpenCV 窗口错误: {exc}")

            if self.args.max_frames > 0 and self.frame_count >= self.args.max_frames:
                break

        cv2.destroyAllWindows()

    @staticmethod
    def should_quit_from_terminal():
        if not sys.stdin.isatty():
            return False
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return False
        text = sys.stdin.readline().strip().lower()
        return text in ("q", "quit", "exit")

    def should_save(self, detection_count):
        if self.args.save_only_detections and detection_count <= 0:
            return False
        if self.args.max_saved > 0 and self.saved_count >= self.args.max_saved:
            return False
        return self.frame_count % self.args.save_every == 0


def parse_args():
    parser = argparse.ArgumentParser(description="测试单目摄像头上的 robocup YOLO 识别可视化效果")
    parser.add_argument(
        "--image-topic",
        default="/camera/color/image_raw",
        help="RealSense D435 彩色图像话题",
    )
    parser.add_argument("--detections-topic", default="/yolos_detector/detections", help="检测结果话题")
    parser.add_argument("--service", default="/yolos_detector/detect", help="检测服务名")
    parser.add_argument("--class-name", default="robocup", help="只显示这个类别；留空则显示全部")
    parser.add_argument("--min-score", type=float, default=0.0, help="可视化最低置信度")
    parser.add_argument("--trigger-hz", type=float, default=2.0, help="服务触发频率，单位 Hz")
    parser.add_argument("--save-dir", default="", help="保存带框图片的目录")
    parser.add_argument("--save-every", type=int, default=10, help="每 N 帧保存一张")
    parser.add_argument("--save-only-detections", action="store_true", help="只保存检测到 robocup 的帧")
    parser.add_argument("--max-saved", type=int, default=0, help="最多保存多少张，0 表示不限")
    parser.add_argument("--max-frames", type=int, default=0, help="最多处理多少帧，0 表示不限")
    parser.add_argument("--no-window", action="store_true", help="不显示 OpenCV 窗口，只保存或打印日志")
    parser.add_argument("--window-name", default="robocup_yolo_test", help="OpenCV 窗口名")
    args = parser.parse_args()

    if args.trigger_hz <= 0:
        parser.error("--trigger-hz 必须大于 0")
    if args.save_every <= 0:
        parser.error("--save-every 必须大于 0")
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = YoloCameraVisualTester(args)
    try:
        node.spin_visual()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
