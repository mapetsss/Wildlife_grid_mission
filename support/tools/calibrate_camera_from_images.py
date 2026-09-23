#!/usr/bin/env python3
"""使用保存好的棋盘格图片离线计算单目相机内参。"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np
import yaml

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def make_camera_info_yaml(image_width, image_height, camera_matrix, dist_coeffs):
    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])
    d = [float(x) for x in dist_coeffs.reshape(-1)]

    return {
        "image_width": int(image_width),
        "image_height": int(image_height),
        "camera_name": "realsense_d435_color",
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [
                fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0.0, 1.0,
            ],
        },
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1,
            "cols": len(d),
            "data": d,
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
            ],
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": [
                fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0,
            ],
        },
    }


def main():
    parser = argparse.ArgumentParser(description="离线棋盘格相机标定")
    parser.add_argument(
        "--images-dir",
        default=os.path.join(PROJECT_ROOT, "calibration", "images"),
        help="棋盘格图片目录",
    )
    parser.add_argument("--cols", type=int, default=9, help="棋盘格内角点列数")
    parser.add_argument("--rows", type=int, default=6, help="棋盘格内角点行数")
    parser.add_argument("--square", type=float, default=0.03, help="方块边长，单位米")
    parser.add_argument(
        "--output",
        default=os.path.join(PROJECT_ROOT, "calibration", "camera_calibration.yaml"),
        help="输出 YAML 文件路径",
    )
    args = parser.parse_args()

    pattern_size = (args.cols, args.rows)
    image_paths = sorted(
        glob.glob(os.path.join(args.images_dir, "*.png")) +
        glob.glob(os.path.join(args.images_dir, "*.jpg")) +
        glob.glob(os.path.join(args.images_dir, "*.jpeg"))
    )

    if not image_paths:
        print(f"没有找到标定图片: {args.images_dir}", file=sys.stderr)
        return 1

    object_points_template = np.zeros((args.rows * args.cols, 3), np.float32)
    object_points_template[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    object_points_template *= args.square

    object_points = []
    image_points = []
    image_size = None
    accepted = 0

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )

    for path in image_paths:
        image = cv2.imread(path)
        if image is None:
            print(f"跳过无法读取的图片: {path}")
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])
        elif image_size != (gray.shape[1], gray.shape[0]):
            print(f"跳过尺寸不一致的图片: {path}")
            continue

        found, corners = cv2.findChessboardCorners(gray, pattern_size)
        if not found:
            print(f"未识别到 {args.cols}x{args.rows} 内角点: {path}")
            continue

        refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        object_points.append(object_points_template.copy())
        image_points.append(refined)
        accepted += 1
        print(f"使用图片: {path}")

    if accepted < 10:
        print(f"有效图片只有 {accepted} 张，建议至少 20 张以上。", file=sys.stderr)
        return 2

    rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )

    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    result = make_camera_info_yaml(image_size[0], image_size[1], camera_matrix, dist_coeffs)
    result["calibration_rms_error"] = float(rms)
    result["accepted_images"] = int(accepted)
    result["checkerboard"] = {
        "inner_corners_cols": int(args.cols),
        "inner_corners_rows": int(args.rows),
        "square_size_m": float(args.square),
    }

    with open(args.output, "w", encoding="utf-8") as output_file:
        yaml.safe_dump(result, output_file, sort_keys=False, allow_unicode=True)

    print("")
    print(f"标定完成，RMS reprojection error: {rms:.4f}")
    print(f"结果已保存: {args.output}")
    print("")
    print("D435 通常直接使用设备发布的 /camera/color/camera_info。")
    print("以下值仅供核对自定义标定结果：")
    print(f"fx: {camera_matrix[0, 0]:.6f}")
    print(f"fy: {camera_matrix[1, 1]:.6f}")
    print(f"cx: {camera_matrix[0, 2]:.6f}")
    print(f"cy: {camera_matrix[1, 2]:.6f}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
