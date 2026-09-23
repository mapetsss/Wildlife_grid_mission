from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    color_image_topic = LaunchConfiguration("color_image_topic")
    color_info_topic = LaunchConfiguration("color_info_topic")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    depth_info_topic = LaunchConfiguration("depth_info_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("mono_camera_capture"), "config", "d435_rgbd.yaml"]
                ),
                description="Path to the Intel RealSense D435 parameter file.",
            ),
            DeclareLaunchArgument(
                "color_image_topic",
                default_value="/camera/color/image_raw",
                description="D435 color image topic used by YOLO.",
            ),
            DeclareLaunchArgument(
                "color_info_topic",
                default_value="/camera/color/camera_info",
                description="D435 factory-calibrated color CameraInfo topic.",
            ),
            DeclareLaunchArgument(
                "depth_image_topic",
                default_value="/camera/aligned_depth_to_color/image_raw",
                description="Depth image aligned to the color optical frame.",
            ),
            DeclareLaunchArgument(
                "depth_info_topic",
                default_value="/camera/aligned_depth_to_color/camera_info",
                description="CameraInfo matching the aligned depth image.",
            ),
            Node(
                package="mono_camera_capture",
                executable="mono_camera_node",
                name="d435_camera_node",
                output="screen",
                parameters=[params_file],
                remappings=[
                    ("color/image_raw", color_image_topic),
                    ("color/camera_info", color_info_topic),
                    ("aligned_depth_to_color/image_raw", depth_image_topic),
                    ("aligned_depth_to_color/camera_info", depth_info_topic),
                ],
            ),
        ]
    )
