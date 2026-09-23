from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("mono_camera_capture"), "config", "d435_rgb_only.yaml"]
                ),
            ),
            Node(
                package="mono_camera_capture",
                executable="mono_camera_node",
                name="d435_camera_node",
                output="screen",
                parameters=[params_file],
                remappings=[
                    ("color/image_raw", "/camera/color/image_raw"),
                    ("color/camera_info", "/camera/color/camera_info"),
                ],
            ),
        ]
    )
