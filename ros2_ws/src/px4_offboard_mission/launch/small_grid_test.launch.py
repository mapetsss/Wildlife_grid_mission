from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    auto_arm_enabled = LaunchConfiguration("auto_arm_enabled")
    auto_start = LaunchConfiguration("auto_start")
    radar_failsafe_enabled = LaunchConfiguration("radar_failsafe_enabled")

    args = [
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("px4_offboard_mission"),
                "config",
                "small_grid_test.yaml",
            ]),
        ),
        DeclareLaunchArgument("auto_arm_enabled", default_value="false"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("radar_failsafe_enabled", default_value="true"),
    ]

    node = Node(
        package="px4_offboard_mission",
        executable="small_grid_test_node",
        name="small_grid_test_node",
        output="screen",
        parameters=[
            config_file,
            {
                "auto_arm_enabled": auto_arm_enabled,
                "auto_start": auto_start,
                "radar_failsafe_enabled": radar_failsafe_enabled,
            },
        ],
    )

    return LaunchDescription(args + [node])
