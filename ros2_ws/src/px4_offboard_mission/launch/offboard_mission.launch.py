from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("takeoff_height_m", default_value="1.4"),
        DeclareLaunchArgument("takeoff_climb_rate_mps", default_value="0.35"),
        DeclareLaunchArgument("hover_time_s", default_value="3.0"),
        DeclareLaunchArgument("left_distance_m", default_value="1.0"),
        DeclareLaunchArgument("position_tolerance_m", default_value="0.15"),
        DeclareLaunchArgument("setpoint_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("prestream_time_s", default_value="3.0"),
        DeclareLaunchArgument("pose_timeout_s", default_value="2.5"),
        DeclareLaunchArgument("takeoff_timeout_s", default_value="25.0"),
        DeclareLaunchArgument("move_timeout_s", default_value="25.0"),
        DeclareLaunchArgument("land_mode", default_value="AUTO.LAND"),
    ]

    node = Node(
        package="px4_offboard_mission",
        executable="offboard_mission_node",
        name="px4_offboard_mission",
        output="screen",
        parameters=[{
            "takeoff_height_m": LaunchConfiguration("takeoff_height_m"),
            "takeoff_climb_rate_mps": LaunchConfiguration("takeoff_climb_rate_mps"),
            "hover_time_s": LaunchConfiguration("hover_time_s"),
            "left_distance_m": LaunchConfiguration("left_distance_m"),
            "position_tolerance_m": LaunchConfiguration("position_tolerance_m"),
            "setpoint_rate_hz": LaunchConfiguration("setpoint_rate_hz"),
            "prestream_time_s": LaunchConfiguration("prestream_time_s"),
            "pose_timeout_s": LaunchConfiguration("pose_timeout_s"),
            "takeoff_timeout_s": LaunchConfiguration("takeoff_timeout_s"),
            "move_timeout_s": LaunchConfiguration("move_timeout_s"),
            "land_mode": LaunchConfiguration("land_mode"),
        }],
    )

    return LaunchDescription(args + [node])
