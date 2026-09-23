from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    auto_arm_enabled = LaunchConfiguration("auto_arm_enabled")
    auto_start = LaunchConfiguration("auto_start")
    use_serial_bridge = LaunchConfiguration("use_serial_bridge")
    serial_port = LaunchConfiguration("serial_port")
    serial_baud = LaunchConfiguration("serial_baud")
    serial_start_after_route = LaunchConfiguration("serial_start_after_route")
    serial_report_enabled = LaunchConfiguration("serial_report_enabled")
    send_summary_each_report = LaunchConfiguration("send_summary_each_report")

    args = [
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("px4_offboard_mission"),
                "config",
                "wildlife_mission.yaml",
            ]),
        ),
        DeclareLaunchArgument("auto_arm_enabled", default_value="true"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("use_serial_bridge", default_value="true"),
        DeclareLaunchArgument("serial_port", default_value="/dev/serial0"),
        DeclareLaunchArgument("serial_baud", default_value="115200"),
        DeclareLaunchArgument("serial_start_after_route", default_value="true"),
        DeclareLaunchArgument("serial_report_enabled", default_value="true"),
        DeclareLaunchArgument("send_summary_each_report", default_value="false"),
    ]

    node = Node(
        package="px4_offboard_mission",
        executable="wildlife_mission_node",
        name="wildlife_mission_node",
        output="screen",
        parameters=[
            config_file,
            {
                "auto_arm_enabled": auto_arm_enabled,
                "auto_start": auto_start,
            },
        ],
    )

    serial_bridge = Node(
        package="px4_offboard_mission",
        executable="serial_route_bridge_node",
        name="serial_route_bridge_node",
        output="screen",
        condition=IfCondition(use_serial_bridge),
        parameters=[
            {
                "port": serial_port,
                "baud": ParameterValue(serial_baud, value_type=int),
                "start_after_route": ParameterValue(
                    serial_start_after_route,
                    value_type=bool,
                ),
                "report_tx_enabled": ParameterValue(
                    serial_report_enabled,
                    value_type=bool,
                ),
                "send_summary_each_report": ParameterValue(
                    send_summary_each_report,
                    value_type=bool,
                ),
            },
        ],
    )

    return LaunchDescription(args + [node, serial_bridge])
