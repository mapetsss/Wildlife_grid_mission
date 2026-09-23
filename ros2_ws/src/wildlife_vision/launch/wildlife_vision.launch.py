from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    model_path = LaunchConfiguration("model_path")
    classes_path = LaunchConfiguration("classes_path")
    image_topic = LaunchConfiguration("image_topic")
    enable_serial_report = LaunchConfiguration("enable_serial_report")
    serial_port = LaunchConfiguration("serial_port")
    serial_baud = LaunchConfiguration("serial_baud")
    send_summary_each_report = LaunchConfiguration("send_summary_each_report")
    debug_stream_enabled = LaunchConfiguration("debug_stream_enabled")
    debug_stream_rate_hz = LaunchConfiguration("debug_stream_rate_hz")

    default_model = PathJoinSubstitution(
        [
            FindPackageShare("ros2_yolos_cpp"),
            "models",
            "drone_yolo26n_480x640.onnx",
        ]
    )
    default_classes = PathJoinSubstitution(
        [FindPackageShare("ros2_yolos_cpp"), "models", "classes.txt"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("wildlife_vision"), "config", "vision.yaml"]
                ),
            ),
            DeclareLaunchArgument("model_path", default_value=default_model),
            DeclareLaunchArgument("classes_path", default_value=default_classes),
            DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("enable_serial_report", default_value="false"),
            DeclareLaunchArgument("serial_port", default_value="/dev/serial0"),
            DeclareLaunchArgument("serial_baud", default_value="115200"),
            DeclareLaunchArgument("send_summary_each_report", default_value="false"),
            DeclareLaunchArgument("debug_stream_enabled", default_value="false"),
            DeclareLaunchArgument("debug_stream_rate_hz", default_value="3.0"),
            Node(
                package="wildlife_vision",
                executable="wildlife_vision_node",
                name="wildlife_vision_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "model_path": model_path,
                        "classes_path": classes_path,
                        "image_topic": image_topic,
                        "debug_stream_enabled": ParameterValue(
                            debug_stream_enabled,
                            value_type=bool,
                        ),
                        "debug_stream_rate_hz": ParameterValue(
                            debug_stream_rate_hz,
                            value_type=float,
                        ),
                    },
                ],
            ),
            Node(
                package="wildlife_vision",
                executable="wildlife_report_serial_node",
                name="wildlife_report_serial_node",
                output="screen",
                condition=IfCondition(enable_serial_report),
                parameters=[
                    {
                        "port": serial_port,
                        "baud": ParameterValue(serial_baud, value_type=int),
                        "send_summary_each_report": ParameterValue(
                            send_summary_each_report,
                            value_type=bool,
                        ),
                    },
                ],
            ),
        ]
    )
