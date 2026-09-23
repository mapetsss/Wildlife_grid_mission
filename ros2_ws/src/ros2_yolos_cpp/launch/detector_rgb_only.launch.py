from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("ros2_yolos_cpp")
    default_model = PathJoinSubstitution(
        [package_share, "models", "drone_yolo26n_480x640.onnx"]
    )
    default_labels = PathJoinSubstitution([package_share, "models", "classes.txt"])

    args = [
        DeclareLaunchArgument(
            "model_path", default_value=default_model, description="Path to ONNX model"
        ),
        DeclareLaunchArgument(
            "labels_path",
            default_value=default_labels,
            description="Path to class labels",
        ),
        DeclareLaunchArgument("yolo_version", default_value="v26"),
        DeclareLaunchArgument("use_gpu", default_value="false"),
        DeclareLaunchArgument("conf_threshold", default_value="0.4"),
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument(
            "camera_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("mono_camera_capture"),
                    "config",
                    "d435_rgb_only.yaml",
                ]
            ),
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ros2_yolos_cpp"), "config", "rgb_only_params.yaml"]
            ),
        ),
        DeclareLaunchArgument("service_name", default_value="~/detect"),
    ]

    camera_node = Node(
        package="mono_camera_capture",
        executable="mono_camera_node",
        name="d435_camera_node",
        parameters=[LaunchConfiguration("camera_params_file")],
        remappings=[
            ("color/image_raw", LaunchConfiguration("image_topic")),
            ("color/camera_info", "/camera/color/camera_info"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_camera")),
    )

    container = ComposableNodeContainer(
        name="yolos_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="ros2_yolos_cpp",
                plugin="ros2_yolos_cpp::YolosDetectorNode",
                name="yolos_detector",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "labels_path": LaunchConfiguration("labels_path"),
                        "use_gpu": LaunchConfiguration("use_gpu"),
                        "yolo_version": LaunchConfiguration("yolo_version"),
                        "use_depth": False,
                        "conf_threshold": LaunchConfiguration("conf_threshold"),
                        "nms_threshold": 0.45,
                        "publish_timing": True,
                    },
                ],
                remappings=[
                    ("~/image_raw", LaunchConfiguration("image_topic")),
                    ("~/detect", LaunchConfiguration("service_name")),
                ],
            )
        ],
        output="screen",
    )

    return LaunchDescription(args + [camera_node, container])
