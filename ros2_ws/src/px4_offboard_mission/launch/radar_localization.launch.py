import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration


ROS_SETUP = "/opt/ros/humble/setup.bash"


def _as_bool(value):
    return value.strip().lower() in ("1", "true", "yes", "on")


def _shell_command(fastlio_ws, launch_command):
    ws = shlex.quote(fastlio_ws)
    return (
        "set -e; "
        f"cd {ws}; "
        f"source {shlex.quote(ROS_SETUP)}; "
        f"source {shlex.quote(fastlio_ws + '/install/setup.bash')}; "
        f"exec {launch_command}"
    )


def _managed_process(name, fastlio_ws, launch_command):
    return ExecuteProcess(
        cmd=["bash", "-lc", _shell_command(fastlio_ws, launch_command)],
        name=name,
        output="screen",
        sigterm_timeout="4.0",
        sigkill_timeout="2.0",
    )


def _launch_setup(context, *args, **kwargs):
    del args, kwargs

    fastlio_ws = LaunchConfiguration("fastlio_ws").perform(context)
    fcu_url = LaunchConfiguration("fcu_url").perform(context)
    start_delay_s = float(LaunchConfiguration("start_delay_s").perform(context))
    use_mavros = _as_bool(LaunchConfiguration("use_mavros").perform(context))
    use_drone_bridge = _as_bool(LaunchConfiguration("use_drone_bridge").perform(context))

    steps = [
        _managed_process(
            "livox_mid360",
            fastlio_ws,
            "ros2 launch livox_ros_driver2 msg_MID360_launch.py",
        ),
        _managed_process(
            "fast_lio",
            fastlio_ws,
            "ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml",
        ),
    ]

    if use_mavros:
        steps.append(
            _managed_process(
                "mavros_px4",
                fastlio_ws,
                "ros2 launch mavros px4.launch fcu_url:=" + shlex.quote(fcu_url),
            )
        )

    if use_drone_bridge:
        steps.append(
            _managed_process(
                "fastlio_to_px4_odom",
                fastlio_ws,
                "ros2 launch drone_bridge fastlio_to_px4_odom.launch.py",
            )
        )

    actions = []
    for index, process in enumerate(steps):
        if index == 0:
            actions.append(process)
        else:
            actions.append(TimerAction(period=start_delay_s * index, actions=[process]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "fastlio_ws",
            default_value="/home/banana/fastlio_ws",
            description="FAST-LIO/Livox/drone_bridge workspace path.",
        ),
        DeclareLaunchArgument(
            "fcu_url",
            default_value="/dev/ttyUSB0:921600",
            description="MAVROS FCU serial URL, for example /dev/ttyUSB0:921600.",
        ),
        DeclareLaunchArgument(
            "start_delay_s",
            default_value="3.0",
            description="Delay between starting localization stack processes.",
        ),
        DeclareLaunchArgument(
            "use_mavros",
            default_value="true",
            description="Start MAVROS PX4 serial connection.",
        ),
        DeclareLaunchArgument(
            "use_drone_bridge",
            default_value="true",
            description="Start FAST-LIO to MAVROS odometry bridge.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
