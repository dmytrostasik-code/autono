import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORKSPACE = os.environ.get("ARDUROVER_NAV_ROOT", "/home/developer/ardurover_navigation")
SCRIPTS = os.path.join(WORKSPACE, "scripts")


def generate_launch_description() -> LaunchDescription:
    gz_gui = DeclareLaunchArgument(
        "gz_gui",
        default_value="true",
        description="Start Gazebo GUI. Set false for headless.",
    )

    gz = ExecuteProcess(
        cmd=["bash", os.path.join(SCRIPTS, "run-gz.sh")],
        additional_env={"GZ_GUI": LaunchConfiguration("gz_gui")},
        output="screen",
    )
    ardurover = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=["bash", os.path.join(SCRIPTS, "run-ardurover.sh")],
                output="screen",
            )
        ],
    )
    bridge = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ground_truth_bridge",
                arguments=[
                    "/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                ],
                output="screen",
            )
        ],
    )
    mavros = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("mavros"), "launch", "apm.launch")
                ),
                launch_arguments={"fcu_url": "udp://:14551@", "gcs_url": ""}.items(),
            )
        ],
    )

    return LaunchDescription([gz_gui, gz, bridge, ardurover, mavros])
