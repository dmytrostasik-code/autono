import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORKSPACE = os.environ.get("ARDUROVER_NAV_ROOT", "/home/developer/ardurover_navigation")
PKG_SHARE = get_package_share_directory("ardurover_nav")


def generate_launch_description() -> LaunchDescription:
    gz_gui = DeclareLaunchArgument(
        "gz_gui",
        default_value="true",
        description="Start Gazebo GUI. Set false for headless.",
    )
    path_file = DeclareLaunchArgument(
        "path_file",
        default_value=os.path.join(WORKSPACE, "paths", "recorded.path"),
    )
    output_file = DeclareLaunchArgument(
        "output_file",
        default_value=os.path.join(WORKSPACE, "paths", "score.txt"),
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(PKG_SHARE, "launch", "sim.launch.py")),
        launch_arguments={"gz_gui": LaunchConfiguration("gz_gui")}.items(),
    )
    controller = Node(
        package="ardurover_nav",
        executable="trajectory_controller_node",
        name="trajectory_controller_node",
        output="screen",
        parameters=[{"path_file": LaunchConfiguration("path_file")}],
    )
    scorer = Node(
        package="ardurover_nav",
        executable="path_scorer_node",
        name="path_scorer_node",
        output="screen",
        parameters=[
            {
                "path_file": LaunchConfiguration("path_file"),
                "output_file": LaunchConfiguration("output_file"),
            }
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(PKG_SHARE, "rviz", "ugv.rviz")],
        output="screen",
    )

    return LaunchDescription([gz_gui, path_file, output_file, sim, controller, scorer, rviz])
