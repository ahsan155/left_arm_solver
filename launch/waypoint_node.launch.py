"""Launch the G1 left-arm waypoint node with parameters from config/waypoint_node.yaml.

Usage (demo — MoveIt must already be running):
    ros2 launch left_arm_solver waypoint_node.launch.py

Real robot (disable controller execution, use /joint_states):
    ros2 launch left_arm_solver waypoint_node.launch.py \\
        execute_via_controller:=false \\
        joint_states_topic:=/joint_states
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(get_package_share_directory("left_arm_solver"))
    default_params = str(pkg_share / "config" / "waypoint_node.yaml")

    # Expose the most frequently overridden parameters as launch arguments so
    # callers can change them without editing the YAML file.
    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Full path to the waypoint_node parameter YAML file.",
        ),
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="/joint_states_ordered",
            description="Joint states input topic.",
        ),
        DeclareLaunchArgument(
            "execute_via_controller",
            default_value="true",
            description="Send trajectory to left_arm_controller (demo). Set false for real robot.",
        ),
        DeclareLaunchArgument(
            "hold_at_waypoint_sec",
            default_value="3.0",
            description="Seconds to hold at each waypoint before planning the next.",
        ),
    ]

    waypoint_node = Node(
        package="left_arm_solver",
        executable="g1_left_arm_waypoint_node",
        name="g1_left_arm_waypoint_node",
        output="screen",
        parameters=[
            # Load the full YAML file first …
            LaunchConfiguration("params_file"),
            # … then override individual params from launch arguments.
            # Launch-argument overrides take precedence over YAML values.
            {
                "joint_states_topic":     LaunchConfiguration("joint_states_topic"),
                "execute_via_controller": LaunchConfiguration("execute_via_controller"),
                "hold_at_waypoint_sec":   LaunchConfiguration("hold_at_waypoint_sec"),
            },
        ],
    )

    return LaunchDescription(args + [waypoint_node])
