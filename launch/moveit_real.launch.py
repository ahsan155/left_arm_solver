# Launch file for use with a REAL robot.
#
# Starts only the nodes the waypoint solver needs:
#   - robot_state_publisher   (TF from URDF)
#   - ordered_joint_states_bridge  (republishes /joint_states as /joint_states_ordered)
#   - move_group              (provides /plan_kinematic_path and /compute_fk services)
#   - RViz                    (optional, use_rviz:=false to suppress)
#
# Does NOT start ros2_control_node or spawn_controllers — those are for simulation only.
# The real robot hardware driver is expected to publish /joint_states externally.
#
# Typical usage:
#   Terminal 1: ros2 launch left_arm_solver moveit_real.launch.py
#   Terminal 2: ros2 launch left_arm_solver waypoint_node.launch.py execute_via_controller:=false

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("g1", package_name="left_arm_solver").to_moveit_configs()
    launch_package_path = moveit_config.package_path

    ordered_topic = "/joint_states_ordered"

    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="50.0"))

    # Publish a fixed odom->pelvis transform so MoveIt TF lookups succeed.
    # On a real mobile base you would replace this with actual odometry.
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_odom_to_pelvis",
            output="log",
            arguments=["0", "0", "0", "0", "0", "0", "1", "odom", "pelvis"],
        )
    )

    # Republish /joint_states with a fixed canonical joint ordering expected by the solver.
    ld.add_action(
        Node(
            package="left_arm_solver",
            executable="ordered_joint_states_bridge",
            output="screen",
            parameters=[
                {
                    "input_topic": "/joint_states",
                    "output_topic": ordered_topic,
                    "default_position_if_missing": 0.0,
                }
            ],
        )
    )

    # Publish TF from URDF so move_group and RViz can resolve all frames.
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            respawn=True,
            output="screen",
            parameters=[
                moveit_config.robot_description,
                {"publish_frequency": LaunchConfiguration("publish_frequency")},
            ],
        )
    )

    # move_group — provides /plan_kinematic_path and /compute_fk.
    # allow_trajectory_execution is false: the solver publishes directly to /arm_joint_cmd
    # instead of going through a ros2_control trajectory controller.
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    move_group_params = [
        moveit_config.to_dict(),
        {
            "publish_robot_description_semantic": True,
            "allow_trajectory_execution": False,
            "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
            "disable_capabilities": ParameterValue(
                LaunchConfiguration("disable_capabilities"), value_type=str
            ),
            "publish_planning_scene": should_publish,
            "publish_geometry_updates": should_publish,
            "publish_state_updates": should_publish,
            "publish_transforms_updates": should_publish,
            "monitor_dynamics": False,
        },
    ]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch/moveit_launch/gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )

    # Optional RViz for monitoring planned paths and robot state.
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    return ld
