# Copyright (c) 2026 — Same stack as demo.launch.py but:
#   * ros2_control still publishes /joint_states (partial list from hardware mock).
#   * ordered_joint_states_bridge republishes /joint_states_ordered with every joint name in
#     ALL_JOINT_STATES_JOINTS order (see left_arm_solver.joint_table_target), filling missing
#     slots with default_position_if_missing (0.0).
#   * robot_state_publisher and move_group remain on /joint_states (MoveIt cannot accept names
#     that do not exist in its robot model).
#   * solver nodes can subscribe to /joint_states_ordered to match rosbag ordering.
#
# To use with real hardware: publish driver joint_states to /joint_states (or set bridge
# input_topic); keep output on /joint_states_ordered for downstream consumers.

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

    # Single place to change if you prefer another topic name.
    ordered_topic = "/joint_states_ordered"

    ld = LaunchDescription()

    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    virtual_joints_launch = launch_package_path / "launch/moveit_launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # g1 URDF uses link ``odom`` + floating_base_joint -> pelvis. Without MultiDOF joint state,
    # robot_state_publisher never publishes odom->pelvis, so frames like ``odom`` are missing and
    # RViz/MoveIt TF lookups fail. Fixed-base demo: lock pelvis to odom with identity.
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_odom_to_pelvis",
            output="log",
            arguments=["0", "0", "0", "0", "0", "0", "1", "odom", "pelvis"],
        )
    )

    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="50.0"))

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

    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
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
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }
    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
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

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_launch/spawn_controllers.launch.py")
            ),
        )
    )

    return ld
