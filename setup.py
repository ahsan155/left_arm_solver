from glob import glob
from setuptools import find_packages, setup

package_name = "left_arm_solver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/moveit_demo.launch.py",
            "launch/moveit_real.launch.py",
            "launch/waypoint_node.launch.py",
        ]),
        ("share/" + package_name + "/launch/moveit_launch", [
            "launch/moveit_launch/move_group.launch.py",
            "launch/moveit_launch/moveit_rviz.launch.py",
            "launch/moveit_launch/rsp.launch.py",
            "launch/moveit_launch/setup_assistant.launch.py",
            "launch/moveit_launch/spawn_controllers.launch.py",
            "launch/moveit_launch/static_virtual_joint_tfs.launch.py",
            "launch/moveit_launch/warehouse_db.launch.py",
        ]),
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf") + glob("urdf/*.xml") + glob("urdf/README.md")),
        ("share/" + package_name + "/urdf/meshes", glob("urdf/meshes/*.STL")),
        ("share/" + package_name + "/config", [
            "config/waypoint_node.yaml",
            "config/g1.ros2_control.xacro",
            "config/g1.srdf",
            "config/g1.urdf.xacro",
            "config/initial_positions.yaml",
            "config/joint_limits.yaml",
            "config/kinematics.yaml",
            "config/moveit_controllers.yaml",
            "config/moveit.rviz",
            "config/pilz_cartesian_limits.yaml",
            "config/ros2_controllers.yaml",
            "config/sensors_3d.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahsan Ahmed",
    maintainer_email="ahsan@scaledrive.ai",
    description="MoveIt2 waypoint node for G1 left arm + Unitree arm_joint_cmd output.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "g1_left_arm_waypoint_node = left_arm_solver.g1_left_arm_waypoint_node:main",
            "ordered_joint_states_bridge = left_arm_solver.ordered_joint_states_bridge:main",
        ],
    },
)
