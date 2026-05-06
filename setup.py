from setuptools import find_packages, setup

package_name = "left_arm_solver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
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
