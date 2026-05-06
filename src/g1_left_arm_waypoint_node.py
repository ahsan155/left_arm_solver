#!/usr/bin/env python3
"""Subscribe to /joint_states, plan left_arm pose goals with MoveIt2, publish Unitree /arm_joint_cmd."""

from __future__ import annotations

import copy
import math
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Pose, Quaternion, Vector3
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
    WorkspaceParameters,
)
from moveit_msgs.srv import GetMotionPlan, GetPositionFK
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float32MultiArray

from left_arm_solver.joint_table_target import (
    LEFT_ARM_JOINTS,
    UNITREE_CMD_JOINTS,
    UNITREE_LIMITS,
    target_waypoints,
)


class _Phase(Enum):
    WAIT_JOINT_STATES = auto()
    CAPTURE_EE_ORIENTATION = auto()
    PLAN_WAYPOINT = auto()
    HOLD = auto()
    FINISHED = auto()


def _clamp_cmd(values: List[float]) -> List[float]:
    out = []
    for i, v in enumerate(values):
        lo, hi = UNITREE_LIMITS[i]
        out.append(float(max(lo, min(hi, v))))
    return out


class G1LeftArmWaypointNode(Node):
    """Plans ``left_arm`` with MoveIt (pose position + fixed orientation), outputs 17-float Unitree command."""

    def __init__(self) -> None:
        super().__init__("g1_left_arm_waypoint_node")

        self.declare_parameter("joint_states_topic", "/joint_states_ordered")
        self.declare_parameter("arm_joint_cmd_topic", "/arm_joint_cmd")
        # MoveIt registers these as relative service names on the move_group node; in ROS 2 they
        # resolve to the root namespace (e.g. /plan_kinematic_path), not /move_group/...
        self.declare_parameter("plan_kinematic_path_service", "/plan_kinematic_path")
        self.declare_parameter("compute_fk_service", "/compute_fk")
        self.declare_parameter("planning_group", "left_arm")
        self.declare_parameter("eef_link", "left_wrist_yaw_link")
        self.declare_parameter("planning_frame", "torso_link")
        self.declare_parameter("position_goal_radius_m", 0.02)
        self.declare_parameter("orientation_tolerance_rad", 0.05)
        self.declare_parameter("hold_at_waypoint_sec", 3.0)
        self.declare_parameter("cmd_publish_hz", 20.0)
        self.declare_parameter("planning_time_sec", 10.0)
        self.declare_parameter("num_planning_attempts", 8)
        self.declare_parameter("max_velocity_scaling", 0.4)
        self.declare_parameter("max_acceleration_scaling", 0.4)
        # Names that must appear on /joint_states before FK + planning (default: SRDF left_arm only).
        self.declare_parameter("joint_states_required_for_start", LEFT_ARM_JOINTS)
        # Used for UNITREE_CMD slots not present on /joint_states (e.g. waist when sim omits them).
        self.declare_parameter("default_position_if_joint_missing", 0.0)

        plan_srv = (
            self.get_parameter("plan_kinematic_path_service").get_parameter_value().string_value
        )
        fk_srv = self.get_parameter("compute_fk_service").get_parameter_value().string_value

        self._plan_cli = self.create_client(GetMotionPlan, plan_srv)
        self._fk_cli = self.create_client(GetPositionFK, fk_srv)

        self.get_logger().info(f"Waiting for MoveIt services: '{plan_srv}', '{fk_srv}'")
        if not self._plan_cli.wait_for_service(timeout_sec=60.0):
            self.get_logger().error(f"Timed out waiting for {plan_srv}")
        if not self._fk_cli.wait_for_service(timeout_sec=60.0):
            self.get_logger().error(f"Timed out waiting for {fk_srv}")

        self._joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )
        self._cmd_topic = (
            self.get_parameter("arm_joint_cmd_topic").get_parameter_value().string_value
        )
        self._planning_group = (
            self.get_parameter("planning_group").get_parameter_value().string_value
        )
        self._eef_link = self.get_parameter("eef_link").get_parameter_value().string_value
        self._planning_frame = (
            self.get_parameter("planning_frame").get_parameter_value().string_value
        )
        self._position_tol = (
            self.get_parameter("position_goal_radius_m").get_parameter_value().double_value
        )
        self._orient_tol = (
            self.get_parameter("orientation_tolerance_rad").get_parameter_value().double_value
        )
        self._hold_sec = (
            self.get_parameter("hold_at_waypoint_sec").get_parameter_value().double_value
        )
        cmd_hz = self.get_parameter("cmd_publish_hz").get_parameter_value().double_value
        self._planning_time = (
            self.get_parameter("planning_time_sec").get_parameter_value().double_value
        )
        self._plan_attempts = (
            self.get_parameter("num_planning_attempts").get_parameter_value().integer_value
        )
        self._vel_scale = (
            self.get_parameter("max_velocity_scaling").get_parameter_value().double_value
        )
        self._acc_scale = (
            self.get_parameter("max_acceleration_scaling").get_parameter_value().double_value
        )
        req = self.get_parameter("joint_states_required_for_start").get_parameter_value().string_array_value
        self._required_joint_states = list(req) if req else list(LEFT_ARM_JOINTS)
        self._default_missing_joint = (
            self.get_parameter("default_position_if_joint_missing").get_parameter_value().double_value
        )
        self._logged_missing_cmd_joints = False

        self._js_positions: Dict[str, float] = {}
        self._cmd_vector: Optional[List[float]] = None
        self._ee_orientation: Optional[Quaternion] = None
        self._phase = _Phase.WAIT_JOINT_STATES
        self._waypoint_index = 0
        self._hold_until = None
        self._waypoints: List[Tuple[float, float, float]] = [
            tuple(p) for p in target_waypoints
        ]

        self._joint_states_sub = self.create_subscription(
            JointState, self._joint_states_topic, self._on_joint_states, 10
        )
        self._cmd_pub = self.create_publisher(Float32MultiArray, self._cmd_topic, 10)

        period = 1.0 / max(cmd_hz, 1.0)
        self.create_timer(period, self._publish_cmd)
        self.create_timer(0.05, self._tick_state_machine)

        self.get_logger().info(
            f"Publishing {self._cmd_topic} at ~{cmd_hz} Hz; "
            f"planning group={self._planning_group}, eef={self._eef_link}, "
            f"frame={self._planning_frame}; "
            f"waiting for joints on {self._joint_states_topic}: {self._required_joint_states}"
        )

    def _on_joint_states(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if math.isnan(pos):
                continue
            self._js_positions[name] = float(pos)

    def _merge_unitree_cmd(
        self, traj_joint_names: List[str], traj_positions: List[float]
    ) -> List[float]:
        d = self._default_missing_joint
        cmd = [self._js_positions.get(j, d) for j in UNITREE_CMD_JOINTS]
        for name, pos in zip(traj_joint_names, traj_positions):
            if name in UNITREE_CMD_JOINTS:
                idx = UNITREE_CMD_JOINTS.index(name)
                cmd[idx] = float(pos)
        return _clamp_cmd(cmd)

    def _robot_state_from_subscription(self) -> RobotState:
        rs = RobotState()
        rs.joint_state.name = list(self._js_positions.keys())
        rs.joint_state.position = [float(self._js_positions[n]) for n in rs.joint_state.name]
        return rs

    def _fk_call_initial_orientation(self) -> bool:
        req = GetPositionFK.Request()
        req.header.frame_id = self._planning_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.fk_link_names = [self._eef_link]
        req.robot_state = self._robot_state_from_subscription()

        res = self._fk_cli.call(req)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(
                f"compute_fk failed ({res.error_code.val}); cannot lock EE orientation yet."
            )
            return False
        if not res.pose_stamped:
            self.get_logger().warn("compute_fk returned no poses.")
            return False

        self._ee_orientation = copy.deepcopy(res.pose_stamped[0].pose.orientation)
        self.get_logger().info(
            "Captured initial EE orientation from live joint state (compute_fk)."
        )
        return True

    def _goal_constraints_for_position(self, xyz: Tuple[float, float, float]) -> Constraints:
        x, y, z = xyz
        goal = Constraints()

        pos = PositionConstraint()
        pos.header.frame_id = self._planning_frame
        pos.link_name = self._eef_link
        pos.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(max(self._position_tol, 0.001))]

        region = BoundingVolume()
        region.primitives.append(sphere)
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0
        region.primitive_poses.append(pose)
        pos.constraint_region = region
        pos.weight = 1.0

        ori = OrientationConstraint()
        ori.header.frame_id = self._planning_frame
        ori.link_name = self._eef_link
        assert self._ee_orientation is not None
        ori.orientation = copy.deepcopy(self._ee_orientation)
        ori.absolute_x_axis_tolerance = float(self._orient_tol)
        ori.absolute_y_axis_tolerance = float(self._orient_tol)
        ori.absolute_z_axis_tolerance = float(self._orient_tol)
        ori.parameterization = OrientationConstraint.XYZ_EULER_ANGLES
        ori.weight = 1.0

        goal.position_constraints.append(pos)
        goal.orientation_constraints.append(ori)
        return goal

    def _workspace(self) -> WorkspaceParameters:
        wp = WorkspaceParameters()
        wp.header.frame_id = self._planning_frame
        wp.min_corner.x = -2.0
        wp.min_corner.y = -2.0
        wp.min_corner.z = -2.0
        wp.max_corner.x = 2.0
        wp.max_corner.y = 2.0
        wp.max_corner.z = 2.0
        return wp

    def _plan_to_xyz(self, xyz: Tuple[float, float, float]) -> bool:
        req = MotionPlanRequest()
        req.workspace_parameters = self._workspace()
        req.start_state = RobotState()
        req.goal_constraints = [self._goal_constraints_for_position(xyz)]
        req.group_name = self._planning_group
        req.num_planning_attempts = int(self._plan_attempts)
        req.allowed_planning_time = float(self._planning_time)
        req.max_velocity_scaling_factor = float(max(min(self._vel_scale, 1.0), 0.01))
        req.max_acceleration_scaling_factor = float(max(min(self._acc_scale, 1.0), 0.01))

        plan_req = GetMotionPlan.Request()
        plan_req.motion_plan_request = req

        self.get_logger().info(f"Planning to waypoint {xyz} (group={self._planning_group})")
        res = self._plan_cli.call(plan_req)
        mp = res.motion_plan_response
        if mp.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Planning failed error_code={mp.error_code.val} at waypoint {xyz}"
            )
            return False

        traj = mp.trajectory.joint_trajectory
        if not traj.points:
            self.get_logger().error("Planner returned empty trajectory.")
            return False

        last = traj.points[-1]
        self._cmd_vector = self._merge_unitree_cmd(traj.joint_names, list(last.positions))
        self.get_logger().info(f"Planned OK; publishing merged /arm_joint_cmd for waypoint {xyz}")
        return True

    def _publish_cmd(self) -> None:
        if self._cmd_vector is None:
            return
        msg = Float32MultiArray()
        msg.data = [float(x) for x in self._cmd_vector]
        self._cmd_pub.publish(msg)

    def _tick_state_machine(self) -> None:
        if self._phase == _Phase.FINISHED:
            return

        if self._phase == _Phase.WAIT_JOINT_STATES:
            if not all(j in self._js_positions for j in self._required_joint_states):
                return
            missing_cmd = [j for j in UNITREE_CMD_JOINTS if j not in self._js_positions]
            if missing_cmd and not self._logged_missing_cmd_joints:
                self.get_logger().warn(
                    "These UNITREE_CMD_JOINTS are not on "
                    f"{self._joint_states_topic} (using default_position_if_joint_missing="
                    f"{self._default_missing_joint} for them): {missing_cmd}. "
                    "Real robot / bag order may match ALL_JOINT_STATES_JOINTS; this is fine."
                )
                self._logged_missing_cmd_joints = True
            self._phase = _Phase.CAPTURE_EE_ORIENTATION

        if self._phase == _Phase.CAPTURE_EE_ORIENTATION:
            if self._fk_call_initial_orientation():
                self._phase = _Phase.PLAN_WAYPOINT
            else:
                self._phase = _Phase.WAIT_JOINT_STATES

        elif self._phase == _Phase.PLAN_WAYPOINT:
            if self._waypoint_index >= len(self._waypoints):
                self.get_logger().info("All waypoints completed.")
                self._phase = _Phase.FINISHED
                return
            xyz = self._waypoints[self._waypoint_index]
            if self._plan_to_xyz(xyz):
                self._hold_until = self.get_clock().now() + Duration(seconds=self._hold_sec)
                self._phase = _Phase.HOLD
            else:
                self.get_logger().warn("Stopping waypoint sequence due to planning failure.")
                self._phase = _Phase.FINISHED

        elif self._phase == _Phase.HOLD:
            if self._hold_until is None:
                self._phase = _Phase.PLAN_WAYPOINT
                return
            if self.get_clock().now() >= self._hold_until:
                self._waypoint_index += 1
                self._phase = _Phase.PLAN_WAYPOINT


def main() -> None:
    rclpy.init()
    node = G1LeftArmWaypointNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
