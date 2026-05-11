#!/usr/bin/env python3
"""Subscribe to /joint_states, plan left_arm pose goals with MoveIt2, publish Unitree /arm_joint_cmd."""

from __future__ import annotations

import math
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Quaternion
from moveit_msgs.msg import (
    DisplayRobotState,
    DisplayTrajectory,
    MotionPlanRequest,
    MoveItErrorCodes,
)
from moveit_msgs.srv import GetMotionPlan, GetPositionFK
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Float32MultiArray

from left_arm_solver.joint_table_target import (
    LEFT_ARM_JOINTS,
    UNITREE_CMD_JOINTS,
    UNITREE_LIMITS,
    target_waypoints,
)
from left_arm_solver.utils import (
    call_fk_for_orientation,
    goal_constraints_for_position,
    make_display_robot_state,
    robot_state_from_positions,
    workspace_parameters,
)

# RGBA colors for start (green) and goal (orange) arm state displays.
_COLOR_START = ColorRGBA(r=0.0, g=0.85, b=0.2, a=0.9)
_COLOR_GOAL = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)


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
        # When True, execute the planned trajectory via left_arm_controller so the Scene Robot
        # in RViz moves. Set False on the real robot (Unitree SDK handles execution via /arm_joint_cmd).
        self.declare_parameter("execute_via_controller", True)
        self.declare_parameter("controller_action", "/left_arm_controller/follow_joint_trajectory")

        plan_srv = (
            self.get_parameter("plan_kinematic_path_service").get_parameter_value().string_value
        )
        fk_srv = self.get_parameter("compute_fk_service").get_parameter_value().string_value

        # Default MutuallyExclusive group + blocking ServiceClient.call() from a timer deadlocks:
        # the timer holds the group while waiting, so the client response cannot run.
        self._cb_group = ReentrantCallbackGroup()

        self._plan_cli = self.create_client(
            GetMotionPlan, plan_srv, callback_group=self._cb_group
        )
        self._fk_cli = self.create_client(
            GetPositionFK, fk_srv, callback_group=self._cb_group
        )

        self._execute_via_controller = (
            self.get_parameter("execute_via_controller").get_parameter_value().bool_value
        )
        ctrl_action = (
            self.get_parameter("controller_action").get_parameter_value().string_value
        )
        self._exec_cli: Optional[ActionClient] = None
        if self._execute_via_controller:
            self._exec_cli = ActionClient(
                self, FollowJointTrajectory, ctrl_action, callback_group=self._cb_group
            )

        self.get_logger().info(f"Waiting for MoveIt services: '{plan_srv}', '{fk_srv}'")
        if not self._plan_cli.wait_for_service(timeout_sec=60.0):
            self.get_logger().error(f"Timed out waiting for {plan_srv}")
        if not self._fk_cli.wait_for_service(timeout_sec=60.0):
            self.get_logger().error(f"Timed out waiting for {fk_srv}")
        if self._exec_cli is not None:
            self.get_logger().info(f"Waiting for controller action: '{ctrl_action}'")
            if not self._exec_cli.wait_for_server(timeout_sec=30.0):
                self.get_logger().warn(
                    f"Controller action '{ctrl_action}' not available; "
                    "execution disabled (set execute_via_controller:=false to suppress)."
                )
                self._exec_cli = None

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
        self._fk_call_in_progress = False
        self._plan_call_in_progress = False

        self._js_positions: Dict[str, float] = {}
        self._cmd_vector: Optional[List[float]] = None
        self._last_traj = None          # stores last planned JointTrajectory for execution
        self._traj_start_time = None    # ROS time when streaming of current traj began
        self._traj_point_index: int = 0 # index of the last point published from _last_traj
        self._ee_orientation: Optional[Quaternion] = None
        self._phase = _Phase.WAIT_JOINT_STATES
        self._waypoint_index = 0
        self._hold_until = None
        self._waypoints: List[Tuple[float, float, float]] = [
            tuple(p) for p in target_waypoints
        ]

        self._joint_states_sub = self.create_subscription(
            JointState,
            self._joint_states_topic,
            self._on_joint_states,
            10,
            callback_group=self._cb_group,
        )
        self._cmd_pub = self.create_publisher(Float32MultiArray, self._cmd_topic, 10)
        # Drives RViz "Planned Path" trajectory animation (the moving shadow).
        self._display_traj_pub = self.create_publisher(
            DisplayTrajectory, "/display_planned_path", 10
        )
        # Drives separate RobotState displays in RViz for the coloured start/goal arms.
        self._display_start_pub = self.create_publisher(
            DisplayRobotState, "/display_start_robot_state", 10
        )
        self._display_goal_pub = self.create_publisher(
            DisplayRobotState, "/display_goal_robot_state", 10
        )

        period = 1.0 / max(cmd_hz, 1.0)
        self.create_timer(period, self._publish_cmd, callback_group=self._cb_group)
        self.create_timer(0.05, self._tick_state_machine, callback_group=self._cb_group)

        self.get_logger().info(
            f"Publishing {self._cmd_topic} at ~{cmd_hz} Hz; "
            f"planning group={self._planning_group}, eef={self._eef_link}, "
            f"frame={self._planning_frame}; "
            f"waiting for joints on {self._joint_states_topic}: {self._required_joint_states}"
        )

    def _on_joint_states(self, msg: JointState) -> None:
        ln, lp = len(msg.name), len(msg.position)
        if ln != lp:
            self.get_logger().warn(
                f"JointState name/position length mismatch ({ln} vs {lp}); using first {min(ln, lp)} pairs.",
                throttle_duration_sec=5.0,
            )
        n = min(ln, lp)
        d = self._default_missing_joint
       
        for i in range(n):
            name = msg.name[i]
            pos = msg.position[i]
      
            if math.isnan(pos) or math.isinf(pos):
                pos = d
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

    def _execute_trajectory_blocking(self) -> bool:
        """Send the last planned trajectory to the controller and block until done.

        Uses threading.Event so the current thread sleeps while the executor's other
        threads process the action client callbacks (ReentrantCallbackGroup is required).
        """
        if self._exec_cli is None or self._last_traj is None:
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._last_traj

        done = threading.Event()
        outcome: List[bool] = [False]

        def _on_goal_response(future: "rclpy.Future") -> None:
            handle = future.result()
            if not handle.accepted:
                self.get_logger().error("Trajectory goal rejected by controller.")
                done.set()
                return
            handle.get_result_async().add_done_callback(_on_result)

        def _on_result(future: "rclpy.Future") -> None:
            res = future.result()
            outcome[0] = (res.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL)
            if not outcome[0]:
                self.get_logger().warn(
                    f"Controller returned error_code={res.result.error_code}"
                )
            done.set()

        self._exec_cli.send_goal_async(goal).add_done_callback(_on_goal_response)

        # Timeout = trajectory duration + generous buffer so we don't hang forever.
        if self._last_traj.points:
            last_pt = self._last_traj.points[-1]
            traj_sec = last_pt.time_from_start.sec + last_pt.time_from_start.nanosec * 1e-9
        else:
            traj_sec = 0.0
        timeout = max(traj_sec + 10.0, 20.0)

        if not done.wait(timeout=timeout):
            self.get_logger().error("Timeout waiting for trajectory execution.")
            return False
        return outcome[0]

    def _plan_to_xyz(self, xyz: Tuple[float, float, float]) -> bool:
        # Snapshot current state before the blocking call so the start state is consistent.
        start_state = robot_state_from_positions(self._js_positions)

        req = MotionPlanRequest()
        req.workspace_parameters = workspace_parameters(self._planning_frame)
        # Full joint vector matches monitored scene; empty joint_state causes MoveIt conversion errors.
        req.start_state = start_state
        req.goal_constraints = [goal_constraints_for_position(
            xyz, self._planning_frame, self._eef_link,
            self._position_tol, self._ee_orientation, self._orient_tol,
        )]
        req.group_name = self._planning_group
        req.num_planning_attempts = int(self._plan_attempts)
        req.allowed_planning_time = float(self._planning_time)
        req.max_velocity_scaling_factor = float(max(min(self._vel_scale, 1.0), 0.01))
        req.max_acceleration_scaling_factor = float(max(min(self._acc_scale, 1.0), 0.01))

        plan_req = GetMotionPlan.Request()
        plan_req.motion_plan_request = req

        # Show green start-state arm in RViz before blocking on the plan call.
        self._display_start_pub.publish(
            make_display_robot_state(dict(self._js_positions), _COLOR_START)
        )

        self.get_logger().info(f"Planning to waypoint {xyz} (group={self._planning_group})")
        res = self._plan_cli.call(plan_req)
        mp = res.motion_plan_response
        self.get_logger().info(
            f"Planning response error_code={mp.error_code.val} time={mp.planning_time:.3f}s "
            f"for waypoint {xyz}"
        )
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
        self._last_traj = traj
        # Arm the streaming publisher: _publish_cmd will walk through every point.
        self._traj_start_time  = self.get_clock().now()
        self._traj_point_index = 0
        self.get_logger().info(f"Planned OK; publishing merged /arm_joint_cmd for waypoint {xyz}")

        # Publish trajectory animation (the moving shadow in RViz Planned Path).
        disp_traj = DisplayTrajectory()
        disp_traj.trajectory_start = start_state
        disp_traj.trajectory = [mp.trajectory]
        self._display_traj_pub.publish(disp_traj)

        # Build goal joint map: current full robot, arm joints overridden to goal config.
        goal_js = dict(self._js_positions)
        for name, pos in zip(traj.joint_names, last.positions):
            goal_js[name] = float(pos)
        self._display_goal_pub.publish(
            make_display_robot_state(goal_js, _COLOR_GOAL)
        )

        return True

    def _publish_cmd(self) -> None:
        if self._cmd_vector is None:
            return

        traj = self._last_traj
        if traj is None or self._traj_start_time is None:
            # No trajectory active — hold the last commanded position.
            msg = Float32MultiArray()
            msg.data = [float(x) for x in self._cmd_vector]
            self._cmd_pub.publish(msg)
            return

        # Advance the point index until we find the point whose time_from_start
        # has not yet been reached.  This makes _publish_cmd step through every
        # intermediate point at the rate the planner intended.
        elapsed = (self.get_clock().now() - self._traj_start_time).nanoseconds * 1e-9
        while self._traj_point_index + 1 < len(traj.points):
            next_pt = traj.points[self._traj_point_index + 1]
            next_t  = (next_pt.time_from_start.sec
                       + next_pt.time_from_start.nanosec * 1e-9)
            if elapsed >= next_t:
                self._traj_point_index += 1
            else:
                break

        pt = traj.points[self._traj_point_index]
        cmd = self._merge_unitree_cmd(traj.joint_names, list(pt.positions))

        # Once the final point is reached keep _cmd_vector in sync so the
        # hold-phase "freeze at goal" logic continues to work.
        if self._traj_point_index == len(traj.points) - 1:
            self._cmd_vector = cmd

        msg = Float32MultiArray()
        msg.data = [float(x) for x in cmd]
        self._cmd_pub.publish(msg)

    def _tick_state_machine(self) -> None:
        if self._phase == _Phase.FINISHED:
            return

        if self._phase == _Phase.WAIT_JOINT_STATES:
            missing_req = [j for j in self._required_joint_states if j not in self._js_positions]
            if missing_req:
                self.get_logger().warn(
                    f"Still waiting for required joints on {self._joint_states_topic}: {missing_req}",
                    throttle_duration_sec=5.0,
                )
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
            if self._fk_call_in_progress:
                return
            self._fk_call_in_progress = True
            try:
                orientation = call_fk_for_orientation(
                    self._fk_cli,
                    self._js_positions,
                    self._planning_frame,
                    self._eef_link,
                    self.get_clock().now().to_msg(),
                    self.get_logger(),
                )
                if orientation is not None:
                    self._ee_orientation = orientation
                    self._phase = _Phase.PLAN_WAYPOINT
                else:
                    self._phase = _Phase.WAIT_JOINT_STATES
            finally:
                self._fk_call_in_progress = False
        elif self._phase == _Phase.PLAN_WAYPOINT:
            if self._plan_call_in_progress:
                return
            if self._waypoint_index >= len(self._waypoints):
                self.get_logger().info("All waypoints completed.")
                self._phase = _Phase.FINISHED
                return
            self._plan_call_in_progress = True
            try:
                xyz = self._waypoints[self._waypoint_index]
                if self._plan_to_xyz(xyz):
                    # Execute the trajectory so the Scene Robot moves in RViz.
                    # The mock hardware mirrors commands → joint_state_broadcaster updates
                    # /joint_states → Scene Robot follows the arm to the goal position.
                    if self._exec_cli is not None:
                        self.get_logger().info(
                            f"Executing trajectory for waypoint {xyz} via controller…"
                        )
                        ok = self._execute_trajectory_blocking()
                        if ok:
                            self.get_logger().info(
                                f"Execution complete for waypoint {xyz}."
                            )
                        else:
                            self.get_logger().warn(
                                "Execution returned failure; holding and continuing."
                            )
                    self._hold_until = self.get_clock().now() + Duration(seconds=self._hold_sec)
                    self._phase = _Phase.HOLD
                else:
                    self.get_logger().warn("Stopping waypoint sequence due to planning failure.")
                    self._phase = _Phase.FINISHED
            finally:
                self._plan_call_in_progress = False

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
