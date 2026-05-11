"""Stateless helper functions shared across left_arm_solver nodes."""

import copy
from typing import Dict, Optional, Tuple

from geometry_msgs.msg import Pose, Quaternion, Vector3
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    DisplayRobotState,
    MoveItErrorCodes,
    ObjectColor,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
    WorkspaceParameters,
)
from moveit_msgs.srv import GetPositionFK
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

# Child links of the LEFT_ARM_JOINTS chain (used for highlight coloring in RViz).
LEFT_ARM_LINKS: list[str] = [
    "left_shoulder_pitch_link",
    "left_shoulder_roll_link",
    "left_shoulder_yaw_link",
    "left_elbow_link",
    "left_wrist_roll_link",
    "left_wrist_pitch_link",
    "left_wrist_yaw_link",
]


def robot_state_from_positions(js_positions: Dict[str, float]) -> RobotState:
    """Build a MoveIt RobotState from a joint-name → position dict."""
    rs = RobotState()
    rs.joint_state.name = list(js_positions.keys())
    rs.joint_state.position = [float(js_positions[n]) for n in rs.joint_state.name]
    return rs


def make_display_robot_state(
    js_positions: Dict[str, float], color: ColorRGBA
) -> DisplayRobotState:
    """Build a DisplayRobotState with left-arm links highlighted in the given color."""
    rs = robot_state_from_positions(js_positions)
    disp = DisplayRobotState()
    disp.state = rs
    for link in LEFT_ARM_LINKS:
        oc = ObjectColor()
        oc.id = link
        oc.color = color
        disp.highlight_links.append(oc)
    return disp


def call_fk_for_orientation(
    fk_cli,
    js_positions: Dict[str, float],
    planning_frame: str,
    eef_link: str,
    stamp,
    logger,
) -> Optional[Quaternion]:
    """Call compute_fk and return the EE orientation, or None on failure.

    Returns the orientation so the caller can store it; avoids side-effects.
    """
    req = GetPositionFK.Request()
    req.header.frame_id = planning_frame
    req.header.stamp = stamp
    req.fk_link_names = [eef_link]
    req.robot_state = robot_state_from_positions(js_positions)
    res = fk_cli.call(req)

    if res.error_code.val != MoveItErrorCodes.SUCCESS:
        logger.warn(f"compute_fk failed ({res.error_code.val}); cannot lock EE orientation yet.")
        return None
    if not res.pose_stamped:
        logger.warn("compute_fk returned no poses.")
        return None

    logger.info("Captured initial EE orientation from live joint state (compute_fk).")
    return copy.deepcopy(res.pose_stamped[0].pose.orientation)


def goal_constraints_for_position(
    xyz: Tuple[float, float, float],
    planning_frame: str,
    eef_link: str,
    position_tol: float,
    ee_orientation: Quaternion,
    orient_tol: float,
) -> Constraints:
    """Build MoveIt goal Constraints for a Cartesian position with a fixed orientation."""
    x, y, z = xyz
    goal = Constraints()

    pos = PositionConstraint()
    pos.header.frame_id = planning_frame
    pos.link_name = eef_link
    pos.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)

    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [float(max(position_tol, 0.001))]

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
    ori.header.frame_id = planning_frame
    ori.link_name = eef_link
    ori.orientation = copy.deepcopy(ee_orientation)
    ori.absolute_x_axis_tolerance = float(orient_tol)
    ori.absolute_y_axis_tolerance = float(orient_tol)
    ori.absolute_z_axis_tolerance = float(orient_tol)
    ori.parameterization = OrientationConstraint.XYZ_EULER_ANGLES
    ori.weight = 1.0

    goal.position_constraints.append(pos)
    goal.orientation_constraints.append(ori)
    return goal


def workspace_parameters(planning_frame: str) -> WorkspaceParameters:
    """Return a ±2 m bounding box centred on planning_frame to focus OMPL sampling."""
    wp = WorkspaceParameters()
    wp.header.frame_id = planning_frame
    wp.min_corner.x = -2.0
    wp.min_corner.y = -2.0
    wp.min_corner.z = -2.0
    wp.max_corner.x = 2.0
    wp.max_corner.y = 2.0
    wp.max_corner.z = 2.0
    return wp
