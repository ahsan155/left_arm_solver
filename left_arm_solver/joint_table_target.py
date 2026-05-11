# Unitree /arm_joint_cmd expects exactly these 17 joints in this order.
# /joint_states names before command joints from the recorded bag.
JOINT_STATES_PREFIX_JOINTS = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
]

UNITREE_CMD_JOINTS = [
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

# SRDF ``left_arm`` chain: torso_link -> left_wrist_yaw_link (same 7 DOF as MoveIt group).
# Simulation stacks often publish only these arm joints on ``/joint_states``, without waist or legs.
LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]


# /joint_states names after command joints from the recorded bag.
JOINT_STATES_SUFFIX_JOINTS = [
    "right_hand_thumb_0_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
]

# Full /joint_states order from the provided bag data.
ALL_JOINT_STATES_JOINTS = (
    JOINT_STATES_PREFIX_JOINTS + UNITREE_CMD_JOINTS
)

# Index-aligned limits for /arm_joint_cmd values.
UNITREE_LIMITS = [
    (-2.5, 2.5),
    (-0.52, 0.52),
    (-0.52, 0.52),
    (-2.9, 2.5),
    (-1.4, 2.1),
    (-2.5, 2.5),
    (-0.9, 2.0),
    (-1.8, 1.8),
    (-1.614, 1.614),
    (-1.614, 1.614),
    (-2.9, 2.5),
    (-2.1, 1.4),
    (-2.5, 2.5),
    (-0.9, 2.0),
    (-1.8, 1.8),
    (-1.614, 1.614),
    (-1.614, 1.614),
]

# end_effector(left_wrist_yaw_link) position in cartesian coordinates if end_effector angle is 0
base_x = 0.199774285
base_y = 0.148661704
base_z = 0.0952328317

# target waypoints(cartesian coordinates) for end effector(left_wrist_yaw_link)
target_waypoints = [
    (0.3, base_y, 0.05),
    (0.3, base_y, 0.15),
    (0.3, base_y, 0.25),
    (0.3, base_y, 0.35),
    (0.3, base_y, 0.45),
    (0.3, 0.16, 0.05),
    (0.3, 0.20, 0.05),
    (0.3, 0.24, 0.05),
    (0.3, 0.28, 0.05),
    (0.3, 0.32, 0.05),
]

# How far can the end effector move in each direction(cartesian coordinates)?
# x = 0.34 forward-most, x = 0.19 initial position
# y = 0.41 left-most, y = 0.14 right-most
# z = 0.001 lowest, z = 0.2 highest
