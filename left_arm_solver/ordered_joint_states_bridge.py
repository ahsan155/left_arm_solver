#!/usr/bin/env python3
"""Subscribe to partial /joint_states, republish with fixed joint order (matches real G1 bag layout)."""

from __future__ import annotations

import math
from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from left_arm_solver.joint_table_target import ALL_JOINT_STATES_JOINTS


class OrderedJointStatesBridge(Node):
    """Merge incoming positions into ALL_JOINT_STATES_JOINTS order for downstream MoveIt / TF."""

    def __init__(self) -> None:
        super().__init__("ordered_joint_states_bridge")

        self.declare_parameter("input_topic", "/joint_states")
        self.declare_parameter("output_topic", "/joint_states_ordered")
        self.declare_parameter("default_position_if_missing", 0.0)
        self.declare_parameter(
            "joint_name_order",
            [],
        )

        in_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._default_pos = (
            self.get_parameter("default_position_if_missing").get_parameter_value().double_value
        )
        order = self.get_parameter("joint_name_order").get_parameter_value().string_array_value
        self._order: List[str] = list(order) if order else list(ALL_JOINT_STATES_JOINTS)

        self._positions: Dict[str, float] = {}

        self._pub = self.create_publisher(JointState, out_topic, 10)
        self.create_subscription(JointState, in_topic, self._on_js, 10)

        self.get_logger().info(
            f"Bridging {in_topic} -> {out_topic} ({len(self._order)} joints in fixed order)"
        )

    def _on_js(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            v = float(pos)
            if math.isnan(v) or math.isinf(v):
                v = float(self._default_pos)
            self._positions[name] = v

        out = JointState()
        out.header = msg.header
        out.name = list(self._order)
        out.position = [self._positions.get(j, self._default_pos) for j in self._order]
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = OrderedJointStatesBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
