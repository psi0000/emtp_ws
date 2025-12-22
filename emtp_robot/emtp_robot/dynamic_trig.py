#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rmf_fleet_msgs.msg import FleetState
import math


# =============================================
# Geometry Utils — 점→선분 거리
# =============================================
def point_to_segment_dist(px, py, x1, y1, x2, y2):
    A = (px - x1, py - y1)
    B = (x2 - x1, y2 - y1)
    B_len2 = B[0]**2 + B[1]**2

    if B_len2 == 0:
        return math.hypot(px - x1, py - y1)

    t = max(0, min(1, (A[0]*B[0] + A[1]*B[1]) / B_len2))
    proj_x = x1 + B[0] * t
    proj_y = y1 + B[1] * t
    return math.hypot(px - proj_x, py - proj_y)


# =============================================
# Trigger Node
# =============================================
class TriggerZone(Node):
    def __init__(self):
        super().__init__('dynamic_trig')

        self.declare_parameter("environment", "college")
        self.env = self.get_parameter("environment").get_parameter_value().string_value

        self.get_logger().info(f"[TriggerZone] environment = {self.env}")

        # 트리거 로드
        self.triggers = self.load_triggers(self.env)
        self.triggered_ids = set()

        self.create_subscription(FleetState, "/fleet_states", self.fleet_callback, 10)
        self.pub = self.create_publisher(String, "/dynamic_event", 10)

        self.get_logger().info("[TriggerZone] Ready with circle + line triggers")

    def load_triggers(self, env):

        # -------------------------
        # warehouse
        # -------------------------
        if env == "warehouse":
            self.get_logger().info("[TriggerZone] Loading warehouse triggers")

            return [
                {
                    "id": 1,
                    "type": "circle",
                    "level": "L1",
                    "x": 89.625,
                    "y": -85.95,
                    "radius": 2,
                    "message": "Don't enter C6, because staff is relaxing there."
                },
                {
                    "id": 2,
                    "type": "multi_circle",
                    "level": "L1",
                    "circles": [
                        {"x": 73.1250, "y": -82.8750, "radius": 2.0},
                        {"x": 73.1250, "y": -88.0500, "radius": 2.0},
                        # {"x": 70.0500, "y": -82.8000, "radius": 2.0},
                        {"x": 69.5250, "y": -77.4750, "radius": 2.0},
                        {"x": 67.5750, "y": -81.9000, "radius": 2.0},
                        {"x": 67.9500, "y": -86.3250, "radius": 2.0},
                        {"x": 70.9500, "y": -87.9750, "radius": 2.0},

                    ],
                    "message": "D1과 D2 사이에 UAV만 출입 가능합니다."
                },
                {
                    "id": 3,
                    "type": "line",
                    "level": "L1",
                    "x1": 59.3250, "y1": -92.7000,
                    "x2": 119.0250, "y2": -92.7000,
                    "threshold": 2.0,
                    "message": "Don't enter the cold storage area"
                },
                {
                    "id": 4,
                    "type": "line",
                    "level": "L1",
                    "x1": 110.3250, "y1": -4.5399,
                    "x2": 110.3250, "y2": -70.2073,
                    "threshold": 2.0,
                    "message": "Can you also inspect the firehydrants in the hallways?"
                },
                {
                    "id": 5,
                    "type": "line",
                    "level": "L1",
                    "x1": 127.65, "y1": -85.20,
                    "x2": 127.65, "y2": -92.70,
                    "threshold": 1.0,
                    "message": "Inspect fire extinguishers in fast-moving stock first"
                },
                {
                "id": 6,
                "type": "circle",
                "level": "L1",
                "x": 70.2000,
                "y": -36.1500,
                "radius": 2.0,
                "message": "Robot R2 malfunctioned."
                }
            ]

        # -------------------------
        # ✅ college (원본 그대로 유지)
        # -------------------------
        self.get_logger().info("[TriggerZone] Loading college triggers")

        return [
            {
                "id": 1,
                "type": "circle",
                "level": "L1",
                "x": 96.4459,
                "y": -104.3249,
                "radius": 3.0,
                "message": "The B5 lab is running experiments, entry is not allowed!"
            },
            {
                "id": 2,
                "type": "circle",
                "level": "L2",
                "x": 69.9988,
                "y": -98.8992,
                "radius": 1.0,
                "message": "The 2nd-floor meeting room (C1) is in use, entry is not allowed!"
            },
            {
                "id": 3,
                "type": "circle",
                "level": "L2",
                "x": 95.0403,
                "y": -87.3872,
                "radius": 2.0,
                "message": "Robot R2 malfunctioned."
            },
            {
                "id": 4,
                "type": "circle",
                "level": "L1",
                "x": 95.5916,
                "y": -86.8583,
                "radius": 2.0,
                "message": "UGVs are not allowed to enter the 1st-floor seminar room."
            },
            {
                "id": 5,
                "type": "circle",
                "level": "L1",
                "x": 37.6861,
                "y": -101.0974,
                "radius": 2.0,
                "message": "Inspect firehydrants before fire extinguishers."
            }
        ]

    # -----------------------------------------------------
    # Trigger check
    # -----------------------------------------------------
    def fleet_callback(self, msg: FleetState):

        for robot in msg.robots:
            robot_name = robot.name
            level = robot.location.level_name
            x = robot.location.x
            y = robot.location.y

            for trig in self.triggers:
                if trig["id"] in self.triggered_ids:
                    continue

                if level != trig["level"]:
                    continue

                t = trig.get("type", "circle")

                if t == "circle":
                    dist = math.hypot(x - trig["x"], y - trig["y"])
                    if dist <= trig["radius"]:
                        self.activate_trigger(robot_name, trig)

                elif t == "line":
                    d = point_to_segment_dist(
                        x, y,
                        trig["x1"], trig["y1"],
                        trig["x2"], trig["y2"]
                    )
                    if d <= trig["threshold"]:
                        self.activate_trigger(robot_name, trig)
                elif t == "multi_circle":
                    for c in trig["circles"]:
                        dist = math.hypot(x - c["x"], y - c["y"])
                        if dist <= c["radius"]:
                            self.activate_trigger(robot_name, trig)
                            break
    # -----------------------------------------------------
    # Activate
    # -----------------------------------------------------
    def activate_trigger(self, robot_name, trig):
        self.triggered_ids.add(trig["id"])

        msg = String()
        msg.data = trig["message"]
        self.pub.publish(msg)

        self.get_logger().info(
            f"[TRIGGER {trig['id']}] Robot {robot_name} → {trig['message']}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TriggerZone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
