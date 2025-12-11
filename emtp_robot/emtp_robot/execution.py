#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from psi_interfaces.msg import TaskRequest, Cancel, FleetState, RobotState, Location

import math

class Execution(Node):
    def __init__(self):
        super().__init__("execution_node")

        self.robot_key = "R1"
        self.robot_name = "UGV1"
        self.waypoints = {
            "L1_delivery_charger1": (0.0, 0.0),
            "L1_delivery_charger2": (1.0, 0.0),
            "L1_delivery_charger3": (2.0, 0.0),
            "H4_fireextinguisher1": (135.30, -93.06)
        }

        # 현재 상태
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_level = "L1"

        self.current_task = ""
        self.target_pos = None

        # Sub/Pub 설정
        self.create_subscription(TaskRequest, "/task_pub", self.task_callback, 10)
        self.create_subscription(Cancel, "/cancel_robot", self.cancel_callback, 10)

        self.state_pub = self.create_publisher(FleetState, "/fleet_states", 10)

        # 주기 상태 발행
        self.create_timer(0.2, self.publish_state)

        self.get_logger().info(f"[{self.robot_name}] Ready")

    # -------------------------------------------
    # Task 수신 → 로봇이 내부 목적지 선택
    # -------------------------------------------
    def task_callback(self, msg: TaskRequest):
        if msg.robot_name != self.robot_name:
            return

        task_id = msg.task_id
        self.get_logger().info(f"[TASK] {task_id} received")

        if task_id in self.waypoints:
            self.current_task = task_id
            self.target_pos = self.waypoints[task_id]
        else:
            self.get_logger().warn(f"[{self.robot_name}] Unknown waypoint: {task_id}")
            return

    # -------------------------------------------
    # Cancel → 즉시 멈춤
    # -------------------------------------------
    def cancel_callback(self, msg: Cancel):
        if msg.cancel not in ["all", self.robot_key, self.robot_name]:
            return

        self.get_logger().warn(f"[{self.robot_name}] CANCEL received!")
        self.current_task = ""
        self.target_pos = None

    # -------------------------------------------
    # 아주 단순한 이동 시뮬레이션
    # 실제 리모/터틀봇 제어 코드로 대체하면 됨
    # -------------------------------------------
    def update_motion(self):
        if not self.target_pos:
            return

        tx, ty = self.target_pos
        dx = tx - self.pose_x
        dy = ty - self.pose_y
        dist = math.hypot(dx, dy)

        # 도착 판정
        if dist < 0.2:
            self.get_logger().info(f"[ARRIVED] {self.current_task} completed")
            self.current_task = ""
            self.target_pos = None
            return

        # 임시로 직진 이동 (여기를 리모/터틀봇 실제 control로 교체)
        step = 0.1
        self.pose_x += (dx / dist) * step
        self.pose_y += (dy / dist) * step

    # -------------------------------------------
    # 서버와 완전히 호환되는 FleetState publish
    # -------------------------------------------
    def publish_state(self):
        self.update_motion()

        msg = FleetState()
        rs = RobotState()
        rs.name = self.robot_name

        loc = Location()
        loc.x = float(self.pose_x)
        loc.y = float(self.pose_y)
        loc.level_name = self.pose_level
        loc.timestamp = int(self.get_clock().now().nanoseconds)
        rs.location = loc

        rs.task_id = self.current_task

        msg.robots.append(rs)
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Execution()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
