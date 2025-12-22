#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from psi_interfaces.msg import TaskRequest, Cancel, FleetState, RobotState, Location, TaskComplete
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy ,ReliabilityPolicy


class Execution(Node):
    def __init__(self):
        super().__init__("execution_node")

        self.robot_key = "R1"
        self.robot_name = "UGV1"

        self.waypoints = {
            "L1_delivery_charger1": (0.0, 0.0),
            "L1_delivery_charger2": (1.0, 0.0),
            "L1_delivery_charger3": (2.0, 0.0),
            "H4_fireextinguisher1": (135.30, -93.06),
            "WP1": (9.74, 2.66),
            "WP2": (11.839999999999998, 21.86),
        }
        self.triggers = [
            {
                "id": 1,
                "level": "L1",
                "x": 9.74,
                "y": 2.66,
                "radius": 2.0,
                "message": "Inspect firehydrants before fire extinguishers."
            },
            {
                "id": 2,
                "level": "L1",
                "x": 11.839999999999998,
                "y": 21.86,
                "radius": 2.0,
                "message": "UGVs are not allowed to enter the seminar room."
            }
        ]
        self.triggered_ids = set()

        # dynamic_event 퍼블리셔
        self.dynamic_pub = self.create_publisher(String, "/dynamic_event", 10)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0
        self.pose_level = "L1"
        self.arrival_threshold = 1.0
        self.current_task = ""
        self.target_pos = None
        self.nav_goal_active = False
        self.is_cancelled = False
        self.pending_tasks = []        # 대기 중 task 큐
        self.completed_tasks = []     # 완료된 task 기록
        self.dist = 0.0

        odom_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE,history=HistoryPolicy.KEEP_LAST,depth=10)
        goal_qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)

        self.goal_pub = self.create_publisher(PoseStamped,"/R1/goal_pose",goal_qos)

        self.cmd_pub = self.create_publisher(Twist,"/R1/cmd_vel",10)
        self.create_subscription(Odometry,"/R1/odom",self.odom_callback,odom_qos )
        self.complete_pub = self.create_publisher(TaskComplete,"/task_complete",10)
        self.create_subscription(TaskRequest, "/task_pub", self.task_callback, 10)
        self.create_subscription(Cancel, "/cancel_robot", self.cancel_callback, 10)
        self.state_pub = self.create_publisher(FleetState, "/fleet_states", 10)
        
        self.create_timer(0.1, self.publish_state)
        self.create_timer(0.1, self.print_current_state)
        self.get_logger().info(f"[{self.robot_name}] Ready")
    def check_triggers(self):
        import math

        for trig in self.triggers:
            if trig["id"] in self.triggered_ids:
                continue

            if self.pose_level != trig["level"]:
                continue

            dist = math.hypot(
                self.pose_x - trig["x"],
                self.pose_y - trig["y"]
            )

            if dist <= trig["radius"]:
                self.triggered_ids.add(trig["id"])

                msg = String()
                msg.data = trig["message"]
                self.dynamic_pub.publish(msg)

                self.get_logger().warn(
                    f"[TRIGGER {trig['id']}] {trig['message']}"
                )
    def publish_goal_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        self.goal_pub.publish(pose)

    def odom_callback(self, msg: Odometry):
        
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.pose_z = msg.pose.pose.position.z
        self.pose_level = "L2" if self.pose_z > 10.0 else "L1"

    def task_callback(self, msg: TaskRequest):
        if msg.robot_name != self.robot_name:
            return
        self.is_cancelled = False
        task_id = msg.task_id

        if task_id not in self.waypoints:
            self.get_logger().warn(f"Unknown waypoint: {task_id}")
            return

        self.pending_tasks.append(task_id)
        self.get_logger().info(f"[QUEUE] Added task: {task_id}")

        if not self.nav_goal_active:
            self.start_next_task()

    def start_next_task(self):
        if not self.pending_tasks:
            return

        self.current_task = self.pending_tasks.pop(0)
        self.target_pos = self.waypoints[self.current_task]
        self.nav_goal_active = True

        self.get_logger().info(f"[START] {self.current_task}")

        self.publish_goal_pose(self.target_pos[0],self.target_pos[1])

    def cancel_callback(self, msg: Cancel):
        if msg.cancel not in ["all", self.robot_key, self.robot_name]:
            return

        self.get_logger().warn(f"[{self.robot_name}] CANCEL received")

        
        self.cmd_pub.publish(Twist())
        self.pending_tasks.clear()
        self.current_task = ""
        self.target_pos = None
        self.nav_goal_active = False
        self.is_cancelled = True

    def check_arrived(self):
        import math
        if not self.nav_goal_active or self.target_pos is None:
            return False

        dx = self.pose_x - self.target_pos[0]
        dy = self.pose_y - self.target_pos[1]
        dist = math.hypot(dx, dy)
        self.dist=dist
        return dist < self.arrival_threshold
    
    def publish_state(self):
        # Nav2 완료 체크
        self.check_triggers()
        if self.check_arrived():
            self.get_logger().info(f"[ARRIVED] {self.current_task}")
            self.cmd_pub.publish(Twist())
            if self.current_task not in self.completed_tasks:
                self.completed_tasks.append(self.current_task)
            msg = TaskComplete()
            msg.robot_key = self.robot_key
            msg.completed_tasks = self.completed_tasks[:]
            self.complete_pub.publish(msg)

            # 상태 초기화
            self.current_task = ""
            self.target_pos = None
            self.nav_goal_active = False
            self.start_next_task()

        msg = FleetState()
        rs = RobotState()
        rs.name = self.robot_name
        rs.model = self.robot_key

        loc = Location()
        loc.x = float(self.pose_x)
        loc.y = float(self.pose_y)
        loc.level_name = self.pose_level
        loc.timestamp = int(self.get_clock().now().nanoseconds)

        rs.location = loc
        rs.task_id = self.current_task

        msg.robots.append(rs)
        self.state_pub.publish(msg)

    def print_current_state(self):
        self.get_logger().info(
            "\n"
            "================ CURRENT STATE ================\n"
            f" current_task: {self.current_task}\n"
            f" target_pos : {self.target_pos}\n"
            f" nav_active : {self.nav_goal_active}\n"
            f" pose       : x={self.pose_x:.2f}, y={self.pose_y:.2f}, z={self.pose_z:.2f}, level={self.pose_level}\n"
            f" distance   : {self.dist:.2f}\n"
            f" completed_tasks: {self.completed_tasks}\n"
            "================================================"
        )
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
