#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from psi_interfaces.msg import TaskRequest, Cancel, FleetState, RobotState, Location, TaskComplete
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy 
import math
import os
import yaml
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class Execution(Node):
    def __init__(self, robot_id: int):
        super().__init__("execution_node")

        self.robot_id = robot_id
        self.robot_key = f"R{robot_id}"
        self.robot_name = f"robot{self.robot_id}"
        self.LEVEL_Z = {
            "L1": 0.0,
            "L2": 3.0
        }

        self.waypoints = self.load_waypoints()
        self.triggers = self.load_triggers()

        self._goal_handle = None
        self.triggered_ids = set()
        self.pose_x = self.pose_y = self.pose_z = 0.0
        self.pose_level = "L1"
	
        self.current_task = ""
        self.target_pos = None
        self.nav_goal_active = False
        self.is_cancelled = False
        self.pending_tasks = []
        self.completed_tasks = []

        # --- Nav2 Action Client ---
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            f'/{self.robot_name}/navigate_to_pose'
        )

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(
            Twist, f"/{self.robot_name}/cmd_vel", 10
        )
        self.complete_pub = self.create_publisher(
            TaskComplete, "/task_complete", 10
        )
        self.state_pub = self.create_publisher(
            FleetState, "/fleet_states", 10
        )
        self.dynamic_pub = self.create_publisher(
            String, "/dynamic_event", 10
        )
        self.trig_marker_pub = self.create_publisher(
            MarkerArray, "/trigger_markers", 1
        )
        self.target_pub = self.create_publisher(
            Point, f"/{self.robot_name}/current_target",10
        )
        # --- Subscribers ---
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(
            Odometry, f"/{self.robot_name}/odom",
            self.odom_callback, odom_qos
        )
        self.create_subscription(
            TaskRequest, "/task_pub",
            self.task_callback, 10
        )
        self.create_subscription(
            Cancel, "/cancel_robot",
            self.cancel_callback, 10
        )

        # --- Timers ---
        self.create_timer(0.1, self.publish_state)
        self.create_timer(1.0, self.print_current_state)
        self.create_timer(1.0, self.publish_trigger_markers)
        self.get_logger().info(f"[{self.robot_name}] Ready")
    
    def publish_trigger_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, trig in enumerate(self.triggers):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = now
            m.ns = "triggers"
            m.id = i
            m.action = Marker.ADD

            if trig["type"] == "circle":
                m.type = Marker.CYLINDER
                m.pose.position.x = trig["x"]
                m.pose.position.y = trig["y"]
                m.pose.position.z = self.LEVEL_Z[trig["level"]]
                m.scale.x = trig["radius"] * 2
                m.scale.y = trig["radius"] * 2
                m.scale.z = 0.1
                m.color.b = 1.0
                m.color.a = 0.3

            elif trig["type"] == "line":   # 이름은 유지해도 되고, "rect"로 바꿔도 됨
                m.type = Marker.CUBE
                m.pose.position.x = trig["x"]
                m.pose.position.y = trig["y"]
                m.pose.position.z = self.LEVEL_Z[trig["level"]]

                # 네모 크기 (threshold 기준)
                m.scale.x = trig["threshold"] * 2
                m.scale.y = trig["threshold"] * 2
                m.scale.z = 1.2   

                # 색상
                m.color.b = 1.0
                m.color.a = 0.3

                m.pose.orientation.x = 0.0
                m.pose.orientation.y = 0.0
                m.pose.orientation.z = 0.0
                m.pose.orientation.w = 1.0

            ma.markers.append(m)

        self.trig_marker_pub.publish(ma)

    def load_waypoints(self):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        wp_path = os.path.join(base_dir, "wp_list.yaml")

        with open(wp_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        waypoints = {}
        for name, info in data["waypoints"].items():
            level = info["level"]
            z = self.LEVEL_Z.get(level, 0.0)

            waypoints[name] = (
                float(info["x"]),
                float(info["y"]),
                float(z)
            )
        return waypoints
    def load_triggers(self):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        trig_path = os.path.join(base_dir, "trig.yaml")

        with open(trig_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        return data.get("triggers", [])
    def check_triggers(self):
        for trig in self.triggers:
            if trig["id"] in self.triggered_ids:
                continue
            if self.pose_level != trig["level"]:
                continue

            ttype = trig.get("type", "circle")

            # --- circle trigger ---
            if ttype == "circle":
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

            # --- line trigger ---
            elif ttype == "line":
                x1, y1 = trig["x1"], trig["y1"]
                x2, y2 = trig["x2"], trig["y2"]

                # 점–선분 거리 (inline, 함수 안 만듦)
                ax = self.pose_x - x1
                ay = self.pose_y - y1
                bx = x2 - x1
                by = y2 - y1
                blen2 = bx*bx + by*by

                if blen2 == 0.0:
                    d = math.hypot(self.pose_x - x1, self.pose_y - y1)
                else:
                    t = max(0.0, min(1.0, (ax*bx + ay*by) / blen2))
                    proj_x = x1 + bx * t
                    proj_y = y1 + by * t
                    d = math.hypot(self.pose_x - proj_x, self.pose_y - proj_y)

                if d <= trig["threshold"]:
                    self.triggered_ids.add(trig["id"])
                    msg = String()
                    msg.data = trig["message"]
                    self.dynamic_pub.publish(msg)
                    self.get_logger().warn(
                        f"[TRIGGER {trig['id']}] {trig['message']}"
                    )

    def send_nav2_goal(self, x, y, z):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        # 로봇 개만
        # if self.robot_id == 3:
        #     goal.pose.pose.position.z = float(z)
        goal.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.nav_goal_response_cb)
        
    def nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self.nav_goal_active = False
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_cb)

    def nav_result_cb(self, future):
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f"[ARRIVED] {self.current_task}")
            if self.current_task not in self.completed_tasks:
                self.completed_tasks.append(self.current_task) # prohibit duplication

            msg = TaskComplete()
            msg.robot_key = self.robot_key
            msg.completed_tasks = self.completed_tasks[:]
            self.complete_pub.publish(msg)

            self.current_task = ""
            self.nav_goal_active = False
            self.start_next_task()

    def odom_callback(self, msg: Odometry):
        
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.pose_z = msg.pose.pose.position.z
        self.pose_level = "L2" if self.pose_z >= self.LEVEL_Z["L2"] else "L1"

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
        p = Point()
        p.x = float(self.target_pos[0])
        p.y = float(self.target_pos[1])
        p.z = float(self.target_pos[2])
        self.target_pub.publish(p)

        self.get_logger().info(f"[START] {self.current_task}")
        self.send_nav2_goal(
            self.target_pos[0],
            self.target_pos[1],
            self.target_pos[2]
        )
    
    def stop_and_hold_position(self):

        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self.cmd_pub.publish(Twist())
        self.nav_goal_active = False
        self.target_pos = None

    def cancel_callback(self, msg: Cancel):
        if msg.cancel not in ["all", self.robot_key, self.robot_name]:
            return
        self.get_logger().warn(f"[{self.robot_name}] CANCEL received")

        self.pending_tasks.clear()
        self.current_task = ""
        self.is_cancelled = True
        self.stop_and_hold_position()

    
    def publish_state(self):
        self.check_triggers()

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
            f"================ {self.robot_name.upper()} STATE ================\n"
            f" current_task: {self.current_task}\n"
            f" target_pos : {self.target_pos}\n"
            f" nav_active : {self.nav_goal_active}\n"
            f" pose       : x={self.pose_x:.2f}, y={self.pose_y:.2f}, z={self.pose_z:.2f}, level={self.pose_level}\n"
            f" completed_tasks: {self.completed_tasks}\n"
            "================================================"
        )

def main(args=None):

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=int, default=1, help="Robot ID (1, 2, 3, ...)")
    parsed_args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = Execution(robot_id=parsed_args.id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()