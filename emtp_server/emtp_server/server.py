#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy, json, time, os
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
from rclpy.node import Node
from psi_interfaces.srv import AllocationResult, DynamicEvent
from psi_interfaces.msg import FleetState,TaskRequest,Cancel,TaskComplete
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
import math
import yaml
import sys

class AllocationServer(Node):
    def __init__(self):
        super().__init__('allocation_server')
        # === log === 
        self.declare_parameter('algo', 'EMTP')
        self.declare_parameter('environment', 'college')
        self.declare_parameter('timestamp', '')
        timestamp_param = self.get_parameter('timestamp').get_parameter_value().string_value
        if timestamp_param:
            self.timestamp = timestamp_param
        else:
            self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.algo = self.get_parameter('algo').get_parameter_value().string_value
        self.environment = self.get_parameter('environment').get_parameter_value().string_value
        self.pkg_root = os.path.expanduser("~/emtp_ws/src/emtp_server/")
        result_dir = os.path.join(
            self.pkg_root,
            "result",
            f"experiment_{self.timestamp}"
        )
        os.makedirs(result_dir, exist_ok=True)
        
        self.is_running = True
        self.total_triggers = 6   # 5개 트리거+초기계획
        self.waypoint_coords = self.load_nav_graph_data()
        
        # === 서비스 및 퍼블리셔 생성 ===
        self.srv = self.create_service(AllocationResult, '/allocation', self.handle_allocation)
        self.event_cli = self.create_client(DynamicEvent, '/event')
        self.task_pub = self.create_publisher(TaskRequest, '/task_pub', 10)
        self.complete_pub = self.create_publisher(String, '/complete_tasks', 10)
        self.cancel_pub = self.create_publisher(Cancel, '/cancel_robot', 10)
        self.create_subscription(String, '/dynamic_event', self.dynamic_event_callback, 10)
        self.create_subscription(FleetState, '/fleet_states', self.fleet_state_callback, 10)
        self.path_matrix = self.load_path_matrix()
        # === 로봇 상태 관리 ===
        self.robot_map = {
            "R1": "robot1",
            "R2": "robot2",
            "R3": "robot3",
        }
        self.robot_tasks = {}
        self.active_tasks = {}
        self.complete_tasks = {key: [] for key in self.robot_map.keys()}
        self.robots_pos = {}

        # === 시간 및 로그 ===
        self.start_time = None
        self.plan_start_time = None
        self.plan_time_total = 0.0
        self.plan_end_time = None
        self.event_count = 0
        self.log_path = os.path.join(
            result_dir,
            f"server_experiment_{self.timestamp}.json"
        )
        with open(self.log_path, 'w', encoding='utf-8') as f:
            json.dump({}, f, indent=2, ensure_ascii=False)
        self.latest_fleet_state = None
        self.fleet_timer = self.create_timer(0.5, self.process_fleet_state)
        print(f"[INIT] Allocation Server Ready")
        self.create_subscription(
            TaskComplete,
            "/task_complete",
            self.task_complete_callback,
            10
        )
        self.task_wp_pub = self.create_publisher(
            MarkerArray,
            '/task_wp',
            1
        )
        self.task_2F_wp_pub = self.create_publisher(
            MarkerArray,
            '/task_2F_wp',
            1
        )
        self.task_3F_wp_pub = self.create_publisher(
            MarkerArray,
            '/task_3F_wp',
            1
        )
        self.wp_timer = self.create_timer(1.0, self.publish_waypoints)
        self.publish_complete_tasks()
    def task_complete_callback(self, msg: TaskComplete):
        key = msg.robot_key
        reported = list(msg.completed_tasks)
        self.complete_tasks[key] = reported

        # active task 정리 + 다음 task 할당
        if key in self.active_tasks:
            current = self.active_tasks[key]["current"]
            if current in reported:
                self.active_tasks.pop(key, None)

                if self.robot_tasks.get(key):
                    next_task = self.robot_tasks[key].pop(0)
                    self.send_task(self.robot_map[key], next_task)
                    self.active_tasks[key] = {
                        "current": next_task,
                        "status": "waiting"
                    }

        self.publish_complete_tasks()
        self.auto_end_check()
    def publish_waypoints(self):
        task_wp = MarkerArray()
        task_2F_wp = MarkerArray()
        task_3F_wp = MarkerArray()
        now = self.get_clock().now().to_msg()

        for idx, (name, info) in enumerate(self.waypoint_coords.items()):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = now
            m.ns = "waypoints"
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = info["x"]
            m.pose.position.y = info["y"]
            m.scale.x = 1.0
            m.scale.y = 1.0
            m.scale.z = 1.0

            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0

            if info["level"] == "L1":
                m.pose.position.z = 0.0
                task_2F_wp.markers.append(m)
            else:
                m.pose.position.z = 3.0
                task_3F_wp.markers.append(m)
            task_wp.markers.append(m)

        self.task_wp_pub.publish(task_wp)
        self.task_2F_wp_pub.publish(task_2F_wp)
        self.task_3F_wp_pub.publish(task_3F_wp)

    def load_path_matrix(self):
    # 설치된 패키지의 share 경로 가져오기
        package_share = get_package_share_directory('emtp_server')
        file_path = os.path.join(package_share, 'env', 'experiment', 'path_matrix.csv')

        if not os.path.exists(file_path):
            self.get_logger().error(f"Path matrix file not found: {file_path}")
            return {}

        matrix = {}
        import csv
        with open(file_path, "r", encoding="utf-8") as f:
            reader = csv.reader(f)
            header = next(reader)[1:]
            for row in reader:
                wp = row[0]
                matrix[wp] = {}
                for i, dist in enumerate(row[1:]):
                    if dist.strip() == "":
                        matrix[wp][header[i]] = None
                    else:
                        matrix[wp][header[i]] = float(dist)
        return matrix
    
    def compute_logical_distance(self, robot_key):
        tasks = self.complete_tasks[robot_key]
        if not tasks:
            return 0.0

        # 시작지점 매핑
        start_wp = {
            "R1": "L1_delivery_charger1",
            "R2": "L1_delivery_charger2",
            "R3": "L1_delivery_charger3"
        }[robot_key]

        total = 0.0
        prev = start_wp

        for t in tasks:
            d = self.path_matrix.get(prev, {}).get(t)
            if d is None:
                print(f"[WARN] No matrix distance: {prev} → {t}")
            else:
                total += d
            prev = t

        return round(total, 2)
    
    def load_nav_graph_data(self):
        package_share = get_package_share_directory('emtp_server')
        graph_file = os.path.join(package_share, 'env', 'experiment', 'wp_list.yaml')

        data = {}

        with open(graph_file, "r", encoding="utf-8") as f:
            yaml_data = yaml.safe_load(f)

        for name, info in yaml_data["waypoints"].items():
            data[name] = {
                "x": float(info["x"]),
                "y": float(info["y"]),
                "level": info["level"],
            }

        return data
    
    def find_nearest_waypoint(self, x, y, level):
        best = None
        best_dist = 999999
        for wp, info in self.waypoint_coords.items():
            if info["level"] != level:
                continue
            dx = x - info["x"]
            dy = y - info["y"]
            d = math.hypot(dx, dy)
            if d < best_dist:
                best = wp
                best_dist = d
        return best

    def get_waypoint_location(self, waypoint_name):
        return self.waypoint_coords.get(waypoint_name)
    
    # ---------------- Allocation ----------------
    def handle_allocation(self, request, response):
        try:
            if self.start_time is None:
                self.start_time = time.time()
                print(f"[START] Simulation started at {datetime.now().strftime('%H:%M:%S')}")

            if self.plan_start_time is not None:
                self.plan_end_time = time.time()
                plan_duration = self.plan_end_time - self.plan_start_time
                self.plan_time_total += plan_duration

            def is_valid_path(seq):
                if not seq:
                    return False
                
                for s in seq:
                    if s is None:
                        return False
                    if not isinstance(s, str):
                        return False
                    if s.strip() == "":
                        return False
                
                return True
            
            self.robot_tasks = {rp.robot_name: list(rp.path_sequence) for rp in request.paths if is_valid_path(rp.path_sequence)}

            print("=== Allocation Received ===")
            for robot, path in self.robot_tasks.items():
                print(f"{robot}: {' -> '.join(path)}")

            import copy
            content = getattr(self, "last_dynamic", "initial_plan")
            self.pending_event_data = {
                "content": content,
                "plan": copy.deepcopy(self.robot_tasks),        # pop 되기 전 순수 경로
                "complete_tasks": copy.deepcopy(self.complete_tasks)
            }

            self.init_tasks()
            response.success = True
            self.dynamic_handled = False

            if self.event_count == 0:
                self.event_count += 1

            self.log_simulation_end()
        except Exception as e:
            print(f"[ERROR] handle_allocation: {e}")
            response.success = False
            
        return response

    # ---------------- 첫번째 task 할당 ----------------
    def init_tasks(self):
        for robot_key, task_list in self.robot_tasks.items():
            if not task_list:
                continue
            if robot_key in self.active_tasks:
                continue
            if task_list:
                next_task = task_list.pop(0)
                robot_name = self.robot_map[robot_key]
                self.send_task(robot_name, next_task)
                self.active_tasks[robot_key] = {"current": next_task, "id": None, "status": "waiting"}
        self.print_active_tasks()

    # ---------------- Send Task ----------------
    def send_task(self, robot, waypoint):
        msg = TaskRequest()
        msg.robot_name = robot       # ex) "deliveryBot1"
        msg.task_id = waypoint       # ex) "L1_delivery_charger1" 같은 waypoint 이름
        self.task_pub.publish(msg)

    # ---------------- Fleet Callback ----------------
    def fleet_state_callback(self, msg: FleetState):
        self.latest_fleet_state = msg

    def process_fleet_state(self):
        if getattr(self, "dynamic_handled", True):
            return
        if self.latest_fleet_state is None:
            return

        fleet_state = self.latest_fleet_state
        for robot_state in fleet_state.robots:
            robot_name = robot_state.name
            key = next((k for k, v in self.robot_map.items() if v == robot_name), None)

            if not key:
                continue
            
            pos_wp = self.find_nearest_waypoint(
                robot_state.location.x,
                robot_state.location.y,
                robot_state.location.level_name
            )
            if pos_wp:
                self.robots_pos[key] = pos_wp

            task_info = self.active_tasks.get(key)
            if not task_info:
                continue

            current_id = robot_state.task_id
            status = task_info["status"]
            is_empty = (not current_id)

            if status == "waiting" and is_empty:
                if not task_info.get("resent", False):
                    self.send_task(self.robot_map[key], task_info["current"])
                    task_info["resent"] = True
                continue

            if status == "waiting" and not is_empty:
                print(f"[ACCEPT] {robot_name} accepted task. Running now.")
                task_info["status"] = "running"
                task_info["id"] = current_id
                task_info.pop("resent", None)
                continue

            if status == "running"  and not is_empty:
                task_info["id"] = current_id
                continue  

        self.print_active_tasks()
        self.auto_end_check()

    def auto_end_check(self):
        if getattr(self, "dynamic_handled", True):
            return
        if any(info["status"] == "waiting" for info in self.active_tasks.values()):
            return
        # 모든 task가 비었는가?
        all_done = all(len(tasks) == 0 for tasks in self.robot_tasks.values()) \
                   and len(self.active_tasks) == 0

        # 모든 trigger 이벤트 처리 되었는가?
        all_triggered = (self.event_count >= self.total_triggers)

        if all_done and (all_triggered or not self.dynamic_handled):

            # Supervisor에 END 전송
            if self.event_cli.wait_for_service(timeout_sec=2.0):
                req = DynamicEvent.Request()
                req.dynamic = json.dumps({"dynamic": "end"}, ensure_ascii=False)
                self.event_cli.call_async(req)
                time.sleep(0.3)

            # 로그 저장
            self.log_simulation_end(event="end")
            
            self.is_running = False
            self.get_logger().info("[AUTO END] Simulation ended automatically.")
            # 서버 종료
            os.system("touch sim_done.flag")
    
    # ---------------- Publish Completed ----------------
    def publish_complete_tasks(self):
        msg = String()
        msg.data = json.dumps(self.complete_tasks, ensure_ascii=False)
        self.complete_pub.publish(msg)

    # ---------------- Print Active ----------------
    def print_active_tasks(self):
        print("----- Active Tasks (DEBUG) -----")
        for k in self.robot_map.keys():
            active = self.active_tasks.get(k)

            if active:
                status = active.get("status", "None")
                task = active.get("current")
                tid = active.get("id")
                print(f"[{k}] task={task}, status={status}, id={tid}")
            else:
                print(f"[{k}] task=None")

        print("-------------------------------")

    
    # ---------------- Log ----------------
    def log_simulation_end(self, event=None):
        end_time = time.time()
        if self.start_time is None:
            print("[WARN] No simulation started, skipping log.")
            return
        total_time = end_time - self.start_time
        plan_time = self.plan_time_total
        exec_time = total_time - plan_time

        
        real_path_length = {
            robot: self.compute_logical_distance(robot)
            for robot in self.robot_map.keys()
        }
        total_completed = sum(len(v) for v in self.complete_tasks.values())
        try:
            if os.path.exists(self.log_path):
                with open(self.log_path, 'r', encoding='utf-8') as f:
                    try:
                        log_data = json.load(f)
                    except json.JSONDecodeError:
                        log_data = {}
            else:
                log_data = {}

             
            if "event_results" not in log_data:
                log_data["event_results"] = {}

            if event is None:
                event_key = str(self.event_count)
            else:
                event_key = event  # "end"

            if event == "end":
                log_data["event_results"][event_key] = {
                    "content": "end",
                    "complete_tasks": self.complete_tasks
                }
            else:
                log_data["event_results"][event_key] = self.pending_event_data
            log_data["total_completed_tasks"] = total_completed
            log_data["real_path_length"] = real_path_length
            log_data["time"] = {
                "total_time": round(total_time, 2),
                "plan_time": round(plan_time, 2),
                "exec_time": round(exec_time, 2)
            }
            with open(self.log_path, 'w', encoding='utf-8') as f:
                json.dump(log_data, f, indent=2, ensure_ascii=False)

            print(f"[LOG] Event {self.event_count} recorded → {self.log_path}")

        except Exception as e:
            print(f"[ERROR] Writing log file: {e}")

    # ---------------- Dynamic Event Callback ----------------
    def dynamic_event_callback(self, msg: String):
        dynamic_str = msg.data.strip()
        if dynamic_str == "" or dynamic_str is None:
            print("[SAFE] Empty or invalid dynamic event → ignored")
            return

        if dynamic_str.lower() == "end":
            print("[END] End event received. Logging and shutting down...")

            if self.event_cli.wait_for_service(timeout_sec=2.0):
                req = DynamicEvent.Request()
                req.dynamic = json.dumps({"dynamic": "end"}, ensure_ascii=False)
                self.event_cli.call_async(req)
                time.sleep(0.3)

            self.log_simulation_end(event="end")

            self.is_running = False
            return
        else:
            if dynamic_str.lower() != "end":
                self.event_count += 1
            

        if hasattr(self, "last_dynamic") and self.last_dynamic == dynamic_str:
            print(f"[SKIP] Duplicate dynamic event ignored → {dynamic_str}")
            return
        self.last_dynamic = dynamic_str

        if getattr(self, "dynamic_handled", False):
            print(f"[SKIP] Already handling dynamic → {dynamic_str}")
            return
        
        
        
        
        self.dynamic_handled = True
        print(f"[EVENT] Dynamic received: {dynamic_str}")
        

        cancel_msg = Cancel()
        cancel_msg.cancel = "all"
        self.cancel_pub.publish(cancel_msg)
        print("[CANCEL] Sent cancel signal for all robots.")

        

        req = DynamicEvent.Request()
        payload = {
            "dynamic": dynamic_str,
            "robots_pos": self.robots_pos,
            "complete_tasks": self.complete_tasks,
        }
        self.plan_end_time = None
        self.plan_start_time = time.time()
        req.dynamic = json.dumps(payload, ensure_ascii=False)
        print(f"req.dynamic: {req.dynamic}")
        if not self.event_cli.wait_for_service(timeout_sec=3.0):
            print("[ERROR] Supervisor /event not available")
            return
        self.event_cli.call_async(req)

        self.robot_tasks.clear()
        self.active_tasks.clear()



def main(args=None):
    rclpy.init(args=args)
    node = AllocationServer()
    from rclpy.executors import MultiThreadedExecutor

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


if __name__ == '__main__':
    main()
