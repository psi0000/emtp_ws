#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy, json, time, os
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
from rclpy.node import Node
from psi_interfaces.srv import AllocationResult, DynamicEvent
from psi_interfaces.msg import FleetState,TaskRequest,Cancel

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
            "R1": "UGV1",
            "R2": "UGV2",
            "R3": "DOG1",
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
        self.publish_complete_tasks()
    
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

        # base = os.path.join(self.pkg_root, "common", "env",)
        # graph_file = os.path.join(base, "experiment",  "0.yaml")
        package_share = get_package_share_directory('emtp_server')
        graph_file = os.path.join(package_share, 'env', 'experiment', '0.yaml')

        if not os.path.exists(graph_file):
            self.get_logger().error(f"Navigation graph file not found: {graph_file}")
            return {}

        data = {}

        try:
            with open(graph_file, "r", encoding="utf-8") as f:
                yaml_data = yaml.safe_load(f)

            levels = yaml_data.get("levels", {})

            for level_name, level_info in levels.items():
                vertices = level_info.get("vertices", [])

                for v in vertices:
                    if not (isinstance(v, list) and len(v) >= 3):
                        continue

                    x = float(v[0])
                    y = float(v[1])
                    props = v[2] if isinstance(v[2], dict) else {}
                    name = props.get("name", "").strip()

                    if not name:
                        continue

                    data[name] = {
                        "x": x,
                        "y": y,
                        "level": level_name,
                    }

        except Exception as e:
            self.get_logger().error(f"[NAV] Error parsing nav graph: {e}")
            return {}
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
                # 1) 빈 리스트면 False
                if not seq:
                    return False
                
                # 2) 내부 값이 빈 문자열/None/공백이면 False
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
            # ----------------------------

            self.init_tasks()
            response.success = True
            self.dynamic_handled = False

            # event_count는 기존 로직 유지
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
        # 메시지만 저장 (heavy logic 금지)
        self.latest_fleet_state = msg
    def process_fleet_state(self):
        if getattr(self, "dynamic_handled", True):
            return
        if self.latest_fleet_state is None:
            return

        fleet_state = self.latest_fleet_state
        POSITION_TOLERANCE = 10.0
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
            current_waypoint = task_info["current"]
            is_empty = (current_id is None or current_id == '')
            if status == "waiting" and is_empty:

                target_pos = self.get_waypoint_location(current_waypoint)
                print(f"Target pos: {target_pos}")
                if target_pos:
                    robot_level = robot_state.location.level_name
                    is_same_level = (robot_level == target_pos['level'])

                    dx = robot_state.location.x - target_pos['x']
                    dy = robot_state.location.y - target_pos['y']
                    distance = math.hypot(dx, dy)
                    
                    is_close_enough = (distance < POSITION_TOLERANCE)
                    print(f"dx={dx:.3f}, dy={dy:.3f}, distance={distance:.3f}")

                    if is_same_level and is_close_enough:
                        print(f"[SKIP/DONE] {robot_name} is already near {current_waypoint} ({distance:.2f}m away). Forcing task completion.")
                        task_info["status"] = "running" 
                        continue
                
                if not task_info.get("resent", False):
                    print(f"[RESEND] {robot_name} task not accepted. Resending once...")
                    robot_name = self.robot_map[key]
                    self.send_task(robot_name, current_waypoint)
                    task_info["resent"] = True
                continue

            # =======================================
            # 2) 로봇이 task 수락함 → current_id != '' AND status='waiting'
            # =======================================
            if status == "waiting" and not is_empty:
                print(f"[ACCEPT] {robot_name} accepted task. Running now.")
                task_info["status"] = "running"
                task_info["id"] = current_id
                task_info.pop("resent", None)
                continue

            # =======================================
            # 3) 실행 중 → 계속 current_id != '' 
            # =======================================
            if status == "running"  and not is_empty:
                task_info["id"] = current_id
                continue  # 그대로 진행

            # =======================================
            # 4) task 완료됨 → status='running'인데 current_id == ''
            # =======================================
            if status == "running" and is_empty:
                done_task = task_info["current"]
                print(f"[DONE] {robot_name} completed task: {done_task}")

                # 기록
                self.robots_pos[key] = done_task
                if done_task not in self.complete_tasks[key]:
                    self.complete_tasks[key].append(done_task)

                self.publish_complete_tasks()

                if key in self.robot_tasks:
                    if self.robot_tasks[key] and self.robot_tasks[key][0] == done_task:
                        self.robot_tasks[key].pop(0)
                # 현재 active 끝
                self.active_tasks.pop(key, None)

                # 다음 task가 있으면 즉시 보내기
                if self.robot_tasks[key]:
                    next_task = self.robot_tasks[key].pop(0)
                    robot_name = self.robot_map[key]
                    self.send_task(robot_name, next_task)
                    self.active_tasks[key] = {
                        "current": next_task,
                        "status": "waiting"
                    }

                continue

        # 🔥 자동 END 체크
        

        self.publish_complete_tasks()
        self.print_active_tasks()
        self.auto_end_check()
    # ----------------------------------------------------
    # 🔥 자동 END 체크 함수
    # ----------------------------------------------------
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

        if not self.event_cli.wait_for_service(timeout_sec=3.0):
            print("[ERROR] Supervisor /event not available")
            return

        req = DynamicEvent.Request()
        payload = {
            "dynamic": dynamic_str,
            "robots_pos": self.robots_pos,
            "complete_tasks": self.complete_tasks,
        }
        self.plan_end_time = None
        self.plan_start_time = time.time()
        req.dynamic = json.dumps(payload, ensure_ascii=False)
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
