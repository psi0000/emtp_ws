from langchain_core.runnables import RunnableLambda
from EMTP.resource.method import FCM, MVRP 
import json
import os
import matplotlib.pyplot as plt
import threading

METHOD = "opt" #"opt" or "sga" or "fcm"



_ros_initialized = False
_ros_node = None
_ros_thread_started = False
_ros_lock = threading.Lock()


def get_ros_allocation_node():
    """
    Node, client, publishers 모두 단 1회만 생성되는 싱글턴 함수
    """
    global _ros_initialized, _ros_node, _ros_thread_started

    with _ros_lock:
        import rclpy
        from rclpy.node import Node
        from psi_interfaces.srv import AllocationResult
        from psi_interfaces.msg import SceneGraph, RobotPath

        # ----- ROS INIT -----
        if not _ros_initialized:
            if not rclpy.ok():
                rclpy.init()
            _ros_initialized = True

        # ----- Node Singleton -----
        if _ros_node is None:
            class AllocationNode(Node):
                def __init__(self):
                    super().__init__("allocation_master")
                    # publisher
                    self.scene_pub = self.create_publisher(SceneGraph, "/updated_scene", 10)
                    self.relevant_pub = self.create_publisher(SceneGraph, "/relevant_classes", 10)

                    # service client
                    self.cli = self.create_client(AllocationResult, "/allocation")
                    if not self.cli.wait_for_service(timeout_sec=10.0):
                        print("[WARN] /allocation service not available")
                        self.cli = None

                    # latest scene (updated by allocation_fn)
                    self.latest_scene = None
                def publish_relevant_classes(self, cls_list):
                    msg = SceneGraph()
                    msg.json_data = json.dumps({"relevant_classes": cls_list}, ensure_ascii=False)
                    self.relevant_pub.publish(msg)
                # 서비스 호출
                def send_paths(self, paths_dict):
                    if self.cli is None:
                        return

                    paths_as_list = {
                        rid: [p.strip() for p in seq.split("->")] 
                        if isinstance(seq, str) else seq
                        for rid, seq in paths_dict.items()
                    }

                    robot_path_msgs = []
                    for rid, seq in paths_as_list.items():
                        msg = RobotPath()
                        msg.robot_name = rid
                        msg.path_sequence = seq
                        robot_path_msgs.append(msg)
                    req = AllocationResult.Request()
                    req.paths = robot_path_msgs

                    future = self.cli.call_async(req)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

                    if future.done() and future.result():
                        # print(f"✅ AllocationResult: success={future.result().success}")
                        pass

                # continuous scene publish
                def publish_latest_scene(self):
                    if self.latest_scene is None:
                        return
                    msg = SceneGraph()
                    msg.json_data = json.dumps(self.latest_scene, ensure_ascii=False)
                    self.scene_pub.publish(msg)

            _ros_node = AllocationNode()

        # ----- Publishing Thread Singleton -----
        if not _ros_thread_started:
            def loop():
                import time
                while rclpy.ok():
                    try:
                        _ros_node.publish_latest_scene()
                        time.sleep(0.2)
                    except Exception:
                        pass

            th = threading.Thread(target=loop, daemon=True)
            th.start()
            _ros_thread_started = True

        return _ros_node
# =========================
# 유틸
# =========================
def _norm_token(x: str) -> str:
    return (str(x or "")).strip().lower()


def _norm_room(x: str) -> str:
    x = _norm_token(x)
    return x.replace("room_", "") if x != "all" else "all"


def _norm_class(x: str) -> str:
    return "all" if _norm_token(x) in ("", "any", "*", "all") else _norm_token(x)


def _selector_key(sel: dict) -> str:
    return f"room={_norm_room(sel.get('room'))}|class={_norm_class(sel.get('object_class'))}"


def _parse_dependencies_to_selectors(dependencies: list) -> dict:
    """
    reasoning.dependencies 를
      - 노드들
      - first 로 지정된 것들
      - before/after 엣지
    로 나눠서 selector 리스트로 만들기
    """
    nodes, first_list, edges = {}, [], []

    if not dependencies:
        return {"first": [], "edges": [], "nodes": {}}

    for d in dependencies:
        # { "first": {room:..., object_class:...} }
        if "first" in d and isinstance(d["first"], dict):
            raw = d["first"]
            sel = {
                "room": _norm_room(raw.get("room")),
                "object_class": _norm_class(raw.get("object_class")),
            }
            k = _selector_key(sel)
            nodes[k] = sel
            first_list.append(k)
        else:
            # { "before": {...}, "after": {...} }
            b, a = d.get("before", {}), d.get("after", {})
            if not isinstance(b, dict) or not isinstance(a, dict):
                continue
            sb = {"room": _norm_room(b.get("room")), "object_class": _norm_class(b.get("object_class"))}
            sa = {"room": _norm_room(a.get("room")), "object_class": _norm_class(a.get("object_class"))}
            kb, ka = _selector_key(sb), _selector_key(sa)
            nodes[kb] = sb
            nodes[ka] = sa
            if kb != ka:
                edges.append((kb, ka))

    return {"first": first_list, "edges": edges, "nodes": nodes}


def _toposort(nodes: dict, edges: list) -> list:
    from collections import defaultdict, deque

    indeg, adj = defaultdict(int), defaultdict(list)
    for u, v in edges:
        adj[u].append(v)
        indeg[v] += 1
        indeg.setdefault(u, 0)
    for k in nodes.keys():
        indeg.setdefault(k, 0)

    q = deque([k for k, d in indeg.items() if d == 0])
    order = []
    while q:
        u = q.popleft()
        order.append(u)
        for v in adj[u]:
            indeg[v] -= 1
            if indeg[v] == 0:
                q.append(v)

    # 남은 노드도 뒤에 붙이기
    for k in nodes.keys():
        if k not in order:
            order.append(k)
    return order


def _selector_matches_obj(sel: dict, obj: dict) -> bool:
    r_ok = (sel["room"] == "all") or (_norm_room(obj.get("room")) == _norm_room(sel["room"]))
    c_ok = (sel["object_class"] == "all") or (_norm_class(obj.get("class")) == _norm_class(sel["object_class"]))
    return r_ok and c_ok


# =========================
# reasoning → scene 업데이트
# =========================
def apply_reasoning_to_scene(scene: dict, reasoning: dict, additional: dict) -> dict:
    import copy
    if not scene:
        return {}

    scene2 = copy.deepcopy(scene)


    # --- 입력 정리 ---
    node_restr = {k.lower(): v for k, v in (reasoning.get("node_restrictions") or {}).items()}
    edge_restr = reasoning.get("edge_restrictions") or {}
    reasoning.setdefault("task_loss", [])
    task_loss_set = {t.strip().lower() for t in reasoning["task_loss"]}

    # ✅ dynamic에서 넘어온 complete_tasks 반영
    complete_dict = additional.get("complete_tasks", {}) or {}
    # flatten all robot tasks into one lowercase set
    complete_tasks = {
        str(x).strip().lower()
        for robot_list in complete_dict.values()
        for x in (robot_list or [])
    }
    
    # --- 노드 제약 및 task_loss 반영 ---
    for room in scene2.get("rooms", []):
        rid = (room.get("room_id") or "").replace("room_", "").lower()
        if rid in node_restr:
            room["node_constraints"] = [str(node_restr[rid])]

        if any(c.lower() == "closed" for c in room.get("node_constraints", [])):
            for obj in room.get("objects", []):
                obj["status"] = "excluded"
                if obj.get("object_name") not in reasoning["task_loss"]:
                    reasoning["task_loss"].append(obj.get("object_name"))

    # --- 최신 task_loss로 오브젝트 상태 갱신 ---
    task_loss_set = {t.strip().lower() for t in reasoning["task_loss"]}
    for room in scene2.get("rooms", []):
        for obj in room.get("objects", []):
            if (obj.get("object_name") or "").strip().lower() in task_loss_set:
                obj["status"] = "excluded"
    for room in scene2.get("rooms", []):
        for obj in room.get("objects", []):
            obj_name = (obj.get("object_name") or "").strip().lower()
            if obj_name in complete_tasks:
                obj["status"] = "complete"
                
    # --- edge 제약 병합 ---
    ec = scene2.get("edge_constraints", {}).copy()

    for k, v in edge_restr.items():
        # always convert to list of strings
        if isinstance(v, str):
            v = [v]
        elif not isinstance(v, list):
            v = [str(v)]

        if k not in ec or not isinstance(ec[k], list):
            ec[k] = v
        else:
            # merge
            ec[k] = list(set(ec[k] + v))

    scene2["edge_constraints"] = ec

    # --- dependency 기반 우선순위 ---
    deps = reasoning.get("dependencies", [])
    if deps:
        dep = _parse_dependencies_to_selectors(deps)
        topo = _toposort(dep["nodes"], dep["edges"])
        priority_map = {k: i + 1 for i, k in enumerate(topo)}
        for room in scene2.get("rooms", []):
            rid = _norm_room(room.get("room_id"))
            for obj in room.get("objects", []):
                obj_class = _norm_class(obj.get("class"))
                for k, v in priority_map.items():
                    if _selector_matches_obj(dep["nodes"][k], {"room": rid, "class": obj_class}):
                        obj["priority"] = v
                        break

    # --- reasoning 메타 저장 ---
    scene2["meta"] = {
        "dependencies": deps,
        "robot_loss": reasoning.get("robot_loss", []),
        "task_loss": reasoning["task_loss"],
        "restrictions_explain": reasoning.get("restrictions_explain", "")
    }
    # print("[DEBUG] ====== UPDATED SCENE ======")
    # print(json.dumps(scene2, indent=2, ensure_ascii=False))
    # print("[DEBUG] ===========================")

    return scene2


# =========================
# 시각화 (간단)
# =========================
def visualize_scene_with_overlays(scene, robots=None, save_path=None):
    robots = robots or []
    rooms = scene.get("rooms", [])
    edge_constraints = scene.get("edge_constraints", {})

    meta = scene.get("meta", {})
    robot_loss = [str(x).upper() for x in meta.get("robot_loss", [])]

    fig = plt.figure(figsize=(16, 8))
    ax_scene = fig.add_subplot(121, projection="3d")
    ax_robot = fig.add_subplot(122)

    # rooms + objects
    for r in rooms:
        center = r.get("room_center") or [0, 0, 0]
        rx, ry, rz = center
        rid = str(r.get("room_id", "")).upper()
        node_cons = r.get("node_constraints") or []
        color = "gold" if node_cons else "lightgray"

        ax_scene.scatter(rx, ry, rz, s=450, c=color, edgecolors="black")
        label = rid if not node_cons else f"{rid} ({' / '.join(node_cons)})"
        ax_scene.text(rx, ry, rz + 1.5, label, fontsize=8)

        for obj in r.get("objects", []):
            ox, oy, oz = obj.get("location", [rx, ry, rz])
            status = (obj.get("status") or "").lower()
            c, m, a = "gray", "o", 1.0
            if status == "excluded":
                c, m = "red", "x"
            elif status == "complete":
                a = 0.3
            ax_scene.scatter(ox, oy, oz, s=60, c=c, marker=m, edgecolors="black", alpha=a)

    # edges
    for r in rooms:
        src = str(r.get("room_id", "")).upper()
        cx, cy, cz = r.get("room_center", [0, 0, 0])
        for tgt in r.get("connected_rooms", []):
            tgt_u = str(tgt).upper()
            tgt_room = next((rr for rr in rooms if str(rr.get("room_id", "")).upper() == tgt_u), None)
            if not tgt_room:
                continue
            tx, ty, tz = tgt_room.get("room_center", [0, 0, 0])

            ek = f"{src}-{tgt_u}"
            rk = f"{tgt_u}-{src}"
            cons = edge_constraints.get(ek) or edge_constraints.get(rk)
            if cons:
                ax_scene.plot([cx, tx], [cy, ty], [cz, tz], color="red", linewidth=2)
                ax_scene.text((cx+tx)/2, (cy+ty)/2, (cz+tz)/2 + 1, str(cons), color="red", fontsize=6)
            else:
                ax_scene.plot([cx, tx], [cy, ty], [cz, tz], color="lightblue", linewidth=1)

    ax_scene.set_title("Scene")
    ax_scene.set_xlabel("X"); ax_scene.set_ylabel("Y"); ax_scene.set_zlabel("Z")

    # robots
    ax_robot.set_title("Robots")
    ax_robot.set_xlim(0, 1); ax_robot.set_ylim(0, 1)
    ax_robot.axis("off")
    if not robots:
        ax_robot.text(0.1, 0.9, "no robots")
    else:
        y = 0.9
        for rb in robots:
            rn = str(rb.get("name", "")).upper()
            rt = str(rb.get("type", "")).upper()
            if rn in robot_loss:
                ax_robot.text(0.05, y, f"{rn} ({rt}) X", color="red")
            else:
                ax_robot.text(0.05, y, f"{rn} ({rt})")
            y -= 0.1

    plt.tight_layout()
    if save_path:
        # plt.show()
        # plt.savefig(save_path, dpi=160)
        pass
    else:
        # plt.show()
        pass


# =========================
# 메인 할당 함수
# =========================

def allocation_fn(state):
    print("Allocation 실행 시작 (method:", METHOD, ")")

    # ----- 입력 파싱 -----
    state_dict = state.dict() if hasattr(state, "dict") else state
    robots = state_dict["robot_info"]
    add = state_dict["additional"]
    env = state_dict.get("environment") or add.get("environment") or "warehouse"

    sg_data = add.get("pruned_scenegraph", {}) or {}
    reasoning = add.get("reasoning_result", {}) or {}
    dependencies = reasoning.get("dependencies", []) or []
    relevant_classes = [cls.lower() for cls in (add.get("relevant_classes") or [])]
    robots_pos = add.get("robots_pos") or {}
    complete_dict = add.get("complete_tasks") or {}
    complete_tasks = {
        _norm_token(t)
        for robot_tasks in complete_dict.values()
        for t in (robot_tasks or [])
    }
    # ----- reasoning → scene -----
    scene_updated = apply_reasoning_to_scene(sg_data, reasoning, add)

    # ----- active 로봇만 -----
    
    # robot_loss = {str(r).strip().upper() for r in reasoning.get("robot_loss", [])} #reasoning 에서 로봇고장 판단할때
    robot_loss = {str(r).strip().upper() for r in add.get("robot_loss", [])}   # supervisor 에서 로봇고장 판단할때

    all_robot_names_upper = [str(r["name"]).upper() for r in robots]
    active_robot_names = [r for r in all_robot_names_upper if r not in robot_loss]
    active_robots = [r for r in robots if str(r["name"]).upper() in active_robot_names]

    

    # ----- 씬 시각화 -----
    visualize_scene_with_overlays(
        scene_updated,
        robots=active_robots
    )
    # ----- 작업 필터링 -----
    reasoning_task_loss = {c.strip().lower() for c in reasoning.get("task_loss", [])}
    all_objects = []
    for room in scene_updated.get("rooms", []):
        rid = room.get("room_id")
        for obj in room.get("objects", []):
            cls_ = (obj.get("class") or "").lower()
            name = obj.get("object_name")
            loc = obj.get("location")
            status = (obj.get("status") or "").lower()

            if not name or not loc:
                continue
            # ✅ 추가 (모두 lower 정규화)
            name_key = name.strip().lower()

            if relevant_classes and cls_ not in relevant_classes:
                continue
            if name_key in reasoning_task_loss:      # 기존 task_loss 체크
                continue
            if name_key in complete_tasks:           # ✅ 수정: 소문자 비교로 통일
                continue
            if status in ("complete", "excluded"):
                continue

            all_objects.append({
                "id": name,
                "class": cls_,
                "room": rid,
                "location": loc,
                "priority": obj.get("priority", None),   # ← 추가
            })
    
    # ----- priority 기반 stage 생성 -----

    priority_groups = {}

    for o in all_objects:
        pr = o.get("priority", None)
        if pr is None:
            pr = 9999
        priority_groups.setdefault(pr, []).append(o)

    stages = [
        (f"priority_{p}", objs) 
        for p, objs in sorted(priority_groups.items(), key=lambda x: x[0])
    ]
    # ----- 제약 추출 -----
    room_constraints = {}
    for r in scene_updated.get("rooms", []):
        rid = r.get("room_id")
        cons = r.get("node_constraints") or []
        if rid and cons:
            room_constraints[rid] = cons[0] if len(cons) == 1 else cons

    edge_constraints = scene_updated.get("edge_constraints", {}) or {}

    # ----- 초기 위치 -----
    default_bases = {
        "R1": "L1_delivery_charger1",
        "R2": "L1_delivery_charger2",
        "R3": "L1_delivery_charger3",
        "R4": "L1_delivery_charger3",
        "R5": "L1_delivery_charger3",
        "R6": "L1_delivery_charger3",
        "R7": "L1_delivery_charger3",
        "R8": "L1_delivery_charger3",
        "R9": "L1_delivery_charger3",
        "R10": "L1_delivery_charger3",
    }
    start_at = {}
    for r in active_robots:
        rn = r["name"]
        start_at[rn] = robots_pos.get(rn) or default_bases.get(rn.upper(), rn)

    # ----- MVRP 실행 -----
    stage_results = []
    last_stage_name = stages[-1][0] if stages else "all"

    for stage_name, stage_objs in stages:
        open_routes = (stage_name != last_stage_name)
        if METHOD == "opt":
            import time
            t0 = time.time()
            res = MVRP(
                objects=stage_objs,
                robots=active_robots,
                environment=env,
                scene=scene_updated,        
                start_at=start_at,
                open_routes=open_routes,
            )
            print(f"MVRP stage '{stage_name}' solved in {time.time()-t0:.2f}s")
        elif METHOD == "sga":
            import time
            t0 = time.time()
            res = SGA(
                objects=stage_objs,
                robots=active_robots,
                environment=env,
                scene=scene_updated,
                start_at=start_at,
                open_routes=open_routes,
            )
            print(f"SGA stage '{stage_name}' solved in {time.time()-t0:.2f}s")
        else:
            res = FCM()

        next_start_at = start_at.copy()
        for rid, end_key in (res.get("end_points") or {}).items():
            if end_key:
                next_start_at[rid] = end_key
        start_at = next_start_at

        stage_results.append({
            "stage": stage_name,
            "objects_count": len(stage_objs),
            "result": res
        })

    # ----- 결과 병합 -----
    flat_allocation, flat_path_len = {}, {}
    stage_allocations, stage_path_lens, stage_paths = {}, {}, {}
    for st in stage_results:
        sname, res = st["stage"], st.get("result", {})
        alloc, pl, pstr = res.get("allocation", {}), res.get("path_len", {}), res.get("paths", {})
        stage_allocations[sname], stage_path_lens[sname], stage_paths[sname] = alloc, pl, pstr
        for r, seq in alloc.items():
            flat_allocation.setdefault(r, []).extend(seq)
        for r, d in pl.items():
            flat_path_len[r] = flat_path_len.get(r, 0.0) + float(d or 0.0)
    flat_paths = {r: " -> ".join(seq) for r, seq in flat_allocation.items()}

    # ✅ 결과 저장
    output_data = {
        "allocation_counts": {r: len(v) for r, v in flat_allocation.items()},
        "allocation_total": sum(len(v) for v in flat_allocation.values()),
        "allocation": flat_allocation,
        "path_len": flat_path_len,
        "paths": flat_paths,
        "total_time": sum(flat_path_len.values()),
        "status": "success"
    }


    node = get_ros_allocation_node()
    node.send_paths(output_data["paths"])
    node.latest_scene = scene_updated
    node.publish_relevant_classes(relevant_classes)
    # ----- state 업데이트 -----
    new_state = state.copy(update={
        "additional": {
            **add,
            "pruned_scenegraph": scene_updated,
            "final_result": {
                "mode": "multi_stage" if len(stages) > 1 else "single_stage",
                "selectors_order": [],
                "stages": stage_results,
                "allocation": flat_allocation,
                "path_len": flat_path_len,
                "paths": flat_paths,
                "stage_allocations": stage_allocations,
                "stage_path_lens": stage_path_lens,
                "stage_paths": stage_paths,
            },
            "active_robots": active_robot_names,
            "allocation_done": True
        }
    })
    return new_state


# runnable 등록
allocation_agent = RunnableLambda(allocation_fn)
allocation_agent.name = "allocation_agent"
allocation_agent.input_keys = ["task_description", "constraint", "robot_info", "additional"]