from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import pandas as pd, os
import numpy as np
import json
import networkx as nx


def FCM():
    pass


# =========================
# 공통 유틸
# =========================
def _load_json_safe(path):
    try:
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
    except Exception as e:
        print(f"[경고] JSON 로드 실패: {path} -> {e}")
    return None


def _build_room_graph(scene):
    G = nx.Graph()
    if not scene or "rooms" not in scene:
        return G
    for room in scene["rooms"]:
        u = str(room.get("room_id", "")).upper()
        if u:
            G.add_node(u)
    for room in scene["rooms"]:
        u = str(room.get("room_id", "")).upper()
        for v in room.get("connected_rooms", []):
            uu, vv = u, str(v).upper()
            if uu and vv:
                G.add_edge(uu, vv)
    return G




# =========================
# 제약 파서
# =========================
def _parse_room_constraint(val):
    # 여러 문자열 합침
    if isinstance(val, list):
        val = " ".join(val)

    s = (str(val) or "").strip().lower()
    rule = {"closed": False, "allowed_types": set(), "allowed_ids": set()}

    if not s:
        return rule

    if "closed" in s:
        rule["closed"] = True
        return rule

    # uav / ugv 판단 강화
    if "uav" in s:
        rule["allowed_types"].add("UAV")
    if "ugv" in s:
        rule["allowed_types"].add("UGV")

    return rule


def _parse_edge_constraint(val):
    # 구조 같게
    return _parse_room_constraint(val)


def _robot_type(robot: dict) -> str:
    # 디폴트를 UGV 로 본다
    return str(robot.get("type") or "UGV").upper()


def _compute_reachable_rooms(G, scene, robot_types, start_key, room_of, df=None):

    """
    접근성 계산 (단순화 버전)
    - 로봇 타입(UGV/UAV)에 따라 허용 노드만 포함
    """
    reachable = {}
    closed_nodes_global = set()

    # print("==================scene===============")
    # print(scene)
    for rid, st in start_key.items():
        rtype = robot_types.get(rid.upper(), "UGV")
        Gc = G.copy()

        # 방 제약 적용
        for room in scene.get("rooms", []):
            rid_room = str(room["room_id"]).upper()
            cons = room.get("node_constraints", [])
            rule = _parse_room_constraint(cons)

            if rule.get("closed"):
                if rid_room in Gc:
                    Gc.remove_node(rid_room)
                    closed_nodes_global.add(rid_room)
            elif rule.get("allowed_types"):
                # 타입 제한 존재 시
                if rtype not in rule["allowed_types"]:
                    if rid_room in Gc:
                        Gc.remove_node(rid_room)
                        closed_nodes_global.add(rid_room)

        # 엣지 제약 (동일)
        for ek, val in (scene.get("edge_constraints") or {}).items():
            erule = _parse_edge_constraint(val)
            parts = ek.split("-")
            if len(parts) != 2:
                continue
            u, v = parts[0].upper(), parts[1].upper()

            # ❌ 완전히 닫힌 경우
            if erule.get("closed"):
                if Gc.has_edge(u, v):
                    Gc.remove_edge(u, v)
                continue

            # ⚠️ 타입 제약 적용
            allowed = erule.get("allowed_types", set())
            if allowed and rtype not in allowed:
                if Gc.has_edge(u, v):
                    Gc.remove_edge(u, v)

        # 시작점 보정 및 연결 영역 탐색
        start_room = str(st).replace("room_", "").upper()
        start_room = room_of.get(str(st).lower(), None)
        if start_room:
            start_room = start_room.upper()
        else:
            start_room = str(st).replace("room_", "").upper()
        if (start_room not in Gc.nodes) and (df is not None):
            if start_room.lower() in df.index:
                valid_nodes = [n for n in Gc.nodes if n.lower() in df.columns]
                if valid_nodes:
                    distances = {
                        n: float(df.loc[start_room.lower(), n.lower()])
                        for n in valid_nodes
                        if n.lower() in df.columns and n.lower() != start_room.lower()
                    }
                    if distances:
                        nearest = min(distances, key=distances.get)
                        # print(f"🔁 {rid}: {start_room} → 최근접 유효 방 {nearest}")
                        start_room = nearest.upper()

        # 연결 가능한 노드 탐색
        if start_room in Gc.nodes:
            reach = nx.node_connected_component(Gc, start_room)
            reachable[rid] = {r.lower() for r in reach}
        else:
            print(f"[주의] {rid}: 시작점 {start_room} 이(가) 그래프에 없음.")
            reachable[rid] = set()

    closed_nodes = {r.lower() for r in closed_nodes_global}
    return reachable, G, closed_nodes


def MVRP(objects, robots, environment="warehouse", scene=None, start_at=None, open_routes=True):
    import copy
    env_dir = os.path.expanduser("~/emtp_ws/src/emtp_server/common/env/college")
    matrix_path = os.path.join(env_dir, "path_matrix.csv")

    df = pd.read_csv(matrix_path, index_col=0)
    index_map = {str(k).lower(): str(k) for k in df.index}
    column_map = {str(k).lower(): str(k) for k in df.columns}
    df.index = df.index.astype(str).str.lower()
    df.columns = df.columns.astype(str).str.lower()

    # scene load
    if scene is None:
        scene_path = os.path.join(env_dir, f"{environment}_hard.json")
        with open(scene_path, "r", encoding="utf-8") as f:
            scene = json.load(f)
    else:
        scene = copy.deepcopy(scene)

    # build graph
    G_rooms = _build_room_graph(scene)

    # robots
    robot_ids = [r["name"].upper() for r in robots]
    robot_id_orig = [r["name"] for r in robots]
    robot_types = {r["name"].upper(): str(r.get("type", "UGV")).upper() for r in robots}
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
    start_key = {
        rid: (start_at or {}).get(rid, default_bases.get(rid, rid))
        for rid in robot_id_orig
    }

    # object → room
    room_of = {}
    for o in objects:
        oid = str(o["id"]).lower()
        rid = str(o.get("room", "")).replace("room_", "").lower()
        room_of[oid] = rid
    object_ids = list(room_of.keys())

    # reachable rooms
    reachable_rooms, G_constrained, closed_nodes = _compute_reachable_rooms(
        G_rooms, scene, robot_types, start_key, room_of, df
    )


    # banned rooms filter
    true_closed_rooms = {
        str(room["room_id"]).replace("room_", "").lower()
        for room in scene.get("rooms", [])
        if "closed" in str(room.get("node_constraints", [])).lower()
    }
    if true_closed_rooms:
        objects = [o for o in objects if room_of[o["id"].lower()] not in true_closed_rooms]
        object_ids = [str(o["id"]).lower() for o in objects]

    # distance matrices
    V, N = len(robot_ids), len(objects)
    start_depot = N
    num_nodes = N + V

    def obj_key(k: str) -> str:
        kk = str(k).lower()
        if (kk in df.index) and (kk in df.columns):
            return kk
        if kk in room_of:
            return room_of[kk]
        return kk

    vehicle_distance_mats = []

    for v_idx, rid in enumerate(robot_id_orig):
        dm = np.zeros((num_nodes, num_nodes), dtype=float)

        # job <-> job
        for i, oi in enumerate(object_ids):
            oi_key = obj_key(oi)
            for j, oj in enumerate(object_ids):
                oj_key = obj_key(oj)
                dm[i, j] = float(df.loc[oi_key, oj_key])

        # start -> job
        s_idx = start_depot + v_idx
        s_key_raw = start_key[rid]
        s_key = obj_key(s_key_raw)
        for j, oj in enumerate(object_ids):
            oj_key = obj_key(oj)
            dm[s_idx, j] = float(df.loc[s_key, oj_key])

        vehicle_distance_mats.append(dm)

    # OR-Tools
    start_nodes = list(range(start_depot, start_depot + V))
    end_nodes   = start_nodes

    manager = pywrapcp.RoutingIndexManager(num_nodes, V, start_nodes, end_nodes)
    routing = pywrapcp.RoutingModel(manager)

    # set cost evaluator per vehicle
    callbacks = []
    for v_idx, dm in enumerate(vehicle_distance_mats):
        def make_cb(dm_local):
            def cb(from_i, to_i):
                a, b = manager.IndexToNode(from_i), manager.IndexToNode(to_i)
                return int(dm_local[a, b] * 1000)
            return cb

        cb_idx = routing.RegisterTransitCallback(make_cb(dm))
        callbacks.append(cb_idx)
        routing.SetArcCostEvaluatorOfVehicle(cb_idx, v_idx)

    # -----------------------------
    # ******* TRUE MIN-MAX *******
    # -----------------------------
    # single global distance callback
    def global_cb(from_i, to_i):
        a = manager.IndexToNode(from_i)
        b = manager.IndexToNode(to_i)
        return int(vehicle_distance_mats[0][a, b] * 1000)

    global_cb_idx = routing.RegisterTransitCallback(global_cb)

    # single global distance dimension
    routing.AddDimension(
        global_cb_idx,
        0,
        int(1e9),
        True,
        "Distance"
    )
    routing.GetDimensionOrDie("Distance").SetGlobalSpanCostCoefficient(1000)

    # -----------------------------
    # reachable constraints
    # -----------------------------
    for i, oid in enumerate(object_ids):
        rm = room_of[oid]
        for v_idx, rU in enumerate(robot_ids):
            if rm.lower() not in reachable_rooms.get(rU, set()):
                routing.VehicleVar(manager.NodeToIndex(i)).RemoveValue(v_idx)

    # search
    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.FromSeconds(4)
    params.log_search = False

    solution = routing.SolveWithParameters(params)
    if not solution:
        raise RuntimeError("❌ Routing 실패: 접근 불가 방 포함 가능")

    # extract result
    routes, total_costs = {}, {}
    for v_idx, rid in enumerate(robot_id_orig):
        routes[rid] = []
        idx = routing.Start(v_idx)

        while not routing.IsEnd(idx):
            node = manager.IndexToNode(idx)
            if node < N:
                routes[rid].append(objects[node]["id"])
            idx = solution.Value(routing.NextVar(idx))

        seq = routes[rid]

        if not seq:
            total_costs[rid] = 0.0
            continue

        # compute cost manually (start→first + seq)
        start_to_first = float(df.loc[obj_key(start_key[rid]), obj_key(seq[0])])
        inter = 0.0
        for a, b in zip(seq[:-1], seq[1:]):
            inter += float(df.loc[obj_key(a), obj_key(b)])
        total_costs[rid] = round(start_to_first + inter, 2)

    def restore_key(k: str) -> str:
        kl = str(k).lower()
        return index_map.get(kl) or column_map.get(kl) or k

    routes_restored = {rid: [restore_key(o) for o in seq] for rid, seq in routes.items()}
    paths_restored = {rid: " -> ".join(routes_restored[rid]) for rid in routes_restored}

    end_points = {}
    for rid in robot_id_orig:
        seq = routes_restored[rid]
        if seq:
            last_obj = seq[-1]
            end_points[rid] = room_of.get(str(last_obj).lower(), start_key[rid])
        else:
            end_points[rid] = start_key[rid]

    return {
        "allocation": routes_restored,
        "path_len": total_costs,
        "paths": paths_restored,
        "scene_used": scene,
        "end_points": end_points
    }
