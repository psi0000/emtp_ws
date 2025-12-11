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
    if isinstance(val, list):
        val = val[0] if val else ""
    s = (str(val) or "").strip().lower()
    if not s:
        return {}
    if "closed" in s:
        return {"closed": True, "allowed_types": set(), "allowed_ids": set()}
    if "uav" in s:
        return {"closed": False, "allowed_types": {"UAV"}, "allowed_ids": set()}
    if "ugv" in s:
        return {"closed": False, "allowed_types": {"UGV"}, "allowed_ids": set()}
    return {}


def _parse_edge_constraint(val):
    # 구조 같게
    return _parse_room_constraint(val)


def _robot_type(robot: dict) -> str:
    # 디폴트를 UGV 로 본다
    return str(robot.get("type") or "UGV").upper()


def _compute_reachable_rooms(G, scene, robot_types, start_key, df=None):
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
                        print(f"🔁 {rid}: {start_room} → 최근접 유효 방 {nearest}")
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
    base_dir = os.path.dirname(os.path.abspath(__file__))
    env_dir  = os.path.join(base_dir, "..", "..", "common", "env", environment)
    matrix_path = os.path.join(env_dir, "path_matrix.csv")

    df = pd.read_csv(matrix_path, index_col=0)
    index_map = {str(k).lower(): str(k) for k in df.index}
    column_map = {str(k).lower(): str(k) for k in df.columns}
    df.index = df.index.astype(str).str.lower()
    df.columns = df.columns.astype(str).str.lower()

    # ✅ scene graph 로드
    if scene is None:
        scene_path = os.path.join(env_dir, f"{environment}_hard.json")
        with open(scene_path, "r", encoding="utf-8") as f:
            scene = json.load(f)
    else:
        scene = copy.deepcopy(scene)

    # ----- 그래프 구성 -----
    G_rooms = _build_room_graph(scene)

    # ----- 로봇 정보 -----
    robot_ids = [r["name"].upper() for r in robots]
    robot_id_orig = [r["name"] for r in robots]
    robot_types = {r["name"].upper(): str(r.get("type", "UGV")).upper() for r in robots}
    default_bases = {
        "R1": "L1_delivery_charger1",
        "R2": "L1_delivery_charger2",
        "R3": "L1_delivery_charger3",
        "R4": "L1_delivery_charger2"
    }
    start_key = {
        rid: (start_at or {}).get(rid, default_bases.get(rid, rid))
        for rid in robot_id_orig
    }

    # ----- 객체-방 매핑 -----
    room_of = {}
    for o in objects:
        oid = str(o["id"]).lower()
        rid = str(o.get("room", "")).replace("room_", "").lower()
        room_of[oid] = rid
    object_ids = list(room_of.keys())

    # ----- Reachable rooms -----
    reachable_rooms, G_constrained, closed_nodes = _compute_reachable_rooms(G_rooms, scene, robot_types, start_key, df)

    # print("🧩 Reachable rooms per robot:")
    # print(json.dumps({k: list(v) for k, v in reachable_rooms.items()}, indent=2, ensure_ascii=False))

    # 🚫 완전히 닫힌 방만 필터링
    true_closed_rooms = {
        str(room["room_id"]).replace("room_", "").lower()
        for room in scene.get("rooms", [])
        if "closed" in str(room.get("node_constraints", [])).lower()
    }

    if true_closed_rooms:
        objects = [o for o in objects if room_of[o["id"].lower()] not in true_closed_rooms]
        object_ids = [str(o["id"]).lower() for o in objects]

    # ----- 거리행렬 (로봇별) -----
    V, N = len(robot_ids), len(objects)
    start_depot = N
    num_nodes, BIG = N + V , 1e6

    def obj_key(k: str) -> str:
        kk = str(k).lower()
        if (kk in df.index) and (kk in df.columns):
            return kk
        if kk in room_of:
            return room_of[kk]
        return kk

    vehicle_distance_mats = []

    for v_idx, rid in enumerate(robot_id_orig):
        rtype = robot_types[rid.upper()]
        dm = np.zeros((num_nodes, num_nodes), dtype=float) 

        # --- 1) 작업 ↔ 작업 거리 ---
        for i, oi in enumerate(object_ids):
            oi_key = obj_key(oi)
            for j, oj in enumerate(object_ids):
                oj_key = obj_key(oj)
                base_cost = float(df.loc[oi_key, oj_key])
                dm[i, j] = base_cost

        # --- 2) 시작 → 작업
        s_idx = start_depot + v_idx
        s_key_raw = start_key[rid]
        s_key = obj_key(s_key_raw)

        # 시작 → 작업
        for j, oj in enumerate(object_ids):
            oj_key = obj_key(oj)
            dm[s_idx, j] = float(df.loc[s_key, oj_key])


        # 저장
        vehicle_distance_mats.append(dm)
    
    start_nodes = list(range(start_depot, start_depot + V))
    end_nodes   = start_nodes 
    # ===== OR-Tools =====
    manager = pywrapcp.RoutingIndexManager(
        num_nodes,
        V,
        start_nodes,
        end_nodes
    )
    routing = pywrapcp.RoutingModel(manager)
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

    for v_idx in range(V):
        routing.AddDimension(
            callbacks[v_idx],
            0,
            int(1e9),
            True,
            f"Distance_{v_idx}"
        )
        # 🔥 이전 코드와 동일하게 span cost coefficient 설정
        routing.GetDimensionOrDie(f"Distance_{v_idx}").SetGlobalSpanCostCoefficient(1000)

    # =========================
    # 3) 노드 접근 제한 (scene reachable 기반)
    # =========================
    for i, oid in enumerate(object_ids):
        room_id = room_of[oid]
        banned_list = []
        for v_idx, rU in enumerate(robot_ids):
            reachable_set = reachable_rooms.get(rU, set())
            if room_id.lower() not in reachable_set:
                routing.VehicleVar(manager.NodeToIndex(i)).RemoveValue(v_idx)
                banned_list.append(rU)
        if len(banned_list) == len(robot_ids):
            # print(f"🚫 {oid} (room={room_id}) → 모든 로봇 접근 불가!")
            continue
        elif banned_list:
            # print(f"🚫 {oid} (room={room_id}) 금지된 로봇: {banned_list}")
            pass
    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.FromSeconds(4)
    params.log_search = False
    solution = routing.SolveWithParameters(params)
    if not solution:
        raise RuntimeError("❌ Routing 실패: 접근 불가 방 포함 가능")

    # ===== 결과 해석 =====
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

        # ✅ 기존 방식 복원: start → first, inter, end → sink
        start_to_first = 0.0
        if obj_key(start_key[rid]).lower() in df.index and obj_key(seq[0]).lower() in df.columns:
            start_to_first = float(df.loc[obj_key(start_key[rid]).lower(), obj_key(seq[0]).lower()])

        inter = 0.0
        for a, b in zip(seq[:-1], seq[1:]):
            a_k, b_k = obj_key(a).lower(), obj_key(b).lower()
            if a_k in df.index and b_k in df.columns:
                inter += float(df.loc[a_k, b_k])

        total_costs[rid] = round(start_to_first + inter, 2)

    # ✅ key 복원 (기존 유지)
    def restore_key(k: str) -> str:
        if not k:
            return ""
        kl = str(k).lower()
        return index_map.get(kl) or column_map.get(kl) or k

    routes_restored = {rid: [restore_key(o) for o in seq] for rid, seq in routes.items()}
    paths_restored = {rid: " -> ".join(seq) for seq in routes_restored.values()}

    print("\n=== 결과 ===")
    for rid in robot_id_orig:
        seq = " → ".join(routes_restored[rid]) or "(작업 없음)"
        print(f"{rid}: {seq} | total={total_costs[rid]:.2f}")

    # ✅ end_points 동일
    end_points = {}
    for rid in robot_id_orig:
        seq = routes_restored.get(rid, [])
        if seq:
            last_obj = seq[-1]
            end_points[rid] = room_of.get(last_obj.lower(), start_key[rid])
        else:
            end_points[rid] = start_key[rid]

    return {
        "allocation": routes_restored,
        "path_len": total_costs,
        "paths": paths_restored,
        "scene_used": scene,
        "end_points": end_points
    }


def SGA(objects, robots, environment="warehouse", scene=None, start_at=None, open_routes=True):
    """
    MVRP와 동일 제약/전처리/입출력 구조를 가진
    Sequential Greedy Assignment(SGA) 기반 VRP.
    OR-Tools 글로벌 최적화만 Greedy로 교체함.
    """

    import copy
    import numpy as np
    import pandas as pd
    import os, json, networkx as nx

    # ======== 기본 환경 세팅 ========
    base_dir = os.path.dirname(os.path.abspath(__file__))
    env_dir  = os.path.join(base_dir, "..", "..", "common", "env", environment)
    matrix_path = os.path.join(env_dir, "path_matrix.csv")

    df = pd.read_csv(matrix_path, index_col=0)
    index_map = {str(k).lower(): str(k) for k in df.index}
    column_map = {str(k).lower(): str(k) for k in df.columns}
    df.index = df.index.astype(str).str.lower()
    df.columns = df.columns.astype(str).str.lower()

    # ======== scene load ========
    if scene is None:
        scene_path = os.path.join(env_dir, f"{environment}_hard.json")
        with open(scene_path, "r", encoding="utf-8") as f:
            scene = json.load(f)
    else:
        scene = copy.deepcopy(scene)

    # ======== Build room graph ========
    G_rooms = _build_room_graph(scene)

    # ======== Robot info ========
    robot_ids = [r["name"].upper() for r in robots]
    robot_id_orig = [r["name"] for r in robots]
    robot_types = {r["name"].upper(): str(r.get("type", "UGV")).upper() for r in robots}

    default_bases = {
        "R1": "L1_delivery_charger1",
        "R2": "L1_delivery_charger2",
        "R3": "L1_delivery_charger3",
        "R4": "L1_delivery_charger2"
    }

    start_key = {
        rid: (start_at or {}).get(rid, default_bases.get(rid, rid))
        for rid in robot_id_orig
    }

    # ======== Object → Room 매핑 ========
    room_of = {}
    for o in objects:
        oid = str(o["id"]).lower()
        rid = str(o.get("room", "")).replace("room_", "").lower()
        room_of[oid] = rid
    object_ids = list(room_of.keys())

    # ======== Reachable rooms 계산 (MVRP와 동일) ========
    reachable_rooms, G_constrained, closed_nodes = _compute_reachable_rooms(
        G_rooms, scene, robot_types, start_key, df
    )

    # ======== 완전히 closed된 방 object 제거 (MVRP와 동일) ========
    true_closed_rooms = {
        str(room["room_id"]).replace("room_", "").lower()
        for room in scene.get("rooms", [])
        if "closed" in str(room.get("node_constraints", [])).lower()
    }

    if true_closed_rooms:
        objects = [o for o in objects if room_of[o["id"].lower()] not in true_closed_rooms]
        object_ids = [str(o["id"]).lower() for o in objects]

    # ======== Helper: key normalizer ========
    def obj_key(k: str) -> str:
        kk = str(k).lower()
        if (kk in df.index) and (kk in df.columns):
            return kk
        if kk in room_of:
            return room_of[kk]
        return kk

    # ======== Sequential Greedy Solver START ========

    # 로봇별 path / cost
    path = {rid: [] for rid in robot_id_orig}
    Sp   = {rid: 0.0 for rid in robot_id_orig}

    # 초기 score
    score = {rid: {} for rid in robot_id_orig}
    for rid in robot_id_orig:
        s_key = obj_key(start_key[rid])
        for oid in object_ids:
            room_s = obj_key(oid)
            score[rid][oid] = -float(df.loc[s_key, room_s])

    J = set(object_ids)

    # 거리 계산 함수 (MVRP와 동일한 규칙)
    def compute_length(seq, rid):
        if not seq:
            return 0.0

        start_r = obj_key(start_key[rid])
        first_r = obj_key(seq[0])
        total = float(df.loc[start_r, first_r])

        for a, b in zip(seq[:-1], seq[1:]):
            total += float(df.loc[obj_key(a), obj_key(b)])

        return total

    # Greedy 반복
    while J:
        best = None
        best_val = -1e18

        # 가장 score가 높은 (robot, task)
        for rid in robot_id_orig:
            reachable = reachable_rooms.get(rid.upper(), set())

            for oid in J:
                if room_of[oid] not in reachable:
                    continue
                val = score[rid][oid]
                if val > best_val:
                    best_val = val
                    best = (rid, oid)

        if best is None:
            break

        rid, oid = best
        J.remove(oid)

        # 삽입 위치 탐색
        cur = path[rid]
        best_pos = 0
        best_score = -1e18

        for k in range(len(cur) + 1):
            newp = cur[:k] + [oid] + cur[k:]
            new_len = compute_length(newp, rid)
            inc = -(new_len - Sp[rid])
            if inc > best_score:
                best_score = inc
                best_pos = k

        # 반영
        cur.insert(best_pos, oid)
        Sp[rid] = compute_length(cur, rid)

        # max route
        maxL = max(Sp.values())

        # score 업데이트
        for r2 in robot_id_orig:
            reachable = reachable_rooms.get(r2.upper(), set())
            for oj in J:

                if room_of[oj] not in reachable:
                    score[r2][oj] = -1e18
                    continue

                p2 = path[r2]
                best_s = -1e18

                for k in range(len(p2) + 1):
                    np2 = p2[:k] + [oj] + p2[k:]
                    new_l = compute_length(np2, r2)

                    if new_l > maxL:
                        s = -(new_l - maxL) * 100
                    else:
                        s = -(new_l - Sp[r2])

                    if s > best_s:
                        best_s = s

                score[r2][oj] = best_s

        # ======== MVRP와 동일한 key 복원 함수 ========
    def restore_key(k: str) -> str:
        if not k:
            return ""
        kl = str(k).lower()
        return index_map.get(kl) or column_map.get(kl) or k

    # ======== allocation 복원 ========
    allocation_restored = {
        rid: [restore_key(o) for o in path[rid]]
        for rid in robot_id_orig
    }

    # ======== paths 복원 ========
    paths_restored = {
        rid: " -> ".join(allocation_restored[rid])
        for rid in robot_id_orig
    }

    # ======== end_points 복원 ========
    end_points = {}
    for rid in robot_id_orig:
        seq = allocation_restored[rid]
        if seq:
            last_obj = seq[-1]
            end_points[rid] = room_of.get(last_obj.lower(), start_key[rid])
        else:
            end_points[rid] = start_key[rid]

    # ======== 최종 return (MVRP 동일) ========
    return {
        "allocation": allocation_restored,
        "path_len": {rid: round(Sp[rid], 2) for rid in robot_id_orig},
        "paths": paths_restored,
        "scene_used": scene,
        "end_points": end_points
    }
