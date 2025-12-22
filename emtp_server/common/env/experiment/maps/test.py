import cv2
import yaml
import math
import json
import heapq
import numpy as np
from collections import deque
import csv
import os

WAYPOINT_JSON = "waypoints.json"
CSV_OUT = "path_matrix.csv"

FLOORS = {
    0: {"pgm": "warehouse.pgm",  "yaml": "warehouse.yaml",  "name": "F0"},
    1: {"pgm": "warehouse2.pgm", "yaml": "warehouse2.yaml", "name": "F1"},
}

FREE_TH = 250
OCC_TH = 50
INTER_FLOOR_COST = 100.0

# ===== 시각화 옵션 =====
VIEW_SCALE = 0.5          # 창 축소 고정
DRAW_MODE = "chain"       # "chain"=WP0-1-2-3...만, "pairs"=일부 쌍
MAX_PAIR_DRAW = 60        # DRAW_MODE="pairs"일 때만 사용
LINE_THICKNESS = 2        # 선 두께

# =========================================================
# 맵 로드
# =========================================================
def load_floor_map(pgm, yaml_path):
    with open(yaml_path) as f:
        info = yaml.safe_load(f)

    img = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(pgm)

    occ = np.zeros_like(img, np.uint8)
    occ[img >= FREE_TH] = 255
    occ[img <= OCC_TH] = 0

    return {
        "img": img,
        "occ": occ,
        "resolution": float(info["resolution"]),
        "origin": tuple(info["origin"][:2]),
        "height": img.shape[0],
        "width": img.shape[1],
    }

maps = {fid: load_floor_map(v["pgm"], v["yaml"])
        for fid, v in FLOORS.items()}

# =========================================================
# A*
# =========================================================
def astar(start, goal, m):
    w, h = m["width"], m["height"]
    occ = m["occ"]

    pq = [(0.0, start)]
    cost = {start: 0.0}
    parent = {start: None}

    moves = [(-1,0),(1,0),(0,-1),(0,1),
             (-1,-1),(-1,1),(1,-1),(1,1)]

    while pq:
        _, cur = heapq.heappop(pq)
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return cost[goal], path[::-1]

        for dx, dy in moves:
            nx, ny = cur[0]+dx, cur[1]+dy
            if not (0<=nx<w and 0<=ny<h): 
                continue
            if occ[ny,nx] != 255: 
                continue

            nd = cost[cur] + math.hypot(dx,dy)
            if (nx,ny) not in cost or nd < cost[(nx,ny)]:
                cost[(nx,ny)] = nd
                parent[(nx,ny)] = cur
                hcost = math.hypot(goal[0]-nx, goal[1]-ny)
                heapq.heappush(pq, (nd+hcost, (nx,ny)))

    raise RuntimeError("path not found")

def astar_cached(s, g, m, cache, need_path):
    if s == g:
        return (0.0, [s]) if need_path else (0.0, None)

    fid = m["_fid"]
    a,b = (s,g) if s<=g else (g,s)
    key = (fid,a,b,need_path)

    if key in cache:
        return cache[key]

    d,p = astar(s,g,m)
    out = (d, p if need_path else None)
    cache[key] = out
    return out

# =========================================================
# (그대로 둠) 거리 행렬 계산
# =========================================================
def compute_matrix(waypoints):
    for fid,m in maps.items():
        m["_fid"] = fid

    N = len(waypoints)
    INF = 1e12
    mat = np.full((N,N), INF)

    connectors = {}
    for i,wp in enumerate(waypoints):
        if wp.get("type")=="connector":
            connectors.setdefault(wp["connect_id"],{})[wp["floor"]] = i

    cache = {}

    for i in range(N):
        mat[i,i] = 0.0
        wi = waypoints[i]
        si = tuple(wi["pixel"])
        mi = maps[wi["floor"]]

        for j in range(i+1,N):
            wj = waypoints[j]
            sj = tuple(wj["pixel"])

            if wi["floor"] == wj["floor"]:
                try:
                    d,_ = astar_cached(si,sj,mi,cache,False)
                    mat[i,j]=mat[j,i]=d*mi["resolution"]
                except RuntimeError:
                    pass
            else:
                best = INF
                for floors in connectors.values():
                    if wi["floor"] in floors and wj["floor"] in floors:
                        ci = tuple(waypoints[floors[wi["floor"]]]["pixel"])
                        cj = tuple(waypoints[floors[wj["floor"]]]["pixel"])
                        try:
                            d1,_ = astar_cached(si,ci,mi,cache,False)
                            d2,_ = astar_cached(cj,sj,maps[wj["floor"]],cache,False)
                            best = min(best,
                                d1*mi["resolution"] +
                                INTER_FLOOR_COST +
                                d2*maps[wj["floor"]]["resolution"])
                        except RuntimeError:
                            pass
                if best<INF:
                    mat[i,j]=mat[j,i]=best

    with open(CSV_OUT,"w",newline="") as f:
        w = csv.writer(f)
        w.writerow([""]+[f"WP{i}" for i in range(N)])
        for i in range(N):
            w.writerow([f"WP{i}"]+
                [("INF" if mat[i,j]>=INF/2 else f"{mat[i,j]:.3f}") for j in range(N)])

    print("saved:", CSV_OUT)
    return mat

# =========================================================
# ✅ 시각화 전용 (수정본)
# =========================================================
def visualize(waypoints):
    # fid 심기
    for fid,m in maps.items():
        m["_fid"] = fid

    # 배경도 축소해서 만들기
    vis = {}
    for fid, m in maps.items():
        raw = cv2.cvtColor(m["img"], cv2.COLOR_GRAY2BGR)
        vis[fid] = cv2.resize(raw, None, fx=VIEW_SCALE, fy=VIEW_SCALE,
                              interpolation=cv2.INTER_AREA)

    cache = {}

    # ----- 어떤 쌍을 그릴지 결정 -----
    pairs = []
    if DRAW_MODE == "chain":
        # WP0->WP1->WP2... (같은 층만)
        for i in range(len(waypoints)-1):
            wi, wj = waypoints[i], waypoints[i+1]
            if wi["floor"] == wj["floor"]:
                pairs.append((i, i+1))
    else:
        # 일부 쌍만 (MAX_PAIR_DRAW 제한)
        cnt = 0
        for i in range(len(waypoints)):
            for j in range(i+1, len(waypoints)):
                if waypoints[i]["floor"] != waypoints[j]["floor"]:
                    continue
                pairs.append((i, j))
                cnt += 1
                if cnt >= MAX_PAIR_DRAW:
                    break
            if cnt >= MAX_PAIR_DRAW:
                break

    # ----- 경로 그리기 -----
    drawn = 0
    for i, j in pairs:
        wi, wj = waypoints[i], waypoints[j]
        fid = wi["floor"]
        try:
            _, path = astar_cached(tuple(wi["pixel"]),
                                   tuple(wj["pixel"]),
                                   maps[fid],
                                   cache, True)
            if not path:
                continue

            for k in range(len(path)-1):
                p1 = (int(path[k][0] * VIEW_SCALE), int(path[k][1] * VIEW_SCALE))
                p2 = (int(path[k+1][0] * VIEW_SCALE), int(path[k+1][1] * VIEW_SCALE))
                cv2.line(vis[fid], p1, p2, (0,0,255), LINE_THICKNESS)

            drawn += 1
        except RuntimeError:
            # 길 없으면 스킵
            pass

    print(f"[visualize] tried={len(pairs)} drawn={drawn}")

    # ----- waypoint 찍기 -----
    for idx, wp in enumerate(waypoints):
        fid = wp["floor"]
        x, y = wp["pixel"]
        sx, sy = int(x*VIEW_SCALE), int(y*VIEW_SCALE)

        c = (0,255,255) if wp.get("type")=="connector" else (0,255,0)
        cv2.circle(vis[fid], (sx, sy), 4, c, -1)
        label = wp.get("object_name", f"{idx}")
        cv2.putText(vis[fid], label,
                    (sx+6, sy+10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,0,0), 1)

    # ----- show -----
    for fid, img in vis.items():
        title = f"{FLOORS[fid]['name']} | scale={VIEW_SCALE} | red=path"
        hud = img.copy()
        cv2.putText(hud, title, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
        cv2.putText(hud, title, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1)
        cv2.imshow(f"paths_{fid}", hud)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

# =========================================================
# main
# =========================================================
if __name__ == "__main__":
    SAVE_MODE = False
    DO_COMPUTE = False
    DO_VISUALIZE = True

    if SAVE_MODE:
        raise NotImplementedError("SAVE_MODE 생략")

    with open(WAYPOINT_JSON) as f:
        waypoints = json.load(f)

    if DO_COMPUTE:
        compute_matrix(waypoints)

    if DO_VISUALIZE:
        visualize(waypoints)
