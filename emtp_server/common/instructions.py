# -*- coding: utf-8 -*-
from typing import List, Tuple, Optional, Dict

def make_instructions(
    environment: str,
    topic: str,
    level: str,
    task_type: str = "test1",            # "test1", "test2-1", "test2-2", ...
    task_indices: Optional[List[int]] = None,
    constraint_indices: Optional[List[int]] = None,
    evaluation: str = "static",          # ★ 추가됨: static / dynamic
    robot_count: int = None,
) -> Tuple[List[str], List[str], List[str]]:

    # ===== 1) TASKS =====
    TASK_LIBRARY: Dict[Tuple[str, str], Dict[str, List[str]]] = {
        ("warehouse", "fire"): {

            "test1": [
                # test1 - 객체 수에 따른 성능 평가
                "Inspect all the firealarmbutton",  # 10
                "Inspect all firedetectors",         # 20
                "Inspect all firealarmbutton and firedetectors",                    # 30
                "Inspect all firealarmbutton and firedetectors and firehydrants",   # 40
                "Inspect all the fire extinguishers",                                # 50 ★ dynamic 기본 사용 문장
                "Inspect all the fire extinguishers and fireexitsigns",             # 60
                "Inspect all the fire extinguishers and firedetectors",              # 70
                "Inspect all the fire extinguishers and firedetectors and firealarmbutton",  # 80
                "Inspect all the fire extinguishers and firedetectors and firealarmbutton and fireexitsigns", # 90
                "Inspect all fire safety equipment",  # 100
            ],

            # test2-* 공통 태스크 문장(시나리오별로 제약만 바뀜)
            "test2": [
                "Inspect all fire safety equipment"
            ],
        },

        ("experiment", "fire"): {
            "test1": [
                "Inspect all fire safety equipment"
            ],
        }
    }

    # ===== 2) SCENARIO 제약 =====
    SCENARIO_CONSTRAINTS: Dict[str, List[str]] = {
        "test2_1": [
            "Inspection of fire safety tasks in B1 & B2 area must be excluded",
            "Inspection of fire safety tasks in fast-moving stock must be excluded",
            "Do not inspect the fire detectors in the A",
            "Do not inspect the fire detectors in the main storage."
        ],
        "test2_2": [
            "firealarmbutton must be inspected first before any other fire safety equipment",
            "Inspect firehydrants before fire extinguishers.",
            "Inspect the fire safety items with fire detectors in the fast-moving stock first.",
            "Handle C1 before B1.",
            "Inspect fire detectors in cold storage first"
        ],
        "test2_3": [
            "Robot R1 has malfunctioned",
            "Robot R2 has malfunctioned",
            "UAV has malfunctioned",
        ],
        "test2_4": [
            "D2 는 UGV 출입을 금지합니다.",
            "fast-moving stock area can only be accessed by UAVs",
            "cold storage area can only be accessed by UGVs",
        ],
        "test2_5": [
            "Do not inspect any fire detectors, fire hydrants, fire extinguishers, or fire exit signs.",
            "Do not inspect any fire alarm buttons, fire hydrants, fire extinguishers, or fire exit signs.",
            "Do not inspect any fire hydrants, fire extinguishers, or fire exit signs.",
            "Do not inspect any fire extinguishers or fire exit signs.",
            "Do not inspect any fire hydrants, fire detectors, fire alarm buttons, or fire exit signs.",
            "Do not inspect any fire detectors, fire alarm buttons, or fire hydrants.",
            "Do not inspect any fire hydrants, fire alarm buttons, or fire exit signs.",
            "Do not inspect any fire hydrants or fire exit signs.",
            "Do not inspect only fire hydrants.",
            "",
        ],
        "test2_6": [
            "First inspect all the fire extinguishers, then the fire hydrants, then the fire alarms, and finally the fireexitsigns.",
        ],
    }

    # ===== 3) 난이도 제약 =====
    DIFFICULTY_CONSTRAINTS: Dict[Tuple[str, str], Dict[str, List[str]]] = {
        ("warehouse", "fire"): {"easy": [""], "medium": [""], "hard": [""]},
        ("experiment", "fire"): {"easy": [""], "medium": [""], "hard": [""]},
    }

    # ===== 4) ROBOT RULES =====
    ROBOT_RULES: Dict[Tuple[str, str], Dict[str, str]] = {
        ("warehouse", "fire"): {
            "easy": "homogeneous",
            "medium": "heterogeneous2",
            "hard": "heterogeneous3"
        },
        ("experiment", "fire"): {
            "easy": "homogeneous",
            "medium": "heterogeneous2",
            "hard": "heterogeneous3"
        },
    }

    ROBOT_SETS: Dict[str, List[str]] = {
        "homogeneous":   ["R1", "R2"],
        "heterogeneous2":["R1", "R3"],
        "heterogeneous3":["R1", "R2", "R3"],
    }

    # ===== helper =====
    def _pick(seq: List[str], idxs: Optional[List[int]]) -> List[str]:
        if idxs is None:
            return seq[:]
        return [seq[i] for i in idxs if 0 <= i < len(seq)]


    level_norm = (level or "").strip().lower()
    env_topic = (environment, topic)

    # =====================================================
    # ★★★  DYNAMIC MODE  — 너가 요구한 그대로 ★★★
    # =====================================================
    if evaluation == "dynamic":
        instructions = ["Inspect all fire safety equipment"]   # ★ 직접 지정
        robot_type = ROBOT_RULES.get(env_topic, {}).get(level_norm, "homogeneous")
        robot_list = ROBOT_SETS.get(robot_type, ["R1"])
        return instructions, [], robot_list     # ★ constraints = 빈 배열

    if task_type == "test3":
        instructions = ["Inspect all fire safety equipment"]
        selected_constraints = []

        # robot_count가 지정되지 않았다면 기본 3대
        if robot_count is None:
            robot_count = 3

        # R1~R10 풀에서 robot_count 만큼만 사용
        full_robots = ["R1","R2","R3","R4","R5","R6","R7","R8","R9","R10"]
        robot_list = full_robots[:robot_count]

        return instructions, selected_constraints, robot_list
    # =====================================================
    # ★ STATIC MODE (기존 코드 그대로 유지)
    # =====================================================

    # ----- A) Task 선택 -----
    tl = TASK_LIBRARY.get(env_topic, {})
    if task_type.startswith("test2_"):
        tasks = tl.get("test2", [])
    else:
        tasks = tl.get(task_type, [])

    if not tasks:
        raise ValueError(f"No tasks for {env_topic} / {task_type}")

    if task_indices is None:
        selected_tasks = tasks[:]
    else:
        selected_tasks = _pick(tasks, task_indices)
        if not selected_tasks:
            raise ValueError(f"No valid task_indices: {task_indices}")

    # ----- B) Constraint 선택 -----
    if task_type.startswith("test2_"):
        cons_all = SCENARIO_CONSTRAINTS.get(task_type, [])
        selected_constraints = _pick(cons_all, constraint_indices)
    else:
        cons_all = DIFFICULTY_CONSTRAINTS.get(env_topic, {}).get(level_norm, [])
        selected_constraints = _pick(cons_all, constraint_indices)

    # ----- C) Robot 구성 -----
    robot_type = ROBOT_RULES.get(env_topic, {}).get(level_norm, "homogeneous")
    robot_list = ROBOT_SETS.get(robot_type, ["R1"])

    # ----- D) 반환 -----
    return selected_tasks, selected_constraints, robot_list
