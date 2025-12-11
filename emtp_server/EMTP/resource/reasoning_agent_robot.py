from langchain_core.runnables import RunnableLambda
from langchain_core.messages import SystemMessage, HumanMessage

import json
import re
import os
import sys
import copy

base_dir = os.path.dirname(os.path.abspath(__file__))
common_dir = os.path.abspath(os.path.join(base_dir, "..", "..", "common"))
if common_dir not in sys.path:
    sys.path.append(common_dir)

from Dynamics.method.common.llm_call import call_llm


# =============================
# JSON 파서
# =============================
def clean_and_parse_json(text):
    json_block = None
    matches = re.findall(r"```json\s*(\{.*?\}|\[.*?\])\s*```", text, re.DOTALL)
    if matches:
        json_block = matches[0]
    else:
        try:
            json_block = json.loads(text)
            return json_block
        except json.JSONDecodeError:
            pass

    if not json_block:
        print(text)
        raise ValueError("❌ JSON 블록이 LLM 응답에 없음:\n" + text[:300])

    return json.loads(json_block)


# =============================
# ✅ complete_tasks 기반 SceneGraph 갱신 함수
# =============================
def update_scenegraph_status(scenegraph: dict, complete_tasks: list[str]) -> dict:
    """
    이미 status 필드가 존재한다고 가정.
    complete_tasks 목록에 있는 object는 status="complete" 로 변경.
    나머지는 기존 status 유지.
    """
    if not scenegraph:
        return scenegraph

    updated = copy.deepcopy(scenegraph)
    completed_set = {name.lower() for name in complete_tasks or []}

    for room in updated.get("rooms", []):
        for obj in room.get("objects", []):
            name = (obj.get("object_name") or "").lower()
            if name in completed_set:
                obj["status"] = "complete"  # ✅ 기존 값 수정
    return updated


# =============================
# Reasoning 함수
# =============================
def reasoning_fn(state):
    print("✅ reasoning_agent 실행됨!")
    state_dict = state.dict() if hasattr(state, "dict") else state

    task_description = state_dict.get("task_description", "")
    mode = state_dict.get("mode", "offline")
    constraint = state_dict.get("constraint", "")

    additional = state.additional

    # ------------------------------
    # ✅ 3D SceneGraph status 갱신
    # ------------------------------
    pruned_scenegraph = additional.get("pruned_scenegraph", {})
    complete_tasks = additional.get("complete_tasks", [])
    pruned_scenegraph = update_scenegraph_status(pruned_scenegraph, complete_tasks)


    # reasoning 입력 데이터 구성
    relevant_object = additional["relevant_classes"]
    sub_tasks = additional["sub_tasks"]
    robot_info = state_dict.get("robot_info", {})

    robot_data = json.dumps(robot_info, ensure_ascii=False, indent=2)
    pruned_scenegraph_text = json.dumps(pruned_scenegraph, ensure_ascii=False, indent=2)
    relevant_object_text = json.dumps(relevant_object, ensure_ascii=False, indent=2)
    sub_task_text = json.dumps(sub_tasks, ensure_ascii=False, indent=2)
    constraint_text = json.dumps(constraint, ensure_ascii=False, indent=2)

    example_path = os.path.join(base_dir, "example.py")
    with open(example_path, "r", encoding="utf-8") as f:
        example = f.read()
    dynamic_text_raw = additional.get("dynamic", "")
    dynamic_text = json.dumps(dynamic_text_raw, ensure_ascii=False, indent=2) if isinstance(dynamic_text_raw, (dict, list)) else dynamic_text_raw
    # =========================
    # 프롬프트 구성
    # =========================
    system_msg = SystemMessage(
    content=(
        """
    You are a robot task reasoning expert using natural-language task instructions and constraints.

    General principles
    - Infer constraint meaning based on action semantics, not keywords.
    - Derive all restrictions only from the *constraint or instruction text* — do NOT infer from 3D scene graph topology or geometry.
    - If a constraint forbids an area/object, treat it as a full exclusion (no robot allowed).
    - If a constraint allows only specific robot types for a room or passage, express it via:
    - node_restrictions: {"<room_id>": "UAV only" | "UGV only" | "" | "closed"}
    - edge_restrictions: {"<roomA>-<roomB>": "UAV only" | "UGV only" | "" | "closed"}
    - Do NOT hallucinate. If ordering is not clearly implied by the instruction/constraints, output no dependency item (i.e., an empty array []).

    Sub-task definition (IMPORTANT)
    - Sub-task = (action, room, object_class).
    - `room` can be a specific room_id (e.g., "A", "B") or "all".
    - `object_class` can be a specific class (e.g., "소화기", "box") or "all".

    Dependencies output (공간-객체클래스 ONLY)
    - Dependencies must *only* encode room/object_class order (no action string in dependency items).
    - Each dependency item is EITHER:
    1) {"first":  {"room":"<room_id|all>", "object_class":"<class|all>"}}
    2) {"before": {"room":"<room_id|all>", "object_class":"<class|all>"},
        "after":  {"room":"<room_id|all>", "object_class":"<class|all>"}}
    - Use "first" when instruction/constraints ask to do something *first* without an explicit comparison target.
    - Use "before/after" only when there is an explicit or strongly implied ordering relation between two targets.
    - If there is no explicit/implicit ordering cue, produce `"dependencies": []`.

    Natural language → dependency mapping (KOR examples)
    - 객체 우선: "소화기보다 소화전을 먼저" →
    {"before":{"room":"all","object_class":"소화전"},
    "after": {"room":"all","object_class":"소화기"}}
    - 공간 우선: "A 구역을 B보다 먼저" →
    {"before":{"room":"A","object_class":"all"},
    "after": {"room":"B","object_class":"all"}}
    "A 구역 먼저" (비교대상 없음) →
    {"first":{"room":"A","object_class":"all"}}
    - 공간+객체: "A 구역의 소화기 먼저" →
    (비교대상 없음) {"first":{"room":"A","object_class":"소화기"}}
    (비교대상 있음) {"before":{"room":"A","object_class":"소화기"}, "after":{...}}

    STRICT extraction from constraints/instruction
    - Dependencies and restrictions MUST be derived only from explicit or strongly implied cues found in:
    (1) the task instruction text, and/or
    (2) the constraint/dynamic texts.
    - Valid priority cues include: "먼저", "우선", "…보다 먼저", "이후에", "다음에", "first", "before", "prioritize", "precede", "order", "순서".
    - Valid restriction cues include:
        Examples of restriction cues:
        - For nodes: “A 구역은 UAV만”, “B 구역은 진입 금지”, “S1은 UGV 전용”
        - For edges: “H8-H10 통로는 UAV만”, “H1과 H2 사이의 바닥에 넘어갈 수 없는 장애물이 있어!”, “A-B corridor is for ground robots only”
    - Normalize restriction values to one of: `"UAV only"`, `"UGV only"`, `""`, `"closed"`.
    - If cues are ambiguous or conflicting, prefer safety: explain ambiguity briefly in `restrictions_explain` and leave unclear nodes/edges unspecified.
    - Never invent rooms, edges, or classes not present in the constraints or instruction.

    Restrictions handling
    - **Fully closed rooms (Example 9 case)**:
    - When the constraint explicitly states that no robot can enter or the area is entirely forbidden, set:
        `node_restrictions["<room>"] = "closed"`.
    - Additionally, move all objects in that room to `task_loss`, since those tasks cannot be performed.
    - This ensures both the map restriction and task exclusion are explicit.
    - **Excluded rooms or already-done tasks (Example 13 case)**:
    - When the instruction says something like “B 구역은 내가 이미 했어 / 제외하고 해”, do *not* mark it as closed.
    - Do **not** modify `node_restrictions`.
    - Instead, simply add all relevant objects from that room to `task_loss`.
    - The intent is exclusion, not physical inaccessibility.

    Static losses
    - task_loss: add objects/rooms that are removed, destroyed, closed, or explicitly excluded according to constraints.
    - robot_loss: list robots that are malfunctioning or unavailable (e.g., low battery, broken).
    - When a room is fully closed, it will appear both in node_restrictions (as `"closed"`) and its objects will be listed in task_loss.
    - When tasks are merely excluded (but the room is still accessible), only task_loss is used.

    Final Output JSON (MUST)
    {
    "dependencies": [
        { "first":  { "room": "A|all", "object_class": "X|all" } },
        { "before": { "room": "...", "object_class": "..." },
        "after":  { "room": "...", "object_class": "..." } }
    ],
    "node_restrictions": { "<room_id>": "UAV only | UGV only |  | closed" },
    "edge_restrictions": { "<room_id>-<room_id>": "UAV only | UGV only |  | closed" },
    "task_loss": ["<object_name>", ...],
    "robot_loss": ["<robot_id>", ...],
    "restrictions_explain": "<short rationale>"
    }
    """
        )
    )

    human_msg = HumanMessage(
        content=(
            "Instruction:\n"
            f"{task_description}\n\n"
            "Sub tasks (triple form expected internally: action, room|all, object_class|all):\n"
            "```json\n" + sub_task_text + "\n```\n"
            "Constraints / Dynamics (natural-language sources of dependency & restriction cues):\n"
            "```json\n" + constraint_text + "\n```\n"
            "Relevant Object Classes:\n"
            "```json\n" + relevant_object_text + "\n```\n"
            "Robot Information:\n"
            "```json\n" + robot_data + "\n```\n"
            "Mode: " + mode + "\n"
            "\n"
            "SceneGraph (SceneGraph는 이미 이전 reasoning 결과와 모든 제약사항이 반영된 최신 환경 상태이다):\n"
            "```json\n" + pruned_scenegraph_text + "\n```\n"
            "📘 Processing Steps\n"
            "OFFLINE mode\n"
            "1) Extract ordering ONLY if instruction/constraints provide explicit or strongly implied priority.\n"
            "2) From the same text, derive node_restrictions and edge_restrictions based on explicit mentions of rooms or connections.\n"
            "   - e.g., “A 구역 UAV만” → node_restrictions[\"A\"] = \"UAV only\"\n"
            "   - “S1-S2 연결 차단” → edge_restrictions[\"S1-S2\"] = \"closed\"\n"
            "   - If a room is fully closed → do not list it under restrictions; instead, add its objects to task_loss.\n"
            "3) Determine static task_loss (destroyed/closed/unreachable) and robot_loss (malfunction/low battery).\n"
            "\n"
            "ONLINE mode (delta update)\n"
            "Dynamic Constraints (test_server에서 전달된 실시간 제약 정보):\n"
            f"{dynamic_text}\n\n"
            "1) dynamic constraints 와 SceneGraph 를 함께 참고하여, 기존 제약/환경정보가 무엇인지와 새로 추가된 제약이 무엇인지 종합적으로 판단하라.\n"
            "2) 새 제약으로 인해 노드/엣지/우선순위가 바뀌면 그 부분만 갱신하라.\n"
            "3) 변화가 없다면 이전 제약과 구조를 그대로 유지하라.\n"
            "\n"


            "Return only the Final Output JSON schema defined above. No extra commentary.\n"
            "\n"
            "Examples for formatting and reasoning reference:\n"
            f"{example}\n"
        )
    )

    prompt = [system_msg, human_msg]

    # =========================
    # LLM 호출
    # =========================
    reasoning = {}
    try:
        output_text = call_llm(prompt)
        print("======== reasoning Result ========")
        reasoning = clean_and_parse_json(output_text)
        print(json.dumps(reasoning, indent=2, ensure_ascii=False))
    except Exception as e:
        print(f"❌ reasoning 실패: {e}")

    # =========================
    # 상태 업데이트
    # =========================
    new_state = state.copy(
        update={
            "additional": {
                **state.additional,
                "pruned_scenegraph": pruned_scenegraph,  # ✅ 갱신된 SceneGraph 반영
                "reasoning_result": reasoning,
                "prior_reasoning": reasoning,
                "reasoning_done": True,
            }
        }
    )

    return new_state


# Runnable 등록
reasoning_agent = RunnableLambda(reasoning_fn)
reasoning_agent.name = "reasoning_agent"
reasoning_agent.input_keys = ["additional"]
