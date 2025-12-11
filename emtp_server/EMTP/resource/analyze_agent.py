from langchain_core.runnables import RunnableLambda
import json
import os
import sys
import re

# 📁 경로 설정
common_dir = os.path.expanduser("~/emtp_ws/src/emtp_server/common")
        
from common.llm_call import call_llm

token_usage = {"analyze_agent": 0}

def analyze_fn(state):
    print("analyze_agent")

    state_dict = state.dict() if hasattr(state, "dict") else state
    mode = state_dict.get("mode", "offline")
    dynamic = ""
    if mode == "online":
        dynamic = state_dict.get("additional", {}).get("dynamic", "")

    task_description = state_dict.get("task_description", "")
    constraint = state_dict.get("constraint", "")
    object_list = state_dict.get("object_list", [])  # object classes only
    robot_info = state_dict.get("robot_info", {})
    additional = state_dict.get("additional", {})

    # 🌐 full SG 로드 (based on environment + level)
    environment = state_dict.get("additional", {}).get("environment", "")
    level = state_dict.get("additional", {}).get("level", "")
    sg_path = os.path.join(common_dir, "env", environment, f"{environment}_{level}.json")
    try:
        with open(sg_path, encoding="utf-8") as f:
            full_sg_data = json.load(f)
    except Exception as e:
        print(f"❌ SG 파일 로드 실패: {e}")
        full_sg_data = {"rooms": []}

    existing_relevant = additional.get("relevant_classes", [])
    existing_pruned_sg = additional.get("pruned_scenegraph", {})
    existing_subtasks = additional.get("sub_tasks", [])
    dynamic_info = additional.get("dynamic", {})

    # 🧠 LLM 프롬프트 구성
    offline_prompt = (
        "You are an AI that extracts only necessary object classes for a given task.\n\n"
        f"Order prompt: \"{task_description}\"\n"
        f"dynamic: {dynamic}\n"
        f"Available Object Classes: {json.dumps(object_list)}\n"
        f"Robot Environment: {json.dumps(robot_info)}\n"
        "Based on the above, extract and return all necessary object's classes the robots must interact with to complete the task.\n"
        "Decompose the instruction into executable sub-tasks based on robot capabilities of Robot Environment.\n"
        "   - DO NOT include specific robot names (e.g., 'r1') in the sub-tasks.\n"
        "   - DO NOT include specific room names (e.g., 'in l2_room3') in the sub-tasks.\n"
        "   - DO NOT generate duplicated sub-tasks per room.\n"
        "   - DO NOT expand the task for each room or object instance. Generalize instead.\n\n"
        "✅ Example :\n"
        "Order prompt: 'i want to clean every beds,vases in the building'\n"
        "Output:\n"
        "{\n"
        '  "relevant_classes": ["bed", "vase"],\n'
        '  "sub_tasks": ["cleanobject bed", "cleanobject vase"]\n'
        "}\n\n"
        "Output format (strictly this and nothing else):\n"
        "{\n"
        '  "relevant_classes": ["object1", "object2", ...],\n'
        '  "sub_tasks": ["sub-task1", "sub-task2", ...]\n'
        "}\n"
        "Return only this raw JSON object. Do NOT include markdown (```), explanation, or any extra text.\n"
        "All property names and string values MUST use double quotes."
    )
    online_prompt = f"""
    You are an AI that extracts only necessary object classes for a given task.

    Task Instruction:
    {task_description}

    New Dynamic Constraint (latest changes only):
    {json.dumps(dynamic_info, ensure_ascii=False)}

    Existing Relevant Classes:
    {json.dumps(existing_relevant, ensure_ascii=False)}

    Existing Sub Tasks:
    {json.dumps(existing_subtasks, ensure_ascii=False)}

    All Available Object Classes:
    {json.dumps(object_list, ensure_ascii=False)}
    Robot Environment:
    {json.dumps(robot_info, ensure_ascii=False)}
    Based on the above, extract only additional object classes newly required due to the dynamic information, excluding classes already included in the existing relevant classes.
    Decompose the instruction into executable sub-tasks based on robot capabilities of Robot Environment.
    - DO NOT include specific robot names (e.g., 'r1') in the sub-tasks.
    - DO NOT include specific room names (e.g., 'in l2_room3') in the sub-tasks.
    - DO NOT generate duplicated sub-tasks per room.
    - DO NOT expand the task for each room or object instance. Generalize instead.

    Example:
    Order prompt: 'i want to clean every beds,vases in the building'
    Output:
    {{
    "relevant_classes": ["bed", "vase"],
    "sub_tasks": ["cleanobject bed", "cleanobject vase"]
    }}

    Output format (STRICTLY this and nothing else):
    {{
    "relevant_classes": ["object1", "object2", ...],
    "sub_tasks": ["sub-task1", "sub-task2", ...],
    }}

    Return only this raw JSON object. No markdown, no explanation.
    All property names and string values MUST use double quotes.
    """

    
    prompt = online_prompt if mode == "online" else offline_prompt
    # 🤖 LLM 호출
    response = call_llm(prompt)
    try:
        json_str = re.search(r"\{(?:.|\n)*\}", response).group(0)
        extracted = json.loads(json_str)
    except Exception as e:
        print("❌ JSON parsing error:", e)
        extracted = {"relevant_classes": [], "sub_tasks": []}

    print(f"📦 extracted_info: {extracted}")

    if mode == "offline":
        relevant_classes = [c.lower() for c in extracted.get("relevant_classes", [])]
        sub_tasks = extracted.get("sub_tasks", [])

        pruned_rooms = []
        for room in full_sg_data.get("rooms", []):
            filtered_objs = [
                o for o in room.get("objects", [])
                if o.get("class", "").lower() in relevant_classes
            ]
            pruned_rooms.append({
                "floor": room.get("floor"),
                "room_id": room.get("room_id"),
                "room_name": room.get("room_name", ""),
                "room_center": room.get("room_center", []),
                "connected_rooms": room.get("connected_rooms", []),
                "node_constraints": room.get("node_constraints", []),
                "objects": filtered_objs
            })

        pruned_scenegraph = {
            "rooms": pruned_rooms,
            "edge_constraints": full_sg_data.get("edge_constraints", {})
        }
    else:
        # 신규 LLM 결과
        new_relevant = extracted.get("relevant_classes", [])
        new_subtasks = extracted.get("sub_tasks", [])

        # 기존 + 신규 병합
        relevant_classes = list(set(
            [c.lower() for c in existing_relevant] +
            [c.lower() for c in new_relevant]
        ))
        sub_tasks = list(set(existing_subtasks + new_subtasks))

        # 기존 pruned SG 참조
        pruned_scenegraph = existing_pruned_sg.copy()
        rooms = pruned_scenegraph.get("rooms", [])

        # full SG 룸 인덱싱
        full_rooms = full_sg_data.get("rooms", [])
        full_rooms_by_id = {r["room_id"]: r for r in full_rooms}

        # 각 room 업데이트
        for room in rooms:
            room_id = room["room_id"]
            objs = room.get("objects", [])

            # 원본 SG 의 동일 room
            full_room = full_rooms_by_id.get(room_id, {})
            full_objs = full_room.get("objects", [])

            # 필요한 object 추가 (중복 체크)
            for obj in full_objs:
                obj_class = obj.get("class", "").lower()
                if obj_class in relevant_classes:
                    if not any(o.get("object_name") == obj.get("object_name") for o in objs):
                        objs.append(obj)

            # 🔥 node_constraints 적용
            if "closed" in room.get("node_constraints", []):
                for o in objs:
                    o["status"] = "excluded"

            room["objects"] = objs  # 수정 적용 완료

        # pruned_scenegraph 방금 수정한 rooms 그대로 재사용
        pruned_scenegraph = {
            "rooms": rooms,
            "edge_constraints": existing_pruned_sg.get("edge_constraints", {})
        }
    # print(pruned_scenegraph)
    # 📤 결과 업데이트
    analyze_output = {
        "relevant_classes": relevant_classes,
        "sub_tasks": sub_tasks,
        "pruned_scenegraph": pruned_scenegraph
    }

    new_state = state.copy(update={
        "additional": {
            **state.additional,
            "analyze_done": True,
            **analyze_output
        }
    })

    return new_state

analyze_agent = RunnableLambda(analyze_fn)
analyze_agent.name = "analyze_agent"
analyze_agent.input_keys = ["task_description", "constraint", "robot_info", "object_list", "additional"]
