from langchain_core.runnables import RunnableLambda
import time
import json
import threading
import rclpy
from rclpy.node import Node
from psi_interfaces.srv import DynamicEvent
from common.llm_call import call_llm


def wait_for_dynamic():
    dynamic_box = {"value": None, "robots_pos": {}, "complete_tasks": []}

    def _service_thread():
        if not rclpy.ok():
            rclpy.init()
        node = Node("supervisor_event_server")
        
        def _callback(request, response):
            try:
                payload = json.loads(request.dynamic)
            except Exception:
                payload = {}
            new_dynamic = (payload.get("dynamic") or "").strip()
            dynamic_box["value"] = new_dynamic
            dynamic_box["robots_pos"] = payload.get("robots_pos", {})
            dynamic_box["complete_tasks"] = payload.get("complete_tasks") or {}
            response.success = True
            response.message = "Supervisor received dynamic"
            node.get_logger().info(f"[EVENT] Dynamic 수신: {new_dynamic}")
            return response

        node.create_service(DynamicEvent, "/event", _callback)
        node.get_logger().info("[EVENT] /event service opened (waiting for AllocationServer)")

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if dynamic_box["value"]:
                break
            time.sleep(0.05)

        node.get_logger().info("[EVENT] /event service closed")
        node.destroy_node()
        # ❌ rclpy.shutdown() 제거 (main.py에서 할 것)

    th = threading.Thread(target=_service_thread, daemon=False)
    th.start()

    while not dynamic_box["value"]:
        time.sleep(0.1)

    th.join()
    return dynamic_box
def classify(task_description: str, constraint: str, robot_info: dict, additional: dict) -> str:
    prompt = f"""
    You are a supervisor agent in a multi-robot system.

    ## Current Task Description:
    {task_description}

    ## Incoming Dynamic Input:
    {constraint}

    ## Robot Information:
    {json.dumps(robot_info, ensure_ascii=False, indent=2)}

    ## Instruction:
    You must classify the dynamic input into exactly ONE of the following categories:

    1. new_task
       - When the dynamic input introduces something that requires the system to perform additional work compared to the current task_description and constraints.
       - If the new information adds new objects, new areas, new targets, new hazards, or anything that requires re-analyzing tasks (즉, analyze 단계부터 다시 시작해야 하는 경우), classify as new_task.

    2. constraint_update
       - When the input modifies constraints, environment conditions, or task parameters, but does NOT add a new task and is NOT about robot malfunction.

    3. robot_failure
       - When the input describes robot breakdown, malfunction, battery failure, communication loss, robot stopping, robot stuck, or anything requiring urgent reallocation.
       - This should be chosen when **any robot becomes unavailable or cannot continue its task**.

    Output rules:
    1) If it is new_task → respond ONLY with: new_task
    2) If it is constraint_update → respond ONLY with: constraint_update
    3) If it is robot_failure → respond ONLY with the JSON:

    respond ONLY with: 
    {{
      "type": "robot_failure",
      "robot_loss": ["<robot_id>", ...]
    }}
    """.strip()
    raw = call_llm(prompt).strip()
    # print(f"supervisor result: {raw}")
    clean = raw.strip()

    # 코드블럭 제거
    if clean.startswith("```"):
        clean = clean.strip("`")           # 앞뒤 ``` 제거
        clean = clean.replace("json", "", 1).strip()

    # 문자열 안에서 JSON 객체 패턴 찾기
    start = clean.find("{")
    end   = clean.rfind("}")

    json_str = None
    if start != -1 and end != -1 and end > start:
        json_str = clean[start:end+1]

    # -------------------------------------------
    # 2) JSON 파싱 시도
    # -------------------------------------------
    if json_str:
        try:
            data = json.loads(json_str)
            if data.get("type") == "robot_failure":
                additional["robot_loss"] = data.get("robot_loss", [])
                print(f"robot failure : {additional['robot_loss']}")
                return "allocation"
        except Exception as e:
            print(f"[JSON PARSE ERROR] {e}")

    # -------------------------------
    # 기존 문자열 처리 그대로 유지
    # -------------------------------
    result = raw.lower()
      # 즉시 재할당

    if "new_task" in result:
        return "analyze"          # 기존 analyze 흐름

    if "constraint_update" in result:
        return "reasoning"        # 제약 update -> reasoning 처리

    # fallback: 기존 irrelevant 처리
    return "irrelevant"

def validate_reasoning(reasoning_result):
        
    prompt = f"""
    You are validation module.

    Check whether this reasoning result is consistent and logically valid.

    Reasoning Result:
    {json.dumps(reasoning_result, ensure_ascii=False, indent=2)}

    Respond only one word: "pass" or "fail".
    """
    out = call_llm(prompt).strip().lower()
    return "pass" if out.startswith("pass") else "fail"
def supervisor_fn(state: dict) -> dict:
    print("supervisor_agent")
    state_dict = state.dict() if hasattr(state, "dict") else state

    mode = state_dict.get("mode", "offline")
    additional = state_dict.get("additional", {}) or {}
    robot_info = state_dict.get("robot_info", {})
    # 현재 state 값
    dynamic = (additional.get("dynamic") or "").strip()
    robots_pos = additional.get("robots_pos") or {}
    complete_tasks = additional.get("complete_tasks") or []
    classification = (additional.get("classification") or "").strip().lower()

    analyze_done     = additional.get("analyze_done", False)
    reasoning_done   = additional.get("reasoning_done", False)
    validation_done  = additional.get("validation_done", False)
    allocation_done  = additional.get("allocation_done", False)
    
    
    # ==================== OFFLINE ====================
    if mode == "offline":
        if not analyze_done:
            next_step = "analyze"
        elif not reasoning_done:
            next_step = "reasoning"
        elif not validation_done:
            # supervisor 내부에서 검증 진행
            print("validation")
            reasoning_result = additional.get("reasoning_result", {})

            # res = validate_reasoning(reasoning_result)
            res= "pass"  # TODO: 임시 하드코딩
            # print(f"🔍 검증 결과: {res}")

            if res == "fail":
                # reasoning 다시 실행하게 플래그 롤백
                print("❌ 검증 실패 → reasoning 다시 진행")
                return state.copy(update={
                    "additional": {
                        **additional,
                        "reasoning_done": False,
                        "validation_done": False,
                        "next_step": "reasoning",
                        "robots_pos": robots_pos,
                        "complete_tasks": complete_tasks,
                    }
                })
            else:
                return state.copy(update={
                    "additional": {
                        **additional,
                        "reasoning_done": True,
                        "validation_done": True,
                        "next_step": "supervisor_agent",
                        "robots_pos": robots_pos,
                        "complete_tasks": complete_tasks,
                    }
                })
        elif not allocation_done:
            next_step = "allocation"
        else:
            return state.copy(update={
                "mode": "online",
                "additional": {
                    **additional,
                    "next_step": "finish",
                    "dynamic": "",
                    "classification": "",
                    "robots_pos": robots_pos,
                    "complete_tasks": complete_tasks,
                }
            })

        return state.copy(update={
            "additional": {
                **additional,
                "next_step": next_step,
                "robots_pos": robots_pos,
                "complete_tasks": complete_tasks,
            }
        })

    # ==================== ONLINE ====================
    elif mode == "online":
        # print("🌐 ONLINE MODE")
        if dynamic and not classification:
            classification = classify(state.task_description, dynamic, robot_info, additional)
            if classification not in {"analyze", "reasoning", "allocation"}:
                print("⚠️ irrelevant dynamic → supervisor 대기")
                return state.copy(update={
                    "additional": {
                        **additional,
                        "next_step": "supervisor_agent",
                        "dynamic": "",
                        "classification": "",
                        "robots_pos": robots_pos,
                        "complete_tasks": complete_tasks,
                    }
                })

            print(f"Next step -> {classification}")
            if classification == "analyze":
                reset_flags = {
                    "analyze_done": False,
                    "reasoning_done": False,
                    "validation_done": False,
                    "allocation_done": False,
                }
            elif classification == "reasoning":
                reset_flags = {
                    "analyze_done": True,
                    "reasoning_done": False,
                    "validation_done": False,
                    "allocation_done": False,
                }
            elif classification == "allocation":
                reset_flags = {
                    "analyze_done": True,
                    "reasoning_done": True,
                    "validation_done": True,
                    "allocation_done": False,
                }
            return state.copy(update={
                "additional": {
                    **additional,
                    **reset_flags,
                    "next_step": classification,
                    "dynamic": dynamic,
                    "classification": classification,
                    "robots_pos": robots_pos,
                    "complete_tasks": complete_tasks,
                }
            })

        
        if dynamic and not analyze_done:
            return state.copy(update={
            "additional": {
                **additional,
                "next_step": "analyze",
                "robots_pos": robots_pos,
                "complete_tasks": complete_tasks,
            }
            })
        if dynamic and analyze_done and not reasoning_done:
            return state.copy(update={
            "additional": {
                **additional,
                "next_step": "reasoning",
                "robots_pos": robots_pos,
                "complete_tasks": complete_tasks,
            }
            })
        # 1) reasoning 끝났고 validation 아직이면 → validation
        if dynamic and reasoning_done and not validation_done:
            # print("📍 ONLINE: reasoning 완료 → supervisor 내부 validation 시작")
            
            reasoning_result = additional.get("reasoning_result", {})

            # res = validate_reasoning(reasoning_result)
            res = "pass"  # TODO: 임시
            # print(f"🔍 검증 결과: {res}")

            if res == "fail":
                print("❌ ONLINE 검증 실패 → reasoning 다시 실행")
                return state.copy(update={
                    "additional": {
                        **additional,
                        "reasoning_done": False,
                        "validation_done": False,
                        "next_step": "reasoning",
                        "robots_pos": robots_pos,
                        "complete_tasks": complete_tasks,
                    }
                })

            # print("✅ ONLINE 검증 통과 → supervisor 로 다시 돌아감")
            return state.copy(update={
                "additional": {
                    **additional,
                    "validation_done": True,
                    "next_step": "supervisor_agent",
                    "robots_pos": robots_pos,
                    "complete_tasks": complete_tasks,
                }
            })

        # 2) validation 끝났고 allocation 아직이면 → allocation
        if dynamic and reasoning_done and validation_done and not allocation_done:
            # print("📍 validation 완료 → allocation 실행")
            return state.copy(update={
                "additional": {
                    **additional,
                    "next_step": "allocation",
                    "robots_pos": robots_pos,
                    "complete_tasks": complete_tasks,
                }
            })

        # 2) allocation 끝났으면 이때만 초기화
        if allocation_done:
            # print("✅ dynamic 처리 완료 → supervisor 대기 복귀")
            return state.copy(update={
                "additional": {
                    **additional,
                    "next_step": "supervisor_agent",
                    "dynamic": "",
                    "classification": "",
                    "robots_pos": robots_pos,
                    "complete_tasks": complete_tasks,
                    "allocation_done": False,
                    "reasoning_done": False,
                    "validation_done": False,
                }
            })

        # 3) 🔴 여기: dynamic 이 비어있으면 서비스로부터 받아온다
        if not dynamic:
            # print("dynamic 없음 → finish 반환")
            return state.copy(update={
                "additional": {
                    **additional,
                    "next_step": "finish",
                }
            })

        if dynamic.lower() == "end":
            print("dynamic='end' → finish")
            return state.copy(update={
                "additional": {
                    **additional,
                    "next_step": "finish",
                    "dynamic": "end"
                }
            })

        if not additional.get("dynamic_time"):
            additional["dynamic_time"] = time.time()

        # 5) LLM 분류
        

    else:
        raise ValueError(f"❌ Unknown mode: {mode}")


supervisor_agent = RunnableLambda(supervisor_fn)
supervisor_agent.name = "supervisor_agent"
supervisor_agent.input_keys = ["mode", "task_description", "additional"]
