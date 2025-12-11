# main.py
import os
import json
import time
from datetime import datetime
import argparse

from langgraph.graph import StateGraph
from pydantic import BaseModel, Field

# ✅ 패키지식 임포트
from EMTP.resource.supervisor_agent import supervisor_agent
from EMTP.resource.analyze_agent import analyze_agent
from EMTP.resource.reasoning_agent import reasoning_agent
from EMTP.resource.allocation_agent import allocation_agent
from common.robot import actions, robots
from common.token_survey import start as token_start, save as token_save
from common.instructions import make_instructions
from EMTP.resource.supervisor_agent import wait_for_dynamic


# -------------------------
# State 정의
# -------------------------
class StateSchema(BaseModel):
    mode: str = "offline"
    task_description: str
    object_list: list[str]
    robot_names: list
    robot_info: list[dict]
    skill_list: str = ""
    constraint: str = ""
    additional: dict = Field(default_factory=dict)

    class Config:
        extra = "allow"


state_schema = StateSchema


def supervisor_router(state):
    next_step = state.additional.get("next_step", "")
    if next_step == "analyze":
        return "analyze_agent"
    elif next_step == "reasoning":
        return "reasoning_agent"
    elif next_step == "allocation":
        return "allocation_agent"
    elif next_step in ("online", "supervisor_agent"):
        return "supervisor_agent"
    elif next_step == "finish":
        return "__end__"


# -------------------------
# LangGraph 빌드 함수
# -------------------------
def build_app():
    graph = StateGraph(state_schema=state_schema)
    graph.add_node("supervisor_agent", supervisor_agent)
    graph.add_node("analyze_agent", analyze_agent)
    graph.add_node("reasoning_agent", reasoning_agent)
    graph.add_node("allocation_agent", allocation_agent)

    graph.add_conditional_edges("supervisor_agent", supervisor_router)
    graph.add_edge("analyze_agent", "supervisor_agent")
    graph.add_edge("reasoning_agent", "supervisor_agent")
    graph.add_edge("allocation_agent", "supervisor_agent")

    graph.set_entry_point("supervisor_agent")
    graph.set_finish_point("supervisor_agent")

    app = graph.compile()
    return app

def save_plan_result(state, result_dir, elapsed_time):
    add = state.get("additional", {})
    final = add.get("final_result", {})
    env = state.get("environment", add.get("environment", "env"))

    flat_allocation = final.get("allocation", {})
    flat_path_len = final.get("path_len", {})
    flat_paths = final.get("paths", {})

    output_data = {
        "allocation_counts": {r: len(v) for r, v in flat_allocation.items()},
        "allocation_total": sum(len(v) for v in flat_allocation.values()),
        "allocation": flat_allocation,
        "path_len": flat_path_len,
        "paths": flat_paths,
        "total_time": round(elapsed_time, 3),
        "status": "success"
    }

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_filename = f"experiment_allocation_{ts}.json"
    result_path = os.path.join(result_dir, result_filename)
    with open(result_path, "w", encoding="utf-8") as f:
        json.dump(output_data, f, indent=2, ensure_ascii=False)
    
    # print(f"💾 allocation result saved → {result_path}")


# -------------------------
# 메인 실행부
# -------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--environment", type=str, default="college")
    parser.add_argument("--topic", type=str, default="fire")
    parser.add_argument("--level", type=str, default="hard")
    parser.add_argument("--task-type", type=str, default="test1")
    parser.add_argument("--task-indices", type=str, default="0")
    parser.add_argument("--constraint-indices", type=str, default="0")
    parser.add_argument("--evaluation", type=str, default="dynamic")
    parser.add_argument("--timestamp", type=str, default=None)
    parser.add_argument("--robot-count", type=int, default=None)
    args = parser.parse_args()

    task_indices = list(map(int, args.task_indices.split(",")))
    constraint_indices = list(map(int, args.constraint_indices.split(",")))

    # 지시사항 생성
    instructions, selected_constraints, robot_names = make_instructions(
        environment=args.environment,
        topic=args.topic,
        level=args.level,
        task_type=args.task_type,
        task_indices=task_indices,
        constraint_indices=constraint_indices,
        evaluation=args.evaluation,
        robot_count=args.robot_count
    )
    task_description = instructions[0]

    # 환경 파일 로드
    base_dir = os.path.expanduser("~/emtp_ws/src/emtp_server/common")
    env_path = os.path.join(base_dir, "env", args.environment, f"{args.environment}_{args.level}.json")
    
    with open(env_path, encoding="utf-8") as f:
        full_env = json.load(f)

    object_classes = set()
    for room in full_env.get("rooms", []):
        for obj in room.get("objects", []):
            c = obj.get("class")
            if c:
                object_classes.add(c)
    obj_data = sorted(list(object_classes))

    robot_info = [r for r in robots if r["name"] in robot_names]

    # 결과 저장
    if args.timestamp is None:
        args.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    timestamp = args.timestamp
    
    
        
    base_dir = os.path.expanduser("~/emtp_ws/src/emtp_server/result")
    result_dir = os.path.join(base_dir, f"experiment_{timestamp}")
    os.makedirs(result_dir, exist_ok=True)
    filename = None

    os.environ["RESULT_DIR"] = result_dir
    os.environ["TOKEN_LOG_FILE"] = os.path.join(result_dir, "token.json")
    
    # 초기 상태 설정
    initial_input = {
        "mode": "offline",
        "task_description": task_description,
        "object_list": obj_data,
        "robot_names": robot_names,
        "robot_skills": actions,
        "robot_info": robot_info,
        "constraint": ", ".join(selected_constraints),
        "environment": args.environment,
        "level": args.level,
        "additional": {
            "next_step": "analyze",
            "environment": args.environment,
            "level": args.level,
            "robots_pos": {},
            "complete_tasks": [],
            "dynamic": "",
            "result_dir": result_dir, 
        }
    }

    # Token 초기화
    token_start()


    # LangGraph 실행
    app = build_app()
    
    
    start_time = time.time()
    state = app.invoke(initial_input)
    elapsed_time = time.time() - start_time
    save_plan_result(state, result_dir, elapsed_time)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    initial_token_file = os.path.join(
        result_dir, 
        f"experiment_token_{ts}.json"
    )
    token_save(initial_token_file)
    while True:
        print(">>> waiting dynamic...")
        dyn = wait_for_dynamic()
        dynamic = dyn["value"]

        if dynamic == "end":
            print("### END RECEIVED — STOP ###")
            break
        token_start()
        start_time = time.time()
        state = app.invoke({
            **state,
            "additional": {
                **state.get("additional", {}),
                "reasoning_done": False,
                "allocation_done": False,
                "dynamic": dynamic,
                "robots_pos": dyn["robots_pos"],
                "complete_tasks": dyn["complete_tasks"],
                "next_step": "supervisor_agent"
            }
        })
        elapsed_time = time.time() - start_time
        save_plan_result(state, result_dir, elapsed_time)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        cycle_token_file = os.path.join(
            result_dir,
            f"experiment_token_{ts}.json"
        )
        token_save(cycle_token_file)
    import rclpy
    if rclpy.ok():
        print("🧹 Shutting down global ROS context...")
        rclpy.shutdown()
if __name__ == "__main__":
    main()
