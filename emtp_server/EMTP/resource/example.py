"""
💡 Final Output Format (MUST keep this JSON schema)

- Dependencies는 오직 "공간(room) - 객체클래스(object_class)" 기준으로만 표기한다. (action 미포함)
- 비교대상이 없을 때는 first, 두 대상의 순서를 비교할 때는 before/after를 사용한다.
- room / object_class 는 특정 값 또는 "all" 을 사용할 수 있다.
- dependencies는 명시적/강한 암시의 우선순위 신호가 Instruction 또는 Constraints에 **존재할 때만** 생성한다.
- 신호가 없으면 dependencies는 빈 배열 [] 로 둔다.
- 접근 제약은 자연어 혹은 constraints에서 명시되거나 알 수 있는 공간(노드) 또는 연결(엣지)에 대해서만 적는다.
- 제약 값은 `"UAV only" | "UGV only" | "" | "closed"` 네 가지로 정규화한다.

{
  "dependencies": [
    { "first":  { "room": "<room_id|all>", "object_class": "<class|all>" } },
    { "before": { "room": "<room_id|all>", "object_class": "<class|all>" },
      "after":  { "room": "<room_id|all>", "object_class": "<class|all>" } }
  ],
  "node_restrictions": {
    "<room_id>": "UAV only | UGV only | | closed"
  },
  "edge_restrictions": {
    "<room_id>-<room_id>": "UAV only | UGV only | | closed"
  },
  "task_loss": ["<object_name>", ...],
  "restrictions_explain": "<short rationale>"
}

🔎 우선순위 신호 예시 (존재할 때만 dependencies 생성):
- "먼저", "우선", "…보다 먼저", "이후에", "다음에", "first", "before", "prioritize", "precede", "order", "순서"
- 애매하면 안전하게: dependencies = [] 로 두고, 필요 시 restrictions_explain에 모호성 짧게 기재.


──────────────────────────────────────────────────────────────────────────────
🧾 Example 1: (객체클래스 우선 / 전역) — “소화전 먼저, 소화기는 나중”

Task description: "소화전부터 하고 소화기는 그 다음에 해줘."
Relevant Object Classes: ["소화전", "소화기"]
Sub Tasks: ["inspect 소화전", "inspect 소화기"]

Constraints or Dynamics:
- (해당 없음)  *우선순위 신호는 Instruction에 존재*
- 접근 제약 관련 언급 없음 → node/edge 기록 안 함

✅ Final output:
{
  "dependencies": [
    {
      "before": { "room": "all", "object_class": "소화전" },
      "after":  { "room": "all", "object_class": "소화기" }
    }
  ],
  "node_restrictions": {},
  "edge_restrictions": {},
  "task_loss": [],
  "restrictions_explain": ""
}


──────────────────────────────────────────────────────────────────────────────
🧾 Example 2: (공간 우선 / 비교대상 있음) — “A 구역을 B보다 먼저”

Task description: "A 구역을 B 구역보다 먼저 처리해."
Relevant Object Classes: ["box", "crate", "floor"]
Sub Tasks: ["inspect box", "inspect crate", "clean floor"]

Constraints or Dynamics:
- 접근 제약 관련 언급 없음

✅ Final output:
{
  "dependencies": [
    {
      "before": { "room": "A", "object_class": "all" },
      "after":  { "room": "B", "object_class": "all" }
    }
  ],
  "node_restrictions": {},
  "edge_restrictions": {},
  "task_loss": [],
  "restrictions_explain": ""
}


──────────────────────────────────────────────────────────────────────────────
🧾 Example 3: (공간 우선 / 비교대상 없음) — “A 구역 먼저”

Task description: "A 구역 먼저 처리해줘."
Relevant Object Classes: ["box", "crate"]
Sub Tasks: ["inspect box", "inspect crate"]

Constraints or Dynamics:
- 접근 제약 관련 언급 없음

✅ Final output:
{
  "dependencies": [
    { "first": { "room": "A", "object_class": "all" } }
  ],
  "node_restrictions": {},
  "edge_restrictions": {},
  "task_loss": [],
  "restrictions_explain": ""
}


──────────────────────────────────────────────────────────────────────────────
🧾 Example 4: (공간+객체 / 비교대상 없음) — “A 구역의 소화기부터”

Task description: "A 구역에 있는 소화기부터 먼저 해줘."
Relevant Object Classes: ["소화기", "소화전"]
Sub Tasks: ["inspect 소화기", "inspect 소화전"]

Constraints or Dynamics:
- 접근 제약 관련 언급 없음

✅ Final output:
{
  "dependencies": [
    { "first": { "room": "A", "object_class": "소화기" } }
  ],
  "node_restrictions": {},
  "edge_restrictions": {},
  "task_loss": [],
  "restrictions_explain": ""
}


──────────────────────────────────────────────────────────────────────────────
🧾 Example 5: (공간클래스+객체클래스 / 비교대상 없음) — “화장실의 소화기부터”

(여기서 방 이름이 자연어로만 나왔다고 치고, constraints 에도 그대로 나온다고 가정)

Task description: "화장실에 있는 소화기부터 먼저 해줘."
Relevant Object Classes: ["소화기", "소화전"]
Sub Tasks: ["inspect 소화기", "inspect 소화전"]

Constraints or Dynamics:
- 접근 제약 관련 언급 없음

✅ Final output:
{
  "dependencies": [
    { "first": { "room": "A", "object_class": "소화기" } },
    { "first": { "room": "B", "object_class": "소화기" } }
  ],
  "node_restrictions": {},
  "edge_restrictions": {},
  "task_loss": [],
  "restrictions_explain": "화장실 A,B 구역의 소화기를 모두 먼저 처리해야 함."
}


──────────────────────────────────────────────────────────────────────────────
🧾 Example 6: (공간+객체 / 비교대상 있음) — “A 구역 소화기 → B 구역 소화전”

Task description: "A 구역의 소화기를 B 구역의 소화전보다 먼저 처리해."
Relevant Object Classes: ["소화기", "소화전"]
Sub Tasks: ["inspect 소화기", "inspect 소화전"]

Constraints or Dynamics:
- 접근 제약 관련 언급 없음

✅ Final output:
{
  "dependencies": [
    {
      "before": { "room": "A", "object_class": "소화기" },
      "after":  { "room": "B", "object_class": "소화전" }
    }
  ],
  "node_restrictions": {},
  "edge_restrictions": {},
  "task_loss": [],
  "restrictions_explain": ""
}


──────────────────────────────────────────────────────────────────────────────
🧾 Example 7: (원문 예시 변환) — “Inspect box before square” + 방 일부 제한

Task description: "Inspect the box first, then inspect the square."
Relevant Object Classes: ["box", "square"]
Sub Tasks: ["inspect box", "inspect square"]

Constraints or Dynamics:
- "B is under cleaning, only UAVs are allowed."

🧠 Explanation:
- 제약이 자연어로 B 구역만 언급 → node_restrictions 에만 기록
- 로봇 ID가 아니라 플랫폼을 말했으므로 "UAV only" 로 정규화

✅ Final output:
{
  "dependencies": [
    {
      "before": { "room": "all", "object_class": "box" },
      "after":  { "room": "all", "object_class": "square" }
    }
  ],
  "node_restrictions": {
    "B": "UAV only"
  },
  "edge_restrictions": {},
  "task_loss": [],
  "restrictions_explain": "Room B allows only UAVs according to the constraint text."
}


──────────────────────────────────────────────────────────────────────────────
🧾 Example 8: (원문 예시 변환) — “Inspect table before cleaning the floor” + 방 일부 제한

Task description: "Inspect the table, then clean the floor."
Relevant Object Classes: ["table", "floor"]
Sub Tasks: ["inspect table", "clean floor"]

Constraints or Dynamics:
- "B is under cleaning, only UAVs are allowed."

✅ Final output:
{
  "dependencies": [
    {
      "before": { "room": "all", "object_class": "table" },
      "after":  { "room": "all", "object_class": "floor" }
    }
  ],
  "node_restrictions": {
    "B": "UAV only"
  },
  "edge_restrictions": {},
  "task_loss": [],
  "restrictions_explain": "Room B is restricted to UAVs."
}


──────────────────────────────────────────────────────────────────────────────
🧾 Example 9: (원문 예시 변환) — 우선순위 없음 + 방 전체 폐쇄

Task description: "Inspect all the boxes and the crates."
Relevant Object Classes: ["box", "crate"]
Sub Tasks: ["inspect box", "inspect crate"]

Constraints or Dynamics:
- "A_crate1 is broken"
- "B is under cleaning, robots aren't allowed"

🧠 Explanation:
- 두 번째 문장은 B 구역이 전면 금지라는 뜻 → node_restrictions["B"] = "closed" 로 명시한다.
  우리는 완전 폐쇄된 방의 객체를 실제로는 못하니까 `task_loss` 에도 기록한다.
✅ Final output:
{
  "dependencies": [],
  "node_restrictions": {
    "B": "closed"
  },
  "edge_restrictions": {},
  "task_loss": ["A_crate1", "B_box2", "B_crate2"],
  "restrictions_explain": "Room B allows no robots, so all objects in that room are treated as inaccessible."
}


──────────────────────────────────────────────────────────────────────────────
🧾 Example 11: (모호성 처리) — 우선순위 신호 애매 → dependencies 비움

Task description: "가능하면 소화전 먼저 고려하되 상황에 따라 소화기를 해도 된다."
Relevant Object Classes: ["소화전", "소화기"]
Sub Tasks: ["inspect 소화전", "inspect 소화기"]

Constraints or Dynamics:
- (해당 없음)
- 표현이 권고 수준

✅ Final output:
{
  "dependencies": [],
  "node_restrictions": {},
  "edge_restrictions": {},
  "task_loss": [],
  "restrictions_explain": "Priority phrasing is advisory/ambiguous; no strict ordering applied."
}


──────────────────────────────────────────────────────────────────────────────
🧾 Example 12: (엣지 제약만 있는 경우) — “H8-H10 통로는 UAV만”

Task description: "H10에 있는 소화기를 점검해."
Relevant Object Classes: ["fireextinguisher"]
Sub Tasks: ["inspect fireextinguisher"]

Constraints or Dynamics:
- "H8과 H10 사이는 UAV만 다닐 수 있다."

🧠 Explanation:
- 제약에 노드 언급 없음 → node_restrictions 비움
- 엣지에만 언급 있음 → edge_restrictions["H8-H10"] = "UAV only"

✅ Final output:
{
  "dependencies": [],
  "node_restrictions": {},
  "edge_restrictions": {
    "H8-H10": "UAV only"
  },
  "task_loss": [],
  "restrictions_explain": "Edge H8-H10 is UAV only according to the constraint text."
}
──────────────────────────────────────────────────────────────────────────────
🧾 Example 13: 우선순위 없음 + 방 전체 제외

Task description: "Inspect all the boxes and the crates."
Relevant Object Classes: ["box", "crate"]
Sub Tasks: ["inspect box", "inspect crate"]

Constraints or Dynamics:
- "A_crate1 is broken"
- "B 에 있는 것들은 내가 했어! 제외하고 해"

🧠 Explanation:
- 두 번째 문장은 B 구역에 있는 task 들을 제외하라는 뜻 -> node_restrictions 를 건들지마, 완전 폐쇄랑은 다르게 객체들을 제외하는거야.

✅ Final output:
{
  "dependencies": [],
  "node_restrictions": {},
  "edge_restrictions": {},
  "task_loss": ["A_crate1", "B_box2", "B_crate2"],
  "restrictions_explain": "Room B allows no robots, so all objects in that room are treated as inaccessible."
}
"""
