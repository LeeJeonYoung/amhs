#!/usr/bin/env python3
"""
AGV Fleet — LLM 두뇌 (격자, Claude 버전, Layer 3: 사령관)

llm_grid.py(로컬 Ollama)와 같은 일을 하되 두뇌를 **Claude(Anthropic API)** 로 바꾼 것.
3대 격자 함대를 자연어로 지휘한다. 도구 호출로 fleet_grid_server 소켓에 명령을 꽂는다.

  사람:  "1번은 12번 노드에서 3번 노드로 운반, 2번은 15번으로 보내"
  Claude:  assign_mission(1,12,3)  send_robot_to(2,15)
  서버 :  최단경로·회전·교차로 예약·데드락 회복은 아래 층이 처리. Claude 는 '무엇을'만.

역할 분리(안전): Claude 는 실시간 모터/충돌제어에 끼지 않는다. '작업 의도'만 결정한다.
  Layer1 펌웨어=라인추종/회전(실시간) · Layer2 서버=경로·예약·배차 · Layer3 Claude=지휘.

사전 준비
  1) pip install anthropic
  2) export ANTHROPIC_API_KEY=...        (Claude API 키)
  3) fleet_grid_server.py 가 먼저 실행 중   (python fleet_grid_server.py --sim --robots 3)

실행:  python llm_grid_claude.py
"""
import json
import socket

try:
    import anthropic  # pip install anthropic
except ImportError:
    raise SystemExit("anthropic 필요: pip install anthropic  (그리고 ANTHROPIC_API_KEY 설정)")

SERVER = ("127.0.0.1", 8765)
MODEL = "claude-opus-4-8"   # 최신 Opus. 더 저렴하게: "claude-sonnet-4-6" / "claude-haiku-4-5"


def _call(req):
    s = socket.socket(); s.connect(SERVER)
    f = s.makefile("rwb")
    f.write((json.dumps(req) + "\n").encode()); f.flush()
    resp = json.loads(f.readline().decode())
    s.close()
    return resp


# ── 서버 소켓에 꽂는 실제 동작 ──
def get_fleet_status():                 return _call({"action": "status"})
def send_robot_to(robot, dest):         return _call({"action": "goto", "robot": robot, "dest": dest})
def assign_mission(robot, src, dst):    return _call({"action": "mission", "robot": robot, "src": src, "dst": dst})
def stop_robot(robot):                  return _call({"action": "stop", "robot": robot})
def stop_all():                         return _call({"action": "stop_all"})

FUNCS = {"get_fleet_status": get_fleet_status, "send_robot_to": send_robot_to,
         "assign_mission": assign_mission, "stop_robot": stop_robot, "stop_all": stop_all}

# ── Claude 도구 정의(Anthropic Messages API 스키마) ──
TOOLS = [
    {"name": "get_fleet_status",
     "description": "모든 로봇의 현재 노드 위치/목적지/임무/상태 조회",
     "input_schema": {"type": "object", "properties": {}}},
    {"name": "send_robot_to",
     "description": "특정 로봇을 목적지 노드로 이동(단순 이동). 경로/회전/교차로 양보는 서버가 처리.",
     "input_schema": {"type": "object", "properties": {
         "robot": {"type": "integer", "description": "로봇 번호 1~3"},
         "dest": {"type": "integer", "description": "목적지 노드 id(4x4면 0~15)"}},
         "required": ["robot", "dest"]}},
    {"name": "assign_mission",
     "description": "특정 로봇에 A→B 운반 임무 배차. src 에서 적재 후 dst 로 하역 운반.",
     "input_schema": {"type": "object", "properties": {
         "robot": {"type": "integer", "description": "로봇 번호 1~3"},
         "src": {"type": "integer", "description": "적재(픽업) 노드 id"},
         "dst": {"type": "integer", "description": "하역(목적) 노드 id"}},
         "required": ["robot", "src", "dst"]}},
    {"name": "stop_robot",
     "description": "특정 로봇 한 대 즉시 정지",
     "input_schema": {"type": "object", "properties": {
         "robot": {"type": "integer", "description": "로봇 번호 1~3"}}, "required": ["robot"]}},
    {"name": "stop_all", "description": "모든 로봇 즉시 정지",
     "input_schema": {"type": "object", "properties": {}}},
]

SYSTEM = (
    "너는 격자(4x4, 노드 0~15) 위 AGV 로봇 3대 함대의 사령관이다. "
    "사용자의 자연어 지시를 도구 호출로 바꿔 함대를 지휘한다. "
    "경로 계산·교차로 양보·충돌 회피·데드락 회복은 하위 서버가 처리하므로, 너는 '어느 로봇을 어디로/무엇을 운반' "
    "할지만 정하면 된다. 단순 이동은 send_robot_to, A→B 운반은 assign_mission 을 쓴다. "
    "안전이 최우선이고, 상황이 모호하면 먼저 get_fleet_status 로 위치를 확인한 뒤 판단한다. "
    "여러 대를 동시에 지시하면 각각 도구를 호출한다. 끝나면 한 줄로 무엇을 했는지 보고한다."
)


def main():
    client = anthropic.Anthropic()   # ANTHROPIC_API_KEY 환경변수 사용
    messages = []
    print("AGV 사령관(Claude). 예: '1번 12번에서 3번으로 운반, 2번 15번으로' / '전부 정지' / '상태'. quit 종료.")
    while True:
        try:
            u = input("사령관에게> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if u in ("quit", "exit"):
            break
        messages.append({"role": "user", "content": u})

        # 도구 호출이 끝날 때까지 도는 수동 에이전트 루프
        while True:
            resp = client.messages.create(model=MODEL, max_tokens=1024,
                                          system=SYSTEM, tools=TOOLS, messages=messages)
            for block in resp.content:
                if block.type == "text" and block.text.strip():
                    print(block.text)

            if resp.stop_reason != "tool_use":
                break

            messages.append({"role": "assistant", "content": resp.content})
            results = []
            for block in resp.content:
                if block.type == "tool_use":
                    try:
                        out = FUNCS[block.name](**block.input)
                    except Exception as e:
                        out = {"ok": False, "error": str(e)}
                    print(f"  · {block.name}({block.input}) → {out}")
                    results.append({"type": "tool_result", "tool_use_id": block.id,
                                    "content": json.dumps(out, ensure_ascii=False)})
            messages.append({"role": "user", "content": results})


if __name__ == "__main__":
    main()
