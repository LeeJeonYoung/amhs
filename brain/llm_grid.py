#!/usr/bin/env python3
"""
AGV Fleet — LLM 두뇌 (격자 버전, Layer 3: 사령관)

llm_orchestrator.py(8자: 출발/정지만)를 격자 관제(fleet_grid_server.py)에 맞게 확장.
자연어 지시 → 도구 호출 → 소켓으로 '목적지 배차'까지 지휘한다.

  사람:  "1번 로봇 15번 노드로, 2번은 12번으로 보내"
  LLM :  send_robot_to(1,15)  send_robot_to(2,12)
  서버 :  최단경로·회전명령·교차로 예약은 아래 층이 알아서. LLM 은 '무엇을'만 결정.

역할 분리(안전): LLM 은 실시간 모터/충돌제어에 절대 끼지 않는다.
  Layer1 펌웨어=라인추종/회전(실시간)  Layer2 서버=경로·교차로 예약(결정론)  Layer3 LLM=작업 의도.

사전 준비
  1) ollama pull qwen3:1.7b   (이미 있으면 생략) + pip install ollama
  2) fleet_grid_server.py 가 먼저 실행 중   (python fleet_grid_server.py --sim   또는 --port ...)

실행:  python llm_grid.py
"""
import inspect
import json
import socket

try:
    import ollama  # pip install ollama
except ImportError:
    raise SystemExit("ollama 필요: pip install ollama  (그리고 Ollama 앱 실행)")

SERVER = ("127.0.0.1", 8765)
MODEL = "qwen3:1.7b"   # 받아둔 가벼운 로컬 모델. 더 똑똑하게: 'qwen2.5'/'gemma3:4b' 등으로 교체


def _call(req):
    s = socket.socket(); s.connect(SERVER)
    f = s.makefile("rwb")
    f.write((json.dumps(req) + "\n").encode()); f.flush()
    resp = json.loads(f.readline().decode())
    s.close()
    return resp


# ── LLM 이 호출할 수 있는 도구 ──
def get_fleet_status():                 return _call({"action": "status"})
def send_robot_to(robot: int, dest: int):
    return _call({"action": "goto", "robot": robot, "dest": dest})
def stop_robot(robot: int):             return _call({"action": "stop", "robot": robot})
def stop_all():                         return _call({"action": "stop_all"})

FUNCS = {"get_fleet_status": get_fleet_status, "send_robot_to": send_robot_to,
         "stop_robot": stop_robot, "stop_all": stop_all}

TOOLS = [
    {"type": "function", "function": {
        "name": "get_fleet_status",
        "description": "모든 로봇의 현재 노드 위치/목적지/상태 조회",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "send_robot_to",
        "description": "특정 로봇을 목적지 노드로 보낸다(경로/회전/교차로 양보는 서버가 처리)",
        "parameters": {"type": "object", "properties": {
            "robot": {"type": "integer", "description": "로봇 번호 1~3"},
            "dest": {"type": "integer", "description": "목적지 노드 id(4x4면 0~15)"}},
            "required": ["robot", "dest"]}}},
    {"type": "function", "function": {
        "name": "stop_robot",
        "description": "특정 로봇 한 대 즉시 정지",
        "parameters": {"type": "object", "properties": {
            "robot": {"type": "integer", "description": "로봇 번호 1~3"}},
            "required": ["robot"]}}},
    {"type": "function", "function": {
        "name": "stop_all", "description": "모든 로봇 즉시 정지",
        "parameters": {"type": "object", "properties": {}}}},
]

SYSTEM = (
    "너는 격자(4x4, 노드 0~15) 위 AGV 로봇 함대의 사령관이다. "
    "사용자의 자연어 지시를 도구 호출로 바꿔 함대를 지휘한다. "
    "경로 계산·교차로 양보·충돌 회피는 하위 서버가 알아서 하므로 너는 '어느 로봇을 어디로' "
    "보낼지만 정하면 된다. 안전이 최우선이고, 상황이 모호하면 먼저 get_fleet_status 로 "
    "위치를 확인한 뒤 판단한다. 여러 대를 옮기라면 로봇마다 send_robot_to 를 각각 호출한다."
)


def main():
    msgs = [{"role": "system", "content": SYSTEM}]
    print("AGV 사령관(격자). 예: '1번 15번으로' / '2번 12번, 3번 3번으로' / '전부 정지' / '상태'. quit 종료.")
    while True:
        try:
            u = input("사령관에게> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if u in ("quit", "exit"):
            break
        msgs.append({"role": "user", "content": u})
        resp = ollama.chat(model=MODEL, messages=msgs, tools=TOOLS, think=False)
        msg = resp["message"]; msgs.append(msg)

        for call in (msg.get("tool_calls") or []):
            fn = call["function"]["name"]
            args = call["function"].get("arguments") or {}
            valid = set(inspect.signature(FUNCS[fn]).parameters)      # 작은 모델 군더더기 인자 필터
            kwargs = {k: v for k, v in args.items() if k in valid}
            try:
                result = FUNCS[fn](**kwargs)
            except Exception as e:
                result = {"ok": False, "error": str(e)}
            print(f"  · {fn}({kwargs}) → {result}")
            msgs.append({"role": "tool", "content": json.dumps(result, ensure_ascii=False)})

        if msg.get("tool_calls"):
            follow = ollama.chat(model=MODEL, messages=msgs, think=False)
            print(follow["message"]["content"]); msgs.append(follow["message"])
        else:
            print(msg.get("content", ""))


if __name__ == "__main__":
    main()
