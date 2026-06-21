#!/usr/bin/env python3
"""
AGV Fleet — LLM 두뇌 (Layer 3: 사령관)

로컬 LLM(Ollama)이 자연어 지시를 받아 도구(tool)를 호출하고,
그 도구가 fleet_server 소켓으로 실제 명령을 보낸다.
LLM 은 '무엇을 할지'만 결정하고, 안전·실시간 제어는 아래 층(서버/펌웨어)이 담당한다.

사전 준비:
  1) Ollama 설치 → 모델 받기:  ollama pull qwen3:1.7b   (이미 있으면 생략)
  2) pip install ollama
  3) fleet_server.py 가 먼저 실행 중이어야 함 (소켓 127.0.0.1:8765)

실행:  python llm_orchestrator.py
예시 지시: "전부 출발", "2번 멈춰", "지금 상태 알려줘"
"""
import inspect, json, socket

try:
    import ollama  # pip install ollama
except ImportError:
    raise SystemExit("ollama 필요: pip install ollama  (그리고 Ollama 앱 실행)")

SERVER = ("127.0.0.1", 8765)
MODEL = "qwen3:1.7b"   # 이미 받아둔 가벼운 로컬 모델. 더 똑똑하게: 'qwen2.5' / 'gemma3:4b' 등으로 교체


def _call(req):
    s = socket.socket(); s.connect(SERVER)
    f = s.makefile("rwb")
    f.write((json.dumps(req) + "\n").encode()); f.flush()
    resp = json.loads(f.readline().decode())
    s.close()
    return resp


# ── LLM 이 호출할 수 있는 도구 ──
def get_fleet_status():           return _call({"action": "status"})
def run_all():                    return _call({"action": "run_all"})
def stop_all():                   return _call({"action": "stop_all"})
def set_robot(robot: int, mode: str):
    return _call({"action": "set_mode", "robot": robot, "mode": mode})

FUNCS = {"get_fleet_status": get_fleet_status, "run_all": run_all,
         "stop_all": stop_all, "set_robot": set_robot}

TOOLS = [
    {"type": "function", "function": {
        "name": "get_fleet_status", "description": "전체 로봇의 현재 상태 조회",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "run_all", "description": "모든 로봇 주행 시작",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "stop_all", "description": "모든 로봇 정지",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "set_robot", "description": "특정 로봇 한 대를 주행/정지",
        "parameters": {"type": "object", "properties": {
            "robot": {"type": "integer", "description": "로봇 번호 1~3"},
            "mode": {"type": "string", "enum": ["run", "stop"]}},
            "required": ["robot", "mode"]}}},
]

SYSTEM = ("너는 3대 AGV 로봇 함대의 사령관이다. 사용자의 자연어 지시를 도구 호출로 "
          "변환해 함대를 지휘한다. 안전이 최우선이며, 상황이 모호하면 먼저 "
          "get_fleet_status 로 상태를 확인한 뒤 판단한다.")


def main():
    msgs = [{"role": "system", "content": SYSTEM}]
    print("AGV 사령관 (LLM). 예: '전부 출발' / '2번 멈춰' / '상태'. quit 종료.")
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

        calls = msg.get("tool_calls") or []
        for call in calls:
            fn = call["function"]["name"]
            args = call["function"].get("arguments") or {}
            # 작은 모델이 불필요한 인자를 붙여도 크래시 안 나게 시그니처로 필터
            valid = set(inspect.signature(FUNCS[fn]).parameters)
            kwargs = {k: v for k, v in args.items() if k in valid}
            result = FUNCS[fn](**kwargs)
            msgs.append({"role": "tool", "content": json.dumps(result, ensure_ascii=False)})

        if calls:
            follow = ollama.chat(model=MODEL, messages=msgs, think=False)
            print(follow["message"]["content"]); msgs.append(follow["message"])
        else:
            print(msg.get("content", ""))


if __name__ == "__main__":
    main()
