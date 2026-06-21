#!/usr/bin/env python3
"""
AGV Fleet — LLM 두뇌 (격자, Claude Code CLI 서브프로세스 버전)

API 키/종량과금 대신 **Claude 구독(Max/Pro)** 으로 동작한다.
이미 로그인된 `claude` CLI 를 헤드리스(-p, print 모드)로 서브프로세스 호출해,
자연어 지시를 '행동 JSON'으로 번역시킨 뒤 그 행동을 fleet_grid_server 소켓에 실행한다.

  사람:  "1번 12번에서 3번으로 운반, 2번 15번으로"
  claude -p (구독):  {"actions":[{"tool":"assign_mission","robot":1,"src":12,"dst":3},
                                 {"tool":"send_robot_to","robot":2,"dest":15}]}
  서버 :  경로/회전/교차로 예약/배차는 아래 층이 처리.

세 가지 두뇌 비교
  llm_grid.py         로컬 Ollama(qwen3:1.7b) — 오프라인·무료, 작은 모델
  llm_grid_claude.py  Anthropic API           — 똑똑함, 종량 과금(API 키 필요)
  llm_grid_cli.py     Claude 구독 CLI(이 파일) — 똑똑함 + 구독 한도 내 무료, API 키 불필요

장점: 추가 비용 없음(구독 한도 내), API 키 불필요. 주의: 호출당 수 초 지연, `claude` 로그인 필요.

사전 준비
  1) Claude Code CLI 설치 + 로그인(평소 쓰던 그대로면 OK)
  2) fleet_grid_server.py 실행 중   (python fleet_grid_server.py --sim --robots 3)

실행:  python llm_grid_cli.py
"""
import json
import re
import socket
import subprocess

SERVER = ("127.0.0.1", 8765)
CLAUDE = "claude"   # CLI 경로(PATH 에 없으면 절대경로로)

PROMPT = """너는 격자(4x4, 노드 0~15) 위 AGV 로봇 3대 함대의 사령관이다.
아래 사용자 지시를 '행동 목록 JSON' 하나로만 변환해 출력한다(설명·코드펜스 없이 JSON만).
사용 가능한 도구:
- {{"tool":"send_robot_to","robot":<1-3>,"dest":<0-15>}}                단순 이동
- {{"tool":"assign_mission","robot":<1-3>,"src":<0-15>,"dst":<0-15>}}   A→B 운반(적재 src→하역 dst)
- {{"tool":"stop_robot","robot":<1-3>}}                                 한 대 정지
- {{"tool":"stop_all"}}                                                 전부 정지
- {{"tool":"get_fleet_status"}}                                         상태 조회
형식: {{"actions":[ ... ]}}

사용자 지시: {order}
JSON:"""


def _call(req):
    s = socket.socket(); s.connect(SERVER)
    f = s.makefile("rwb")
    f.write((json.dumps(req) + "\n").encode()); f.flush()
    r = json.loads(f.readline().decode()); s.close()
    return r


# 행동(JSON) → 서버 소켓 요청
ACTION_TO_REQ = {
    "send_robot_to":   lambda a: {"action": "goto", "robot": a["robot"], "dest": a["dest"]},
    "assign_mission":  lambda a: {"action": "mission", "robot": a["robot"], "src": a["src"], "dst": a["dst"]},
    "stop_robot":      lambda a: {"action": "stop", "robot": a["robot"]},
    "stop_all":        lambda a: {"action": "stop_all"},
    "get_fleet_status": lambda a: {"action": "status"},
}


def ask_claude(order):
    """구독 CLI 를 헤드리스로 호출해 행동 목록을 얻는다."""
    prompt = PROMPT.format(order=order)
    out = subprocess.run([CLAUDE, "-p", prompt, "--output-format", "json"],
                         capture_output=True, text=True, timeout=120)
    raw = out.stdout.strip()
    # --output-format json 래퍼면 result 필드 추출, 아니면 stdout 그대로
    try:
        wrapped = json.loads(raw)
        text = wrapped.get("result", raw) if isinstance(wrapped, dict) else raw
    except json.JSONDecodeError:
        text = raw
    m = re.search(r"\{.*\}", text, re.S)   # 첫 JSON 블록 추출
    if not m:
        raise ValueError(f"JSON 응답 못 찾음: {text[:200]}")
    return json.loads(m.group(0)).get("actions", [])


def main():
    print("AGV 사령관(Claude 구독 CLI). 예: '1번 12번에서 3번으로 운반' / '전부 정지' / '상태'. quit 종료.")
    while True:
        try:
            order = input("사령관에게> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if order in ("quit", "exit"):
            break
        try:
            actions = ask_claude(order)
        except Exception as e:
            print("  ! 해석 실패:", e)
            continue
        for a in actions:
            make_req = ACTION_TO_REQ.get(a.get("tool"))
            if not make_req:
                print("  ? 알 수 없는 도구:", a)
                continue
            print(f"  · {a} → {_call(make_req(a))}")


if __name__ == "__main__":
    main()
