#!/usr/bin/env python3
"""
라인트레이서 1대의 주행을 '로그성 데이터'로 풀어서 보여주는 추적기 (하드웨어 0%).

서버가 실제로 계산하는 navigator.plan() 을 그대로 써서, 무선 프로토콜과
펌웨어 상태머신(라인추종→교차로감지→정지→회전)을 한 줄씩 풀어쓴다.

무선 프로토콜
  Mac→로봇 :  C <id> <mode> <speed>     mode 0정지/1주행/2직진/3우회전/4유턴/5좌회전
  로봇→Mac :  S <id> <state> <node> 0   state 0대기/1주행/2교차로도착/3중앙진입/4회전중

예) python trace_robot.py --from 0 --to 15 --heading N
"""
import argparse
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from amhs.graph import RailGraph                                      # noqa: E402
from amhs.router import static_route                                 # noqa: E402
from amhs.geometry import N, E, S, W, DIR_NAME                       # noqa: E402
from amhs.navigator import plan, MODE_NAME                           # noqa: E402

HEADING = {"N": N, "E": E, "S": S, "W": W}
DIR_KO = {0: "북(↑)", 1: "동(→)", 2: "남(↓)", 3: "서(←)"}
TURN_KO = {"STRAIGHT": "직진 통과", "RIGHT": "오른쪽 90° 회전",
           "UTURN": "유턴 180°", "LEFT": "왼쪽 90° 회전"}
# robot_grid.ino 실주행 확정 튜닝값
NUDGE_MS, TURN_MS, UTURN_MS, INTERSECT_MS = 150, 650, 1300, 50


def rc(g, n):
    r, c = g.rc(n)
    return f"{n}({r},{c})"


def trace(rows, cols, start, heading, dest):
    g = RailGraph(rows, cols)
    route = static_route(g, start, dest)
    steps = plan(g, start, heading, route)

    print(f"\n{'='*64}")
    print(f" 로봇1 주행 추적 — {rc(g,start)} → {rc(g,dest)}   (격자 {rows}x{cols})")
    print(f" 시작 방위: {DIR_KO[heading]}   목표까지 교차로 {len(steps)}개")
    print(f" 계산된 경로: {' → '.join(rc(g,n) for n in [start]+[s.to for s in steps])}")
    print(f"{'='*64}\n")

    t = 0
    def log(arrow, msg):
        nonlocal t
        print(f"[{t:5d}ms] {arrow} {msg}")

    # 출발 — IDLE 탈출
    log("Mac→로봇", "C 1 1 150     (RUN: 주행 시작 지시)")
    log("로봇   ", "상태 IDLE→RUNNING. 모터 ON, 라인 추종 시작")
    t += 50
    log("로봇   ", f"출발 노드 {rc(g,start)} 가 교차로라 즉시 감지 → 정지")
    log("로봇→Mac", "S 1 2 1 0     (WAIT_NODE: 교차로 도착, 명령 대기)")

    cur, h = start, heading
    for i, s in enumerate(steps, 1):
        turn = s.turn_name
        dur = UTURN_MS if turn == "UTURN" else (0 if turn == "STRAIGHT" else TURN_MS)
        print()
        log("Mac   ", f"계획표: 이 교차로에선 '{TURN_KO[turn]}'. 다음 노드 {rc(g,s.to)} 예약 OK")
        log("Mac→로봇", f"C 1 {s.fw_mode} 150     (mode={s.fw_mode} {MODE_NAME[s.fw_mode]})")
        t += 10
        log("로봇   ", f"교차로 중앙까지 {NUDGE_MS}ms 전진(NUDGE)")
        t += NUDGE_MS
        if turn == "STRAIGHT":
            log("로봇   ", "→ 직진 그대로 통과, 라인 재포착")
        else:
            log("로봇   ", f"→ 제자리 {TURN_KO[turn]} {dur}ms (TURNING) → 라인 재포착")
            t += dur
        # 다음 교차로까지 라인 추종
        h = s.new_heading
        nxt = "목적지" if i == len(steps) else "다음 교차로"
        log("로봇   ", f"이제 {DIR_KO[h]} 보고 라인 추종 → {nxt}({rc(g,s.to)}) 로 직진")
        t += 400  # 한 칸 주행(가정)
        if i < len(steps):
            log("로봇   ", "양쪽 IR 센서 동시 검정 감지 "
                          f"({INTERSECT_MS}ms 지속) → 교차로 확정, 정지")
            log("로봇→Mac", f"S 1 2 {i+1} 0     (WAIT_NODE: {i+1}번째 교차로 도착)")
        cur = s.to

    print()
    log("로봇   ", f"목적지 {rc(g,dest)} 도착 (라인 위 정지)")
    log("로봇→Mac", "S 1 2 ? 0     (WAIT_NODE)")
    log("Mac   ", "계획표 소진 = 목적지 도착 확인")
    log("Mac→로봇", "C 1 0 0       (STOP: 정지)")
    log("로봇   ", "상태 RUNNING→IDLE. 모터 OFF. 임무 완료 ✓")
    print(f"\n 요약: 교차로 {len(steps)}개 통과 / "
          f"회전 {sum(1 for s in steps if s.turn_name!='STRAIGHT')}회 "
          f"({', '.join(TURN_KO[s.turn_name] for s in steps if s.turn_name!='STRAIGHT') or '없음'}) / "
          f"총 {t}ms (라인추종 시간은 거리 가정값)\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rows", type=int, default=4)
    ap.add_argument("--cols", type=int, default=4)
    ap.add_argument("--from", dest="src", type=int, default=0)
    ap.add_argument("--to", dest="dst", type=int, default=15)
    ap.add_argument("--heading", choices=list(HEADING), default="N")
    a = ap.parse_args()
    trace(a.rows, a.cols, a.src, HEADING[a.heading], a.dst)


if __name__ == "__main__":
    main()
