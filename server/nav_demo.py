#!/usr/bin/env python3
"""
교차로 회전결정 알고리즘 데모 — 하드웨어 불필요.

출발/도착 노드만 주면 (1) 최단경로 (2) 각 교차로에서 좌/우/직진/유턴 결정
(3) 실제로 로봇에 보낼 명령(mode) 까지 한눈에 출력한다.
면접에서 "어떻게 길을 알고리즘으로 정하나"를 노트북만으로 시연할 때 쓴다.

예)
  python nav_demo.py                       # 4x4, 0(좌상)→15(우하), 북쪽 보고 시작
  python nav_demo.py --from 0 --to 3 --heading E
  python nav_demo.py --blocked 5,6         # 막힌 노드 회피 재경로 시연
  python nav_demo.py --scenarios           # 대표 시나리오 묶음(데모용)
"""
import argparse
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from amhs.graph import RailGraph                                      # noqa: E402
from amhs.router import static_route                                 # noqa: E402
from amhs.geometry import N, E, S, W, DIR_NAME                       # noqa: E402
from amhs.navigator import plan, explain, fw_commands, final_heading # noqa: E402

HEADING = {"N": N, "E": E, "S": S, "W": W}


def show(g, start, heading, dst, blocked=None):
    route = static_route(g, start, dst, blocked=blocked)
    print(f"\n■ {start} → {dst}  (시작 방위 {DIR_NAME[heading]}"
          + (f", 막힌 노드 {sorted(blocked)}" if blocked else "") + ")")
    if not route:
        print("  경로 없음 (도달 불가)")
        return
    print(f"  최단경로 노드열 : {[start] + route}")
    print(explain(g, start, heading, route))
    steps = plan(g, start, heading, route)
    print(f"  → 로봇 전송 명령 순서(mode): {fw_commands(steps)}")
    print(f"  → 도착 후 방위           : {DIR_NAME[final_heading(steps, heading)]}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rows", type=int, default=4)
    ap.add_argument("--cols", type=int, default=4)
    ap.add_argument("--from", dest="src", type=int, default=0)
    ap.add_argument("--to", dest="dst", type=int, default=15)
    ap.add_argument("--heading", choices=list(HEADING), default="N")
    ap.add_argument("--blocked", default="", help="회피할 노드들, 콤마구분 예: 5,6")
    ap.add_argument("--scenarios", action="store_true", help="대표 시나리오 묶음 출력")
    args = ap.parse_args()

    g = RailGraph(args.rows, args.cols)
    print(f"격자 {args.rows}x{args.cols}  (노드 id = row*{args.cols}+col, 0=좌상단)")
    print("방위: N=위 E=오른쪽 S=아래 W=왼쪽  /  회전: STRAIGHT·RIGHT·UTURN·LEFT")

    if args.scenarios:
        show(g, 0, N, 15)                       # 대각선 횡단
        show(g, 0, E, 3)                        # 한 줄 직진
        show(g, 5, N, 6, blocked={2, 10})       # 회피 재경로
        show(g, 15, N, 0)                        # 반대로(유턴 포함 가능)
        return

    blocked = {int(x) for x in args.blocked.split(",") if x.strip()} or None
    show(g, args.src, HEADING[args.heading], args.dst, blocked=blocked)


if __name__ == "__main__":
    main()
