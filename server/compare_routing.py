#!/usr/bin/env python3
"""
홉(거리) 최단 vs 시간(회전 포함) 최단 — 같은 출발/도착인데 결과가 어떻게 달라지나.
실행: python compare_routing.py
"""
import sys
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from amhs.graph import RailGraph                                  # noqa: E402
from amhs.router import static_route                              # noqa: E402
from amhs.geometry import N, E, S, W, DIR_NAME, turn_for, TURN_NAME  # noqa: E402
from amhs.timing import time_route, step_time                     # noqa: E402

g = RailGraph(4, 4)

def path_time(path, heading):
    """주어진 경로(노드열)를 그 방위로 갈 때 총 시간/회전수."""
    ms, turns, h, cur = 0, 0, heading, path[0]
    for nxt in path[1:]:
        turn, h = turn_for(g, h, cur, nxt)
        ms += step_time(turn)
        if TURN_NAME[turn] != "STRAIGHT":
            turns += 1
        cur = nxt
    return ms, turns

def show(src, dst, heading):
    hop = [src] + static_route(g, src, dst)            # 홉 최단(다익스트라, 회전 무시)
    tpath, tms, tturns = time_route(g, src, dst, heading)  # 시간 최단(회전 반영)
    hms, hturns = path_time(hop, heading)
    print(f"\n■ {src} → {dst}  (출발 방위 {DIR_NAME[heading]})")
    print(f"  홉 최단 : {hop}   회전 {hturns}회 · 예상 {hms}ms")
    print(f"  시간최단: {tpath}   회전 {tturns}회 · 예상 {tms}ms"
          + (f"   ⟸ {hms-tms}ms 빠름" if tms < hms else "   (동일)"))

show(0, 10, S)    # ★ 같은 4홉인데 홉최단은 회전2, 시간최단은 회전1 → 갈림
show(2, 12, E)    # ★ 홉최단이 유턴까지 끼면 시간 차이가 큼
show(0, 15, E)    # 둘이 같은 경우(이미 직선 방향)
print()
