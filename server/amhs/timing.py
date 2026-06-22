"""
회전·이동 '시간'을 반영한 다익스트라.

홉수(거리) 최단 ≠ 시간 최단. 회전이 한 칸 주행보다 오래 걸리기 때문(90°=650ms vs 직진=400ms).
핵심 트릭: **상태를 (노드)에서 (노드, 바라보는 방위)로 확장**한다. 그러면 같은 노드라도
'어느 방향에서 들어왔나'에 따라 다음 회전 비용이 달라지는 걸 다익스트라가 반영할 수 있다.
'여러 변수 고려'도 같은 방식 — edge_cost 로 혼잡·우선구역 같은 추가 시간을 더 얹으면 된다.

시간 상수는 robot_grid.ino 실측 기반(ms). 속도가 바뀌면 여기만 고치면 된다.
"""
import heapq
from .geometry import abs_dir, relative_turn, STRAIGHT, UTURN

DRIVE_MS = 400     # 인접 교차로 사이 라인추종 1칸 주행
NUDGE_MS = 150     # 교차로 중앙 진입(매 노드)
TURN_MS  = 650     # 90° 제자리 회전 (좌/우)
UTURN_MS = 1300    # 180° 유턴


def step_time(turn):
    """회전코드(geometry) → 그 한 칸을 가는 데 드는 시간(ms)."""
    tt = 0 if turn == STRAIGHT else (UTURN_MS if turn == UTURN else TURN_MS)
    return NUDGE_MS + tt + DRIVE_MS


def time_route(g, src, dst, heading, blocked=None, edge_cost=None):
    """
    상태=(노드, 방위) 다익스트라로 '시간 최단' 경로를 찾는다.

    반환 (path, total_ms, turns)
      path  : 노드열(현재 src 포함, 도달 불가면 [])
      total_ms : 예상 소요시간(ms)
      turns : 회전 횟수(직진 제외)
    blocked  : 회피할 노드 집합(dst 는 허용)
    edge_cost(u, v): 선택. 혼잡 등 추가 시간(ms)을 더하고 싶을 때.
    """
    if src == dst:
        return [src], 0, 0
    blocked = blocked or set()
    dist = {(src, heading): 0}
    prev = {}
    pq = [(0, src, heading)]
    seen = set()
    while pq:
        d, u, h = heapq.heappop(pq)
        if (u, h) in seen:
            continue
        seen.add((u, h))
        if u == dst:
            path = [u]; turns = 0; cur = (u, h)
            while cur in prev:
                pstate, turn = prev[cur]
                if turn != STRAIGHT:
                    turns += 1
                path.append(pstate[0]); cur = pstate
            path.reverse()
            return path, d, turns
        for v in g.adj[u]:
            if v in blocked and v != dst:
                continue
            d_dir = abs_dir(g, u, v)
            turn = relative_turn(h, d_dir)
            nd = d + step_time(turn) + (edge_cost(u, v) if edge_cost else 0)
            st = (v, d_dir)
            if st not in dist or nd < dist[st]:
                dist[st] = nd
                prev[st] = ((u, h), turn)
                heapq.heappush(pq, (nd, v, d_dir))
    return [], float("inf"), 0


def route_time(g, src, dst, heading, blocked=None):
    """배차 비용용 — (src,heading) 에서 dst 까지 예상 소요시간(ms)만."""
    return time_route(g, src, dst, heading, blocked=blocked)[1]
