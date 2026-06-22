"""amhs 패키지 단위/통합 테스트. 실행: cd server && python -m pytest tests -q  (또는 python tests/test_amhs.py)"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amhs.graph import RailGraph, dijkstra, all_pairs           # noqa: E402
from amhs.geometry import (turn_for, abs_dir, relative_turn,     # noqa: E402
                           N, E, S, W, STRAIGHT, RIGHT, UTURN, LEFT)
from amhs.dispatch import hungarian, greedy                      # noqa: E402
from amhs.traffic import Traffic                                 # noqa: E402
from amhs.router import static_route                             # noqa: E402
from amhs.sim import GridAMHS                                    # noqa: E402
from amhs.navigator import (plan, fw_commands, final_heading,    # noqa: E402
                            MODE_STRAIGHT, MODE_RIGHT, MODE_LEFT, MODE_UTURN)
import fleet_grid_server as fgs                                  # noqa: E402


# ── 그래프 ──
def test_grid_structure():
    g = RailGraph(4, 4)
    assert len(g.stations) == 16
    assert set(g.adj[0]) == {1, 4}            # 코너: 이웃 2개
    assert set(g.adj[5]) == {1, 4, 6, 9}      # 내부: 이웃 4개
    assert g.nid(2, 3) == 11 and g.rc(11) == (2, 3)


def test_dijkstra_shortest():
    g = RailGraph(4, 4)
    p = dijkstra(g, 0, 15)                     # (0,0)→(3,3)
    assert p[0] == 0 and p[-1] == 15
    assert len(p) - 1 == 6                     # 맨해튼 거리 3+3
    # 인접 노드만 거치는지
    for a, b in zip(p, p[1:]):
        assert b in g.adj[a]


def test_dijkstra_blocked_reroute():
    g = RailGraph(3, 3)
    full = dijkstra(g, 0, 8)
    # 중앙(4) 막으면 길이는 같아도 4를 피해야
    p = dijkstra(g, 0, 8, blocked={4})
    assert 4 not in p
    assert p and p[0] == 0 and p[-1] == 8


def test_all_pairs():
    g = RailGraph(4, 4)
    D = all_pairs(g)
    assert D[0][15] == 6
    assert D[5][5] == 0


# ── 기하(heading→회전) ──
def test_turns():
    g = RailGraph(4, 4)
    # 노드5=(1,1). 동(6), 서(4), 북(1), 남(9)
    assert abs_dir(g, 5, 6) == E
    assert abs_dir(g, 5, 1) == N
    assert abs_dir(g, 5, 9) == S
    assert abs_dir(g, 5, 4) == W
    # 북을 보고 동으로 = 우회전
    assert relative_turn(N, E) == RIGHT
    assert relative_turn(N, W) == LEFT
    assert relative_turn(N, N) == STRAIGHT
    assert relative_turn(N, S) == UTURN
    # turn_for 가 (회전, 새 heading) 반환
    turn, d = turn_for(g, N, 5, 6)
    assert turn == RIGHT and d == E
    turn, d = turn_for(g, E, 5, 1)             # 동을 보고 북 = 좌회전
    assert turn == LEFT and d == N


# ── 배차 ──
def test_hungarian_optimal():
    # 최적 할당이 대각선(합=0)
    cost = [[0, 9, 9], [9, 0, 9], [9, 9, 0]]
    assert hungarian(cost) == [0, 1, 2]
    # greedy 가 손해 보는 케이스에서 hungarian 이 전역 최적
    cost = [[1, 2], [2, 100]]
    a = hungarian(cost)                         # 0→1(2), 1→0(2) 총4  vs greedy 0→0(1),1→1(100)
    assert a == [1, 0]


def test_hungarian_rectangular():
    cost = [[5, 1, 9]]                          # 로봇1 작업3
    a = hungarian(cost)
    assert a == [1]                             # 가장 싼 작업1


# ── 예약 ──
def test_traffic_reservation():
    t = Traffic()
    assert t.reserve(5, 0) is True
    assert t.reserve(5, 1) is False            # 점유 중
    assert t.reserve(5, 0) is True             # 본인은 OK
    t.release(5, 0)
    assert t.reserve(5, 1) is True
    assert t.blocked_set(0, dest=9) == {5}     # 1이 5 점유, dest 제외


# ── 라우터 ──
def test_static_route_excludes_current():
    g = RailGraph(4, 4)
    r = static_route(g, 0, 2)
    assert r == [1, 2]                          # 현재 노드 0 제외


# ── 통합: 시뮬 ──
def test_sim_deterministic():
    a = GridAMHS(seed=42).run(500)
    b = GridAMHS(seed=42).run(500)
    assert a == b                              # 동일 seed → 동일 결과


def test_sim_completes_tasks_no_collision():
    sim = GridAMHS(rows=4, cols=4, n_veh=3, seed=7, rate=0.15)
    sim.run(2000)                              # step() 내부에서 점유 불변식 assert
    k = sim.kpis()
    assert k["completed"] > 50                 # 작업이 실제로 처리됨
    assert k["completion_rate"] > 0.5
    # 점유 노드 수 = 로봇 수 (한 노드 한 대)
    assert len(sim.traffic.occ) == len(sim.veh)


def test_sim_no_permanent_gridlock():
    # 높은 부하에서도 데드락 회복으로 계속 처리되는지
    sim = GridAMHS(rows=3, cols=3, n_veh=3, seed=3, rate=0.4)
    sim.run(1500)
    assert sim.completed > 20


# ── navigator: 경로 → 회전명령 ──
def test_navigator_plan_turns():
    g = RailGraph(4, 4)
    # 0→15 를 북쪽 보고 시작: 우회전으로 동진 시작
    route = static_route(g, 0, 15)               # [1,2,3,7,11,15]
    steps = plan(g, 0, N, route)
    assert [s.to for s in steps] == [1, 2, 3, 7, 11, 15]
    assert steps[0].turn_name == "RIGHT"         # 북→동 = 우회전
    assert steps[1].turn_name == "STRAIGHT"
    # fw 명령은 mode 번호열
    assert fw_commands(steps) == [s.fw_mode for s in steps]
    assert MODE_STRAIGHT in fw_commands(steps) and MODE_RIGHT in fw_commands(steps)
    assert final_heading(steps, N) == S          # 마지막엔 남쪽


def test_navigator_uturn():
    g = RailGraph(4, 4)
    # 5(1,1) 에서 북(N) 보고 있는데 남(9)으로 = 유턴
    steps = plan(g, 5, N, [9])
    assert steps[0].turn_name == "UTURN"
    assert steps[0].fw_mode == MODE_UTURN


def test_navigator_empty_route():
    g = RailGraph(4, 4)
    assert plan(g, 5, N, []) == []
    assert final_heading([], N) == N


# ── 격자 관제 서버(GridFleet) — 하드웨어 없이 동기 구동 ──
class _SyncDriver:
    """send() 를 받아 결정론적으로 '도착 보고'를 되먹여 GridFleet 을 끝까지 돌린다."""
    def __init__(self):
        self.inbox = []
        self.sent = []           # (rid, mode) 전부 기록

    def send(self, rid, mode, speed):
        self.inbox.append((rid, mode))
        self.sent.append((rid, mode))

    def run(self, fleet, max_steps=20000):
        steps = 0
        while self.inbox and steps < max_steps:
            rid, mode = self.inbox.pop(0)
            steps += 1
            # 펌웨어 흉내: STOP→IDLE(무동작), 그 외→다음 노드 도착(WAIT_NODE)
            state = fgs.ST_IDLE if mode == fgs.MODE_STOP else fgs.ST_WAIT_NODE
            fleet.on_status(rid, state, 0, 0)
        return steps


def _occ_invariant(fleet):
    occ = fleet.traffic.occ
    return len(set(occ.values())) == len(occ)    # 한 노드 한 대


def test_gridfleet_single_robot_reaches_dest():
    d = _SyncDriver()
    fleet = fgs.GridFleet(4, 4, {1: 0}, d.send)
    fleet.goto(1, 15)
    used = d.run(fleet)
    assert used < 20000
    assert fleet.robots[1].pos == 15            # 목적지 도착
    assert fleet.robots[1].dest is None         # 작업 종료
    assert _occ_invariant(fleet)
    assert (1, fgs.MODE_STOP) in d.sent          # 도착 시 정지 명령


def test_gridfleet_three_robots_no_collision():
    d = _SyncDriver()
    starts = {1: 0, 2: 3, 3: 12}                 # 세 코너
    fleet = fgs.GridFleet(4, 4, starts, d.send)
    fleet.goto(1, 15)                            # 0 → 15 (대각)
    fleet.goto(2, 12)                            # 3 → 12 (대각, 1과 교차)
    fleet.goto(3, 3)                             # 12 → 3 (2와 정반대)
    used = d.run(fleet)
    assert used < 20000, "수렴 실패(데드락 가능)"
    assert _occ_invariant(fleet)                 # 충돌 0 (예약 불변식)
    for rid, dest in {1: 15, 2: 12, 3: 3}.items():
        assert fleet.robots[rid].pos == dest, f"로봇{rid} 미도착: {fleet.robots[rid].pos}"


def test_gridfleet_reroute_around_block():
    d = _SyncDriver()
    # 로봇2를 5에 세워 두고(목적지 없음), 로봇1이 그 노드를 피해가는지
    fleet = fgs.GridFleet(4, 4, {1: 4, 2: 5}, d.send)
    fleet.goto(1, 6)                             # 4→6 최단은 4-5-6 인데 5가 점유됨
    used = d.run(fleet)
    assert used < 20000
    assert fleet.robots[1].pos == 6
    # 5는 로봇2가 계속 점유 → 로봇1 경로에 5가 없어야
    assert _occ_invariant(fleet)


def test_gridfleet_deadlock_retreat():
    """사방의 생산적 경로가 막히면 인접 빈 노드로 후퇴(_recover)하는지 — 직접 검증.
    3x3 중앙(4)에서 0으로 가야 하는데 0의 유일한 이웃 1,3을 유휴 로봇이 점유 → 후퇴만 가능."""
    d = _SyncDriver()
    fleet = fgs.GridFleet(3, 3, {1: 4, 2: 1, 3: 3}, d.send)
    fleet.goto(1, 0)                              # RUN 전송(R2,R3은 유휴 블로커)
    # 출발 보고 1회 + 임계치까지 폴링(허브 연속 폴링 흉내) → _recover 가 후퇴 명령을 내림
    for _ in range(fgs.DEADLOCK_THRESH + 2):
        fleet.on_status(1, fgs.ST_WAIT_NODE, 0, 0)
    r = fleet.robots[1]
    # 막힌 중앙(4)을 떠나 빈 이웃(5 또는 7)으로 실제 후퇴했는지(durable outcome)
    assert r.pos in (5, 7), f"후퇴 안 함: pos={r.pos}"
    assert _occ_invariant(fleet)                 # 후퇴 중에도 한 노드 한 대
    assert any(rid == 1 and mode not in (fgs.MODE_STOP, fgs.MODE_RUN)
               for rid, mode in d.sent)          # 후퇴 회전명령이 실제로 나감


def test_gridfleet_mission_pickup_deliver():
    """A→B 운반 임무: 적재지 경유 후 하역지 도착."""
    d = _SyncDriver()
    fleet = fgs.GridFleet(4, 4, {1: 0}, d.send)
    fleet.assign_mission(1, 5, 15)               # 적재 5 → 하역 15
    used = d.run(fleet)
    assert used < 20000
    assert fleet.robots[1].pos == 15             # 하역지 도착
    assert fleet.robots[1].mission is None        # 임무 종료
    assert _occ_invariant(fleet)


def test_gridfleet_dispatch_hungarian():
    """여러 A→B 작업을 유휴 로봇에 Hungarian 최적 배차 → 전원 완료."""
    d = _SyncDriver()
    fleet = fgs.GridFleet(4, 4, {1: 0, 2: 3}, d.send)
    fleet.dispatch([(1, 13), (2, 14)])           # 두 작업, 두 로봇
    used = d.run(fleet)
    assert used < 20000
    assert all(fleet.robots[r].mission is None for r in (1, 2))   # 둘 다 완료
    assert _occ_invariant(fleet)


from amhs.timing import time_route, NUDGE_MS, DRIVE_MS, TURN_MS   # noqa: E402


def test_time_route_minimizes_turns():
    g = RailGraph(4, 4)
    path, ms, turns = time_route(g, 0, 10, E)    # 동쪽 보고 0→10
    assert path[0] == 0 and path[-1] == 10
    assert turns == 1                            # L자(회전1) 선택, 지그재그(회전3) 회피
    assert ms == 3 * (NUDGE_MS + DRIVE_MS) + (NUDGE_MS + TURN_MS + DRIVE_MS)


def test_time_route_start_heading_matters():
    g = RailGraph(4, 4)
    _, ms_e, t_e = time_route(g, 0, 3, E)        # 이미 동쪽 → 직진 3칸
    _, ms_n, t_n = time_route(g, 0, 3, N)        # 북쪽 → 출발에서 우회전 필요
    assert t_e == 0 and t_n == 1
    assert ms_e < ms_n


def test_gridfleet_time_routing_completes():
    d = _SyncDriver()
    fleet = fgs.GridFleet(4, 4, {1: 0}, d.send, routing="time")
    fleet.goto(1, 15)
    used = d.run(fleet)
    assert used < 20000 and fleet.robots[1].pos == 15 and _occ_invariant(fleet)


if __name__ == "__main__":
    import traceback
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    passed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
            passed += 1
        except Exception:
            print(f"FAIL  {fn.__name__}")
            traceback.print_exc()
    print(f"\n{passed}/{len(fns)} passed")
    sys.exit(0 if passed == len(fns) else 1)
