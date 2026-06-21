"""
격자 AMHS 오프라인 시뮬 — 하드웨어 없이 두뇌(경로+배차+예약+데드락)를 검증.

이동 모델: '노드 스텝'. 매 tick, 이동 중 로봇은 다음 노드를 예약할 수 있으면 한 칸 전진.
  → 실물의 '노드에서 정지→예약 받으면 회전·전진' 과 같은 의미라, 여기서 통과하면
    예약/데드락 로직이 실물에서도 그대로 성립한다.
적재/하역은 dwell(틱) 로 모델링. 모든 무작위는 seed 기반(재현 가능).
"""
from .graph import RailGraph, all_pairs
from .router import static_route, congestion_route
from .dispatch import hungarian, greedy
from .traffic import Traffic
from .geometry import turn_for, TURN_NAME, N
from .rng import RNG

IDLE, TO_SRC, LOADING, TO_DST, UNLOADING = range(5)
STATE_NAME = {IDLE: "IDLE", TO_SRC: "TO_SRC", LOADING: "LOADING",
              TO_DST: "TO_DST", UNLOADING: "UNLOADING"}


class Vehicle:
    __slots__ = ("id", "pos", "heading", "state", "task", "dest",
                 "path", "blocked", "proc", "last_turn")

    def __init__(self, vid, pos):
        self.id = vid
        self.pos = pos
        self.heading = N
        self.state = IDLE
        self.task = None
        self.dest = None
        self.path = []
        self.blocked = 0
        self.proc = 0
        self.last_turn = None


class GridAMHS:
    def __init__(self, rows=4, cols=4, n_veh=3, seed=1,
                 dispatcher="hungarian", routing="static", rate=0.15,
                 p_hot=0.1, w1=0.5, hot_bonus=1000.0, alpha=0.5,
                 load_ticks=2, unload_ticks=2, window=30,
                 deadlock_thresh=8):
        self.g = RailGraph(rows, cols)
        self.D = all_pairs(self.g)
        self.traffic = Traffic()
        self.rng = RNG(seed)
        self.disp = dispatcher
        self.routing = routing
        self.rate = rate
        self.p_hot, self.w1, self.hot_bonus, self.alpha = p_hot, w1, hot_bonus, alpha
        self.load_ticks, self.unload_ticks = load_ticks, unload_ticks
        self.window, self.deadlock_thresh = window, deadlock_thresh

        # 로봇을 station 에 고르게 배치
        S = self.g.stations
        step = max(1, len(S) // n_veh)
        self.veh = []
        for i in range(n_veh):
            pos = S[(i * step) % len(S)]
            while self.traffic.owner(pos) is not None:   # 충돌 방지
                pos = S[(pos + 1) % len(S)]
            v = Vehicle(i, pos)
            self.traffic.place(pos, i)
            self.veh.append(v)

        self.pending = []
        self.next_tid = 0
        self.now = 0
        # KPI
        self.generated = self.completed = self.assigned = 0
        self.sum_cyc = self.sum_q = 0.0
        self.deadlocks = self.reroutes = self.blocked_ticks = self.busy = 0
        self.turn_log = []   # (now, vid, turn_name) — 실물로 보낼 회전 명령 추적

    # ── 작업 생성 ──
    def gen_tasks(self):
        k = self.rng.poisson(self.rate)
        S = self.g.stations
        for _ in range(k):
            a = self.rng.randint(0, len(S))
            b = self.rng.randint(0, len(S) - 1)
            if b >= a:
                b += 1
            hot = self.rng.random() < self.p_hot
            self.pending.append({"id": self.next_tid, "src": S[a], "dst": S[b],
                                 "hot": hot, "created": self.now, "assigned": -1})
            self.next_tid += 1
            self.generated += 1

    # ── 경로 ──
    def _edge_load(self):
        el = {}
        for v in self.veh:
            if v.path:
                e = (v.pos, v.path[0])
                el[e] = el.get(e, 0) + 1
        return el

    def route(self, src, dst, blocked=None):
        if self.routing == "congestion":
            return congestion_route(self.g, src, dst, self._edge_load(),
                                    self.alpha, blocked=blocked)
        return static_route(self.g, src, dst, blocked=blocked)

    # ── 이동(노드 스텝) ──
    def _advance(self, v, nxt):
        """nxt 는 이미 예약된 인접 노드. 회전 기록 + 위치/heading 갱신."""
        turn, d = turn_for(self.g, v.heading, v.pos, nxt)
        v.last_turn = turn
        self.turn_log.append((self.now, v.id, TURN_NAME[turn]))
        self.traffic.release(v.pos, v.id)
        v.heading = d
        v.pos = nxt
        v.path.pop(0)
        v.blocked = 0

    def _move(self, v):
        if not v.path:
            self._phase(v)
            return
        nxt = v.path[0]
        if self.traffic.reserve(nxt, v.id):
            self._advance(v, nxt)
            if not v.path:
                self._phase(v)
            return
        # 막힘 → 점유 노드 회피 재경로
        alt = self.route(v.pos, v.dest, blocked=self.traffic.blocked_set(v.id, v.dest))
        if alt and self.traffic.reserve(alt[0], v.id):
            v.path = alt
            self.reroutes += 1
            self._advance(v, alt[0])
            if not v.path:
                self._phase(v)
            return
        v.blocked += 1
        self.blocked_ticks += 1

    def _phase(self, v):
        if v.state == TO_SRC and v.pos == v.dest and not v.path:
            v.state = LOADING
            v.proc = self.load_ticks
        elif v.state == TO_DST and v.pos == v.dest and not v.path:
            v.state = UNLOADING
            v.proc = self.unload_ticks

    def _proc(self, v):
        v.proc -= 1
        if v.proc > 0:
            return
        if v.state == LOADING:
            v.dest = v.task["dst"]
            v.path = self.route(v.pos, v.dest)
            v.state = TO_DST
            self._phase(v)
        elif v.state == UNLOADING:
            cyc = self.now - v.task["created"]
            self.completed += 1
            self.sum_cyc += cyc
            v.task = None
            v.dest = None
            v.path = []
            v.state = IDLE

    # ── 데드락 회복(실물용 yield/retreat) ──
    def _recover(self):
        for v in self.veh:
            if v.blocked <= self.deadlock_thresh or not v.path:
                continue
            # 1) 전체 점유 회피 재경로 한 번 더
            alt = self.route(v.pos, v.dest, blocked=self.traffic.blocked_set(v.id, v.dest))
            if alt and self.traffic.free(alt[0], v.id):
                v.path = alt
                v.blocked = 0
                self.reroutes += 1
                self.deadlocks += 1
                continue
            # 2) 인접 빈 노드로 후퇴(유턴) → 사이클 해소
            for adj in self.g.adj[v.pos]:
                if self.traffic.reserve(adj, v.id):
                    self.traffic.release(v.pos, v.id)
                    turn, d = turn_for(self.g, v.heading, v.pos, adj)
                    v.heading = d
                    v.pos = adj
                    v.path = self.route(v.pos, v.dest)
                    v.blocked = 0
                    self.deadlocks += 1
                    break
            else:
                v.blocked = 0   # 사방 막힘 → 다음 tick 재시도

    # ── 배차 ──
    def _candidates(self):
        arr = sorted(self.pending,
                     key=lambda t: (not t["hot"], t["created"], t["id"]))
        return arr[:self.window]

    def dispatch(self):
        idle = [v for v in self.veh if v.state == IDLE]
        if not idle or not self.pending:
            return
        cand = self._candidates()
        pairs = []
        if self.disp == "greedy":
            pairs = greedy(idle, cand, lambda v, t: self.D[v.pos][t["src"]])
        else:
            cost = []
            for v in idle:
                row = []
                for t in cand:
                    c = self.D[v.pos][t["src"]] - self.w1 * (self.now - t["created"])
                    if t["hot"]:
                        c -= self.hot_bonus
                    row.append(c)
                cost.append(row)
            a = hungarian(cost)
            for i, v in enumerate(idle):
                j = a[i]
                if 0 <= j < len(cand):
                    pairs.append((v, cand[j]))
        for v, t in pairs:
            v.task = t
            v.dest = t["src"]
            v.state = TO_SRC
            t["assigned"] = self.now
            self.assigned += 1
            self.sum_q += (self.now - t["created"])
            v.path = self.route(v.pos, v.dest)
            self.pending.remove(t)
            self._phase(v)

    # ── 한 틱 ──
    def step(self):
        self.gen_tasks()
        for v in self.veh:
            if v.state in (TO_SRC, TO_DST):
                self._move(v)
            elif v.state in (LOADING, UNLOADING):
                self._proc(v)
        self._recover()
        self.dispatch()
        for v in self.veh:
            if v.state != IDLE:
                self.busy += 1
        # 점유 불변식 체크(디버그): 노드당 최대 1대
        assert len(set(self.traffic.occ.values())) == len(self.traffic.occ), \
            "점유 불변식 위반: 한 노드 다중 점유"
        self.now += 1

    def run(self, ticks):
        for _ in range(ticks):
            self.step()
        return self.kpis()

    def kpis(self):
        T = max(1, self.now)
        Nv = len(self.veh)
        return {
            "ticks": self.now,
            "generated": self.generated,
            "completed": self.completed,
            "throughput_per_1k": round(self.completed / T * 1000, 2),
            "avg_cycle": round(self.sum_cyc / self.completed, 2) if self.completed else 0,
            "avg_queue": round(self.sum_q / self.assigned, 2) if self.assigned else 0,
            "completion_rate": round(self.completed / self.generated, 3) if self.generated else 0,
            "utilization": round(self.busy / (Nv * T), 3),
            "deadlocks": self.deadlocks,
            "reroutes": self.reroutes,
            "block_ratio": round(self.blocked_ticks / (Nv * T), 3),
        }

    def render(self):
        """격자 + 로봇 위치 텍스트 렌더(디버그)."""
        cell = {}
        for v in self.veh:
            cell[v.pos] = str(v.id)
        out = []
        for r in range(self.g.rows):
            row = []
            for c in range(self.g.cols):
                n = self.g.nid(r, c)
                row.append(cell.get(n, "·"))
            out.append(" ".join(row))
        return "\n".join(out)
