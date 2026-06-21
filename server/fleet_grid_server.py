#!/usr/bin/env python3
"""
AGV Fleet — 격자 관제 서버 (Layer 2: 작전참모, 격자 버전)
─────────────────────────────────────────────────────────────────────
fleet_server.py(8자, 교차로 1개)를 격자(4x4 등)로 확장한 서버.
robot_grid.ino + navigator.py 와 한 세트.

하는 일
  1. 각 로봇의 pose(노드/방위)를 서버가 추적한다(로봇은 자기 노드번호를 모름).
  2. 목적지를 받으면 최단경로 → navigator 로 '노드별 회전 명령'으로 펼친다.
  3. 로봇이 교차로에 도착해 ST_WAIT_NODE 를 보고하면, 계획대로 다음 회전을 내려보낸다.
  4. 3대일 때: 다음 노드를 traffic 예약으로 한 대만 점유 → 회피 재경로 → (그래도 막히면) 유턴 후퇴.
     ※ 예약/회피/회복 로직은 오프라인 시뮬(amhs.sim)에서 검증된 것과 같은 알고리즘.

제어 로직(GridFleet)은 시리얼과 분리돼 있어 하드웨어 없이도 테스트/시연 가능:
  - 실물:  python fleet_grid_server.py --port /dev/tty.usbserial-XXXX
  - 시연:  python fleet_grid_server.py --sim          (가짜 로봇으로 전체 스택 구동)
"""
import argparse
import json
import socket
import sys
import threading
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from amhs.graph import RailGraph, all_pairs                       # noqa: E402
from amhs.router import static_route                              # noqa: E402
from amhs.traffic import Traffic                                  # noqa: E402
from amhs.dispatch import hungarian                               # noqa: E402
from amhs.geometry import N, DIR_NAME                             # noqa: E402
from amhs.navigator import (plan, MODE_STOP, MODE_RUN, MODE_NAME) # noqa: E402

# 펌웨어 RobotSt 와 동일
ST_IDLE, ST_RUNNING, ST_WAIT_NODE, ST_NUDGE, ST_TURNING = 0, 1, 2, 3, 4
ST_NAME = {0: "IDLE", 1: "RUNNING", 2: "WAIT_NODE", 3: "NUDGE", 4: "TURNING"}

DEADLOCK_THRESH = 4         # 막힌 채 이만큼 펌프되면 회복(유턴 후퇴) 시도
PH_TO_SRC, PH_TO_DST = 0, 1 # A→B 운반 임무 단계: 적재지로 / 하역지로


class RobotState:
    __slots__ = ("pos", "heading", "dest", "steps", "pending_to",
                 "pending_heading", "waiting", "blocked", "mission", "phase")

    def __init__(self, pos, heading=N):
        self.pos = pos              # 서버가 믿는 현재 노드
        self.heading = heading      # 현재 방위
        self.dest = None            # 목적지(없으면 유휴)
        self.steps = []             # 남은 navigator Step 들
        self.pending_to = None      # 지금 이동 중인 목표 노드(정지 중이면 None)
        self.pending_heading = heading
        self.waiting = False        # 노드에서 명령 대기 중(ST_WAIT_NODE)
        self.blocked = 0            # 연속 막힘 횟수(데드락 감지)
        self.mission = None         # A→B 운반 임무 (src, dst) — 없으면 단순 이동
        self.phase = None           # PH_TO_SRC / PH_TO_DST


class GridFleet:
    """순수 제어 로직 — send(rid, mode, speed) 콜백으로만 바깥과 통신(테스트 용이)."""

    def __init__(self, rows, cols, starts, send, speed=150, on_event=None):
        self.g = RailGraph(rows, cols)
        self.D = all_pairs(self.g)          # 노드쌍 최단거리 — Hungarian 배차 비용
        self.send = send
        self.speed = speed
        self.on_event = on_event or (lambda m: None)
        self.traffic = Traffic()
        self.robots = {}
        for rid, start in starts.items():
            self.traffic.place(start, rid)
            self.robots[rid] = RobotState(start)
        self.lock = threading.Lock()

    # ── 한 로봇을 dest 로 출발시키는 공통 루틴(락은 호출자가 보유) ──
    def _start(self, rid, dest):
        r = self.robots[rid]
        r.dest = dest
        r.blocked = 0
        route = static_route(self.g, r.pos, dest, blocked=self.traffic.blocked_set(rid, dest))
        r.steps = plan(self.g, r.pos, r.heading, route)
        r.pending_to = None
        r.waiting = False
        # IDLE 탈출용 RUN. 로봇은 출발 노드에서 ST_WAIT_NODE 를 한 번 보고한다.
        self.send(rid, MODE_RUN, self.speed)

    # ── 단순 이동 지시 ──
    def goto(self, rid, dest):
        with self.lock:
            r = self.robots[rid]
            r.mission = None
            r.phase = None
            self._start(rid, dest)
            self.on_event(f"[지시] 로봇{rid}: {r.pos} → {dest}  "
                          f"경로 {[r.pos] + [s.to for s in r.steps]}")

    # ── A→B 운반 임무 배차(한 대) ──
    def assign_mission(self, rid, src, dst):
        with self.lock:
            r = self.robots[rid]
            r.mission = (src, dst)
            r.phase = PH_TO_SRC
            self._start(rid, src)          # 먼저 적재지(src)로
            self.on_event(f"[배차] 로봇{rid}: 적재 {src} → 하역 {dst} (현재 {r.pos})")

    # ── 여러 A→B 작업을 유휴 로봇에 Hungarian 최적 배차 ──
    def dispatch(self, tasks):
        """tasks = [(src, dst), ...]. 유휴 로봇에 '가장 가까운 적재지' 기준 최적 할당."""
        with self.lock:
            idle = [rid for rid, r in self.robots.items()
                    if r.dest is None and r.mission is None]
        if not idle or not tasks:
            return
        cost = [[self.D[self.robots[rid].pos].get(src, 1e9) for src, _ in tasks]
                for rid in idle]
        a = hungarian(cost)                # 로봇 i → 작업 a[i] (없으면 -1)
        self.on_event(f"[배차] Hungarian: 유휴 {idle} × 작업 {tasks}")
        for i, rid in enumerate(idle):
            j = a[i]
            if 0 <= j < len(tasks):
                src, dst = tasks[j]
                self.assign_mission(rid, src, dst)

    def stop(self, rid):
        with self.lock:
            r = self.robots[rid]
            r.dest = None
            r.steps = []
            r.pending_to = None
            r.waiting = False
            self.send(rid, MODE_STOP, 0)

    def stop_all(self):
        for rid in self.robots:
            self.stop(rid)

    # ── 로봇 상태 수신(허브 시리얼 / 시뮬 공통 진입점) ──
    def on_status(self, rid, state, node, obstacle):
        if rid not in self.robots:
            return
        if state == ST_WAIT_NODE:
            self._arrived(rid)
        self._pump()

    def _arrived(self, rid):
        with self.lock:
            r = self.robots[rid]
            if r.pending_to is not None:           # 이동을 마치고 목표 노드에 도착
                self.traffic.release(r.pos, rid)
                r.pos = r.pending_to
                r.heading = r.pending_heading
                if r.steps:
                    r.steps.pop(0)
                r.pending_to = None
                r.blocked = 0
            r.waiting = True                       # 이제 노드에서 명령 대기

    # ── 대기 중인 로봇에게 다음 회전 내려보내기(교통관리 핵심) ──
    def _pump(self):
        with self.lock:
            for rid, r in self.robots.items():
                if not r.waiting or r.pending_to is not None or r.dest is None:
                    continue

                if r.pos == r.dest:                         # 목표 노드 도착
                    if r.mission is not None and r.phase == PH_TO_SRC:
                        # 적재지 도착 → 하역지로 계속(정지하지 않고 재계획)
                        r.phase = PH_TO_DST
                        r.dest = r.mission[1]
                        r.steps = []
                        self.on_event(f"[적재] 로봇{rid} {r.pos} 적재 → {r.dest} 운반")
                    else:                                    # 단순이동 도착 or 하역 완료
                        self.send(rid, MODE_STOP, 0)
                        r.waiting = False
                        if r.mission is not None:
                            self.on_event(f"[하역] 로봇{rid} {r.dest} 도착·하역 완료")
                        else:
                            self.on_event(f"[완료] 로봇{rid} 목적지 {r.dest} 도착")
                        r.dest = None
                        r.mission = None
                        r.phase = None
                        continue

                if not r.steps:                             # 경로 소진(후퇴 등) → 재계획
                    route = static_route(self.g, r.pos, r.dest,
                                         blocked=self.traffic.blocked_set(rid, r.dest))
                    r.steps = plan(self.g, r.pos, r.heading, route)
                    if not r.steps:                         # 정말 길 없음 → 보류
                        r.blocked += 1
                        if r.blocked >= DEADLOCK_THRESH:
                            self._recover(rid, r)
                        continue

                nxt = r.steps[0].to
                if self.traffic.reserve(nxt, rid):          # 다음 노드 한 대만 점유
                    self._dispatch(rid, r, r.steps[0])
                    continue

                # 막힘 → 점유 노드 회피 재경로
                alt = static_route(self.g, r.pos, r.dest,
                                   blocked=self.traffic.blocked_set(rid, r.dest))
                if alt and self.traffic.reserve(alt[0], rid):
                    r.steps = plan(self.g, r.pos, r.heading, alt)
                    self.on_event(f"[재경로] 로봇{rid}: {nxt} 막힘 → 우회 {alt[0]}")
                    self._dispatch(rid, r, r.steps[0])
                    continue

                # 그래도 막힘 → 데드락 회복(인접 빈 노드로 후퇴)
                r.blocked += 1
                if r.blocked >= DEADLOCK_THRESH:
                    self._recover(rid, r)

    def _dispatch(self, rid, r, step):
        self.send(rid, step.fw_mode, self.speed)
        r.pending_to = step.to
        r.pending_heading = step.new_heading
        r.waiting = False
        r.blocked = 0
        self.on_event(f"  → 로봇{rid} {MODE_NAME[step.fw_mode]} "
                      f"({r.pos}→{step.to}, 향후 {DIR_NAME[step.new_heading]})")

    def _recover(self, rid, r):
        """사방이 막혀 못 갈 때: 인접 빈 노드로 후퇴해 사이클을 푼다(시뮬 _recover 와 동일 발상).
        후퇴해서 자기 노드를 비워주면 상대 로봇이 지나갈 수 있다. 도착 뒤 _pump 가 목적지까지 재계획."""
        for adj in self.g.adj[r.pos]:
            if adj != r.dest and self.traffic.reserve(adj, rid):
                detour = plan(self.g, r.pos, r.heading, [adj])   # 한 칸 후퇴 명령
                r.steps = detour                                  # 후퇴만; 도착 후 재계획
                self._dispatch(rid, r, detour[0])
                self.on_event(f"[회복] 로봇{rid} 데드락 → {adj} 로 후퇴 후 재계획")
                return
        self.on_event(f"[경고] 로봇{rid} 사방 막힘 — 다음 틱 재시도")
        r.blocked = 0

    def snapshot(self):
        with self.lock:
            return {
                "robots": {rid: {
                    "pos": r.pos, "heading": DIR_NAME[r.heading],
                    "dest": r.dest, "moving_to": r.pending_to,
                    "mission": r.mission,
                    "remaining": len(r.steps),
                    "status": "moving" if r.pending_to is not None
                              else ("waiting" if r.waiting else "idle"),
                } for rid, r in self.robots.items()},
                "occupied": dict(self.traffic.occ),
            }


# ─────────────────────────────────────────────────────────────────────
# 전송층 A — 실물(허브 USB 시리얼)
# ─────────────────────────────────────────────────────────────────────
class SerialTransport:
    def __init__(self, port, baud=115200):
        import serial  # pip install pyserial
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # 아두이노 리셋 대기

    def send(self, rid, mode, speed):
        self.ser.write(f"C {rid} {mode} {speed}\n".encode())

    def start_reader(self, fleet):
        def loop():
            while True:
                try:
                    raw = self.ser.readline().decode(errors="ignore").strip()
                except Exception:
                    continue
                if raw.startswith("S "):
                    p = raw.split()
                    if len(p) == 5:
                        fleet.on_status(int(p[1]), int(p[2]), int(p[3]), int(p[4]))
                elif raw.startswith("#"):
                    print("[허브]", raw)
        threading.Thread(target=loop, daemon=True).start()


# ─────────────────────────────────────────────────────────────────────
# 전송층 B — 시뮬(가짜 로봇). 토폴로지를 모르고 펌웨어처럼 '명령→도착보고'만 흉내.
#   MODE_RUN  → 출발노드에서 WAIT_NODE 1회
#   회전(2~5) → 잠시 뒤 다음 노드 도착 WAIT_NODE
#   MODE_STOP → IDLE
# 이렇게 하면 실제 제어 로직(GridFleet)을 하드웨어 없이 그대로 검증한다.
# ─────────────────────────────────────────────────────────────────────
class SimTransport:
    def __init__(self, rids, delay=0.25):
        self.fleet = None
        self.delay = delay
        self.rids = rids

    def bind(self, fleet):
        self.fleet = fleet

    def send(self, rid, mode, speed):
        if mode == MODE_STOP:
            threading.Timer(0.01, self.fleet.on_status, (rid, ST_IDLE, 0, 0)).start()
        elif mode == MODE_RUN:
            # 출발 노드에 앉아 있다가 "도착(대기)" 보고
            threading.Timer(self.delay, self.fleet.on_status,
                            (rid, ST_WAIT_NODE, 0, 0)).start()
        else:  # 회전 명령 → 한 칸 주행 후 다음 노드 도착 보고
            threading.Timer(self.delay, self.fleet.on_status,
                            (rid, ST_WAIT_NODE, 0, 0)).start()


# ── 외부 제어 소켓 (LLM/CLI 가 붙음) ──
def socket_server(fleet, host="127.0.0.1", port=8765):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port)); srv.listen(5)
    print(f"[소켓] 제어 포트 {host}:{port}")
    while True:
        conn, _ = srv.accept()
        threading.Thread(target=_client, args=(fleet, conn), daemon=True).start()


def _client(fleet, conn):
    f = conn.makefile("rwb")
    for line in f:
        try:
            req = json.loads(line.decode())
        except Exception:
            continue
        f.write((json.dumps(_dispatch(fleet, req), ensure_ascii=False) + "\n").encode())
        f.flush()


def _dispatch(fleet, req):
    act = req.get("action")
    try:
        if act == "status":
            return {"ok": True, **fleet.snapshot()}
        if act == "goto":
            fleet.goto(int(req["robot"]), int(req["dest"]))
            return {"ok": True}
        if act == "goto_all":
            for rid, dest in req["dests"].items():
                fleet.goto(int(rid), int(dest))
            return {"ok": True}
        if act == "mission":
            fleet.assign_mission(int(req["robot"]), int(req["src"]), int(req["dst"]))
            return {"ok": True}
        if act == "dispatch":
            fleet.dispatch([(int(s), int(d)) for s, d in req["tasks"]])
            return {"ok": True}
        if act == "stop":
            fleet.stop(int(req["robot"])); return {"ok": True}
        if act == "stop_all":
            fleet.stop_all(); return {"ok": True}
    except Exception as e:
        return {"ok": False, "error": str(e)}
    return {"ok": False, "error": "unknown action"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="허브 아두이노 시리얼 포트(실물)")
    ap.add_argument("--sim", action="store_true", help="가짜 로봇으로 시연(하드웨어 불필요)")
    ap.add_argument("--rows", type=int, default=4)
    ap.add_argument("--cols", type=int, default=4)
    ap.add_argument("--robots", type=int, default=1, help="로봇 수(1~3)")
    ap.add_argument("--speed", type=int, default=150)
    args = ap.parse_args()
    if not args.port and not args.sim:
        ap.error("--port 또는 --sim 중 하나는 필요")

    # 로봇을 격자에 고르게 초기 배치
    n = args.rows * args.cols
    rids = list(range(1, args.robots + 1))
    starts = {rid: (i * (n // max(1, args.robots))) % n for i, rid in enumerate(rids)}

    if args.sim:
        transport = SimTransport(rids)
        fleet = GridFleet(args.rows, args.cols, starts, transport.send,
                          speed=args.speed, on_event=print)
        transport.bind(fleet)
        print(f"[시뮬] {args.rows}x{args.cols} 격자, 로봇 {rids}, 시작위치 {starts}")
    else:
        transport = SerialTransport(args.port)
        fleet = GridFleet(args.rows, args.cols, starts, transport.send,
                          speed=args.speed, on_event=print)
        transport.start_reader(fleet)
        print(f"[실물] 포트 {args.port}, 로봇 {rids}, 시작위치 {starts}")

    threading.Thread(target=socket_server, args=(fleet,), daemon=True).start()

    print("명령: goto <rid> <dest> / mission <rid> <src> <dst> / "
          "dispatch <s1> <d1> <s2> <d2>... / status / stop <rid> / stopall / quit")
    while True:
        try:
            c = input("grid> ").strip().split()
        except (EOFError, KeyboardInterrupt):
            break
        if not c:
            continue
        if c[0] == "goto" and len(c) == 3:
            fleet.goto(int(c[1]), int(c[2]))
        elif c[0] == "mission" and len(c) == 4:
            fleet.assign_mission(int(c[1]), int(c[2]), int(c[3]))
        elif c[0] == "dispatch" and len(c) >= 3 and (len(c) - 1) % 2 == 0:
            fleet.dispatch([(int(c[i]), int(c[i + 1])) for i in range(1, len(c), 2)])
        elif c[0] == "status":
            print(json.dumps(fleet.snapshot(), ensure_ascii=False, indent=2))
        elif c[0] == "stop" and len(c) == 2:
            fleet.stop(int(c[1]))
        elif c[0] == "stopall":
            fleet.stop_all()
        elif c[0] == "quit":
            break
    fleet.stop_all()


if __name__ == "__main__":
    main()
