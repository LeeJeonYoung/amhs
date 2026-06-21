#!/usr/bin/env python3
"""
AGV Fleet — 관제 서버 (Layer 2: 작전참모)

역할:
  - 허브 아두이노(USB 시리얼)와 통신하며 로봇 3대의 상태를 추적
  - 교차로 교통관리(구역 락): 한 번에 한 대만 교차로 통과
  - localhost 소켓(JSON)으로 외부 제어(LLM 오케스트레이터 / 다른 CLI)를 받음
  - 내장 CLI 로 직접 run / stop / status 가능

실행:
  python fleet_server.py --port /dev/tty.usbserial-XXXX
  (포트 확인:  ls /dev/tty.* )
"""
import argparse, json, socket, threading, time

try:
    import serial  # pip install pyserial
except ImportError:
    raise SystemExit("pyserial 필요: pip install pyserial")

# ── 펌웨어와 동일한 상수 ──
MODE_STOP, MODE_RUN, MODE_GO_THROUGH = 0, 1, 2
ST_IDLE, ST_RUNNING, ST_PAUSED_OBSTACLE, ST_WAIT_NODE = 0, 1, 2, 3
STATE_NAME = {0: "IDLE", 1: "RUNNING", 2: "PAUSED_OBSTACLE", 3: "WAIT_NODE"}
NUM_ROBOTS = 3


class Fleet:
    def __init__(self, ser):
        self.ser = ser
        self.lock = threading.Lock()
        self.cmd = {i: {"mode": MODE_STOP, "speed": 150} for i in range(1, NUM_ROBOTS + 1)}
        self.status = {i: {"state": ST_IDLE, "node": 0, "obstacle": 0, "ts": 0}
                       for i in range(1, NUM_ROBOTS + 1)}
        # figure-8 트랙은 교차로 1개 → 점유 중인 로봇 id (없으면 None)
        self.intersection_owner = None
        self.granted = set()   # 통과 허가(GO_THROUGH) 내려준 로봇

    # ── 허브로 명령 송신 ──
    def _send(self, rid):
        c = self.cmd[rid]
        self.ser.write(f"C {rid} {c['mode']} {c['speed']}\n".encode())

    def set_mode(self, rid, mode, speed=None):
        with self.lock:
            self.cmd[rid]["mode"] = mode
            if speed is not None:
                self.cmd[rid]["speed"] = speed
        self._send(rid)

    def run_all(self):
        for i in range(1, NUM_ROBOTS + 1):
            self.set_mode(i, MODE_RUN)

    def stop_all(self):
        for i in range(1, NUM_ROBOTS + 1):
            self.set_mode(i, MODE_STOP)

    # ── 로봇 상태 수신 시 호출 ──
    def on_status(self, rid, state, node, obstacle):
        with self.lock:
            self.status[rid] = {"state": state, "node": node,
                                "obstacle": obstacle, "ts": time.time()}
        self._traffic(rid, state)

    def _traffic(self, rid, state):
        """교차로 구역 락: 비어있으면 점유+통과허가, 통과 끝나면 해제."""
        if state == ST_WAIT_NODE:
            with self.lock:
                if self.intersection_owner is None:
                    self.intersection_owner = rid
                    self.granted.add(rid)
                    self.cmd[rid]["mode"] = MODE_GO_THROUGH
                    self._send(rid)
                    print(f"[교통] 교차로 점유 → 로봇 {rid} 통과 허가")
                # 점유 중이면 그대로 대기 (로봇은 WAIT_NODE 에서 멈춰 있음)
        elif state == ST_RUNNING and rid in self.granted:
            with self.lock:
                self.granted.discard(rid)
                if self.intersection_owner == rid:
                    self.intersection_owner = None
                    self.cmd[rid]["mode"] = MODE_RUN
                    self._send(rid)
                    print(f"[교통] 로봇 {rid} 통과 완료 → 교차로 해제")

    def snapshot(self):
        with self.lock:
            return {
                "robots": {i: {"state": STATE_NAME[self.status[i]["state"]],
                               "node": self.status[i]["node"],
                               "obstacle": self.status[i]["obstacle"],
                               "mode": self.cmd[i]["mode"]} for i in self.status},
                "intersection_owner": self.intersection_owner,
            }


# ── 허브 시리얼 읽기 스레드 ──
def serial_reader(fleet, ser):
    while True:
        try:
            raw = ser.readline().decode(errors="ignore").strip()
        except Exception:
            continue
        if not raw:
            continue
        if raw.startswith("S "):
            p = raw.split()
            if len(p) == 5:
                fleet.on_status(int(p[1]), int(p[2]), int(p[3]), int(p[4]))
        elif raw.startswith("#"):
            print("[허브]", raw)


# ── 외부 제어 소켓 (LLM/CLI 가 붙음) ──
def socket_server(fleet, host="127.0.0.1", port=8765):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port)); srv.listen(5)
    print(f"[소켓] 제어 포트 열림 {host}:{port}")
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
        f.write((json.dumps(_dispatch(fleet, req)) + "\n").encode()); f.flush()


def _dispatch(fleet, req):
    act = req.get("action")
    if act == "status":
        return {"ok": True, **fleet.snapshot()}
    if act == "run_all":
        fleet.run_all(); return {"ok": True}
    if act == "stop_all":
        fleet.stop_all(); return {"ok": True}
    if act == "set_mode":
        m = {"stop": MODE_STOP, "run": MODE_RUN}.get(req.get("mode"), MODE_STOP)
        fleet.set_mode(int(req["robot"]), m); return {"ok": True}
    return {"ok": False, "error": "unknown action"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="허브 아두이노 시리얼 포트")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2)  # 아두이노 리셋 대기
    fleet = Fleet(ser)
    threading.Thread(target=serial_reader, args=(fleet, ser), daemon=True).start()
    threading.Thread(target=socket_server, args=(fleet,), daemon=True).start()

    print("AGV Fleet 서버 시작. 명령: run / stop / status / go <id> / halt <id> / quit")
    while True:
        try:
            c = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if c == "run":
            fleet.run_all()
        elif c == "stop":
            fleet.stop_all()
        elif c == "status":
            print(json.dumps(fleet.snapshot(), ensure_ascii=False, indent=2))
        elif c.startswith("go "):
            fleet.set_mode(int(c.split()[1]), MODE_RUN)
        elif c.startswith("halt "):
            fleet.set_mode(int(c.split()[1]), MODE_STOP)
        elif c == "quit":
            break
    fleet.stop_all(); ser.close()


if __name__ == "__main__":
    main()
