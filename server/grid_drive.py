#!/usr/bin/env python3
"""
격자 주행 시퀀서 — 고수준 경로("직진2칸, 우회전2칸")를 교차로마다의 명령으로 자동 변환·전송.
★불안정 통신 대응: 각 명령을 "로봇 상태(S)가 바뀌어 받았다고 확인될 때까지" 여러 번 재전송한다.

브로커를 통해 동작: 로그(~/agv_serial.log)에서 교차로 도착(state=2, WAIT_NODE)을 감지하고,
명령파일(~/agv_cmd.txt)로 다음 명령을 보낸다. (허브 시리얼은 serial_broker.py가 점유)

사용:
  python3 grid_drive.py <robot_id> "<경로>" [속도]
경로 토큰(공백/쉼표 구분), 글자=방향 / 숫자=칸수:
  S<n> 직진 n칸(첫 구간)   R<n> 우회전 후 n칸   L<n> 좌회전 후 n칸   U<n> 유턴 후 n칸
예) python3 grid_drive.py 3 "S2 R2" 170   → 직진2칸 → 우회전 → 직진2칸 → 정지

전제: robot_grid v5(교차로에서 멈춰 대기) + 허브 브로커 실행 + 로봇이 격자 라인 위.
"""
import os, sys, time, re

FW = {"S": 2, "R": 3, "U": 4, "L": 5}        # STRAIGHT/RIGHT/UTURN/LEFT (펌웨어 enum과 동일)
RUN, STOP, WAIT_NODE = 1, 0, 2
NAME = {"S": "직진", "R": "우회전", "L": "좌회전", "U": "유턴"}

rid   = int(sys.argv[1])
path  = sys.argv[2]
speed = int(sys.argv[3]) if len(sys.argv) > 3 else 170
home  = os.path.expanduser("~")
logf  = os.path.join(home, "agv_serial.log")
cmdf  = os.path.join(home, "agv_cmd.txt")

# 경로 파싱 → 구간 [(방향, 칸수)]; 첫 구간 방향은 보통 'S'
segs = []
for tok in path.replace(",", " ").split():
    d = tok[0].upper()
    n = int(tok[1:]) if tok[1:].isdigit() else 1
    segs.append((d, n))
if not segs:
    print("경로가 비었습니다"); sys.exit(1)

# 교차로별 행동 리스트(마지막=정지)
total = sum(n for _, n in segs)
boundary = {}
cum = 0
for i in range(1, len(segs)):
    cum += segs[i - 1][1]
    boundary[cum] = segs[i][0]
actions = []
for k in range(1, total + 1):
    if k == total:      actions.append("STOP")
    elif k in boundary: actions.append(boundary[k])
    else:               actions.append("S")

def send(mode, sp):
    with open(cmdf, "a") as f:
        f.write(f"C {rid} {mode} {sp}\n")

def latest():
    """로그 끝에서 robot rid 최신 (state, node)"""
    try:
        lines = open(logf).readlines()[-500:]
    except FileNotFoundError:
        return None
    st = nd = None
    for ln in lines:
        m = re.search(rf"S {rid} (\d+) (\d+)", ln)
        if m: st, nd = int(m.group(1)), int(m.group(2))
    return (st, nd) if st is not None else None

def send_until(mode, sp, ok, label, tries=7, gap=0.3):
    """ok(state,node)->bool 가 참이 될 때까지(=로봇이 받음) 재전송. 실패하면 False."""
    for i in range(1, tries + 1):
        send(mode, sp)
        t = time.time()
        while time.time() - t < gap:
            time.sleep(0.08)
            s = latest()
            if s and ok(*s):
                if i > 1: print(f"      ↻ {i}회 재전송 후 전달됨")
                return True
    print(f"      ⚠ {label}: {tries}회 재전송해도 응답 없음 (전원/통신 확인)")
    return False

print(f"[robot {rid}] 경로 '{path}' → 교차로 행동 {actions} (총 {total}칸)")

# 출발: IDLE(0)을 벗어날 때까지 RUN 재전송
print(f"  ▶ RUN {speed} (받을 때까지 재전송)")
send_until(RUN, speed, lambda st, nd: st != 0, "RUN")

last_node = None
idx = 0
while idx < len(actions):
    # 다음 교차로 도착 대기
    t0 = time.time(); arrived = False
    while time.time() - t0 < 12:
        s = latest()
        if s and s[0] == WAIT_NODE and s[1] != last_node:
            last_node = s[1]; arrived = True; break
        time.sleep(0.12)
    if not arrived:
        print("  ⏱ 교차로 도착 대기 타임아웃 (전원/라인 확인)"); send(STOP, 0); break

    nodeK = last_node
    act = actions[idx]; idx += 1
    if act == "STOP":
        send_until(STOP, 0, lambda st, nd: st == 0, "STOP")
        print(f"  ■ 교차로{nodeK} 도착 → 정지 (목적지)")
        break
    # 행동: 이 교차로(WAIT_NODE)를 벗어날 때까지 재전송 = 받았다는 확인
    ok = send_until(FW[act], speed, lambda st, nd: st != WAIT_NODE or nd > nodeK, NAME[act])
    print(f"  → 교차로{nodeK} 도착 → {NAME[act]}" + ("" if ok else " (전달 실패)"))

print("완료.")
