#!/usr/bin/env python3
"""
시리얼 브로커 — 한 프로세스가 포트를 잡고, 받은 줄을 로그파일에 계속 기록.
다른 터미널/사람/스크립트는 로그파일을 tail -f 로 같이 보고, 명령은 cmd 파일로 넣는다.
(시리얼 포트는 1프로세스 전용이라, 이렇게 중계해야 둘 다 본다.)

사용: python3 serial_broker.py <port> [logfile] [cmdfile] [baud]
  기본 log =  ~/agv_serial.log
  기본 cmd =  ~/agv_cmd.txt   (여기에 "C 3 1 180" 한 줄 쓰면 그 줄을 시리얼로 전송)

보기:  tail -f ~/agv_serial.log
보내기: echo "C 3 1 180" >> ~/agv_cmd.txt
"""
import os
import sys
import time
import serial

port = sys.argv[1]
home = os.path.expanduser("~")
logf = sys.argv[2] if len(sys.argv) > 2 else os.path.join(home, "agv_serial.log")
cmdf = sys.argv[3] if len(sys.argv) > 3 else os.path.join(home, "agv_cmd.txt")
baud = int(sys.argv[4]) if len(sys.argv) > 4 else 115200

open(cmdf, "w").close()                       # 명령 파일 비우고 시작
s = serial.Serial(port, baud, timeout=0.1)
time.sleep(2)                                  # 보드 리셋 대기

with open(logf, "a", buffering=1) as lg:
    lg.write(f"\n=== broker 시작 {time.strftime('%H:%M:%S')}  port={port} baud={baud} ===\n")
    buf = ""
    while True:
        try:
            chunk = s.read(s.in_waiting or 1).decode(errors="ignore")
        except Exception:
            chunk = ""
        if chunk:                                  # 완전한 줄(\n)만 기록 — 쪼개짐 방지
            buf += chunk
            while "\n" in buf:
                ln, buf = buf.split("\n", 1)
                ln = ln.rstrip("\r")
                if ln:
                    lg.write(time.strftime("%H:%M:%S  ") + ln + "\n")
        # 명령 파일에 내용 있으면 시리얼로 전송 후 비움
        try:
            if os.path.getsize(cmdf) > 0:
                with open(cmdf) as cf:
                    cmds = cf.read()
                open(cmdf, "w").close()
                for c in cmds.splitlines():
                    c = c.strip()
                    if c:
                        s.write((c + "\n").encode()); s.flush()
                        lg.write(time.strftime("%H:%M:%S  ") + ">>> 전송: " + c + "\n")
        except Exception:
            pass
        time.sleep(0.02)
