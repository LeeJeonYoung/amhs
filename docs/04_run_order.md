# 04. 부품 도착 후 실행 순서 (체크리스트)

> 위에서 아래로 순서대로. 각 단계에서 **확인**이 끝나야 다음으로.

## STEP 0 — 개발환경 (부품 기다리는 동안 미리)
- [ ] 아두이노 IDE 설치
- [ ] 라이브러리 매니저에서 **"RF24" by TMRh20** 설치
- [ ] Mac 파이썬: `cd server && pip install -r requirements.txt`
- [x] (LLM용) Ollama + `qwen3:1.7b` 설치 완료 (`llm_orchestrator.py` 가 이 모델 사용)

## STEP 1 — 1대 라인추종만 검증 (무선 빼고)
- [ ] 키트 1대 조립 (`docs/01_assembly.md`)
- [ ] **부위별 진단 + 캘리브레이션** (`docs/05_calibration.md`) — 모터/IR/초음파 값 확정
- [ ] ⚠️ 단독 검증은 `firmware/diagnostics/05_linefollow_solo` 업로드로 한다
      (`robot.ino` 는 무선 RUN 명령을 받아야만 움직이므로 허브 없이는 안 움직임)
- [ ] 8자 트랙 위에 올려 라인 따라가는지 확인
- [ ] **안 맞으면**: 센서 polarity → `LINE_ON` 을 LOW 로, 모터 방향 반대면 IN1/IN2 스왑
- [ ] 확정값을 `robot.ino` 에 반영 후 `robot.ino`(ROBOT_ID=1) 업로드 → STEP 2 로

## STEP 2 — 무선 1:1 검증 (허브 ↔ 로봇1)
- [ ] 로봇1에 nRF24 배선 (`docs/02_wiring.md`)
- [ ] 허브 조립 + `firmware/hub/hub.ino` 업로드
- [ ] 허브를 Mac에 USB 연결, 포트 확인: `ls /dev/tty.*`
- [ ] `python server/fleet_server.py --port /dev/tty.usbserial-XXXX`
- [ ] 서버에서 `status` → 로봇1 상태 보이면 무선 OK
- [ ] `go 1` → 로봇1 출발 / `halt 1` → 정지 확인

## STEP 3 — 3대로 확장
- [ ] 로봇2(ROBOT_ID=2), 로봇3(ROBOT_ID=3) 업로드 + 배선
- [ ] 서버 `status` 에 3대 다 보이는지
- [ ] `run` → 3대 동시 출발, 8자 교차로에서 **한 대씩만 통과**하는지 확인
      (로그에 `[교통] 교차로 점유 → ...` / `... 통과 완료 → 해제`)

## STEP 4 — LLM 두뇌 연결
- [ ] fleet_server 실행한 상태에서, 새 터미널: `python brain/llm_orchestrator.py`
- [ ] "전부 출발" / "2번 멈춰" / "상태 알려줘" 자연어로 지휘되는지 확인

## 흔한 문제
| 증상 | 원인/해결 |
|---|---|
| 무선 `no-ack` | 어댑터 VCC를 5V에 꽂았나 / 주소 RBT0x 일치 / 모듈 불량(예비 교체) |
| 라인 못 따라감 | `LINE_ON` 극성, 센서 높이, 바닥 대비, 속도 너무 빠름(BASE_SPEED↓) |
| 교차로 못 멈춤 | 가로선이 충분히 굵은가, 두 센서가 동시에 닿는가 |
| 한 대만 안 움직임 | 그 로봇 ROBOT_ID 중복 업로드? / 배선 확인 |
| 포트 못 찾음 | `ls /dev/tty.*` 에서 usbserial/usbmodem 확인 |
