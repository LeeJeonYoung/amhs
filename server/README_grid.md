# 격자 AMHS (Phase 0 — 오프라인 두뇌)

실물 4×4 격자 함대의 **두뇌**를 하드웨어 없이 먼저 만들고 검증한 것.
`~/Downloads/nix` 시뮬레이터(JS)의 알고리즘을 파이썬·격자로 포팅했다.

## 구성
```
amhs/
  graph.py     격자 그래프 + Dijkstra(blocked 회피) + all-pairs 거리캐시
  geometry.py  노드/heading → 상대회전(직진/우/유턴/좌)  ← 실물 Command.turn 그대로
  router.py    static / congestion 경로
  dispatch.py  hungarian(최적 배차) / greedy
  traffic.py   노드 점유 예약(상호배제) + 회피용 blocked 집합
  rng.py       결정론 RNG(mulberry32) + Poisson 작업 생성
  sim.py       GridAMHS — 노드스텝 이동 + 예약 + 데드락회복 + 작업 + 배차 + KPI
sim_grid.py    CLI 러너
tests/         단위/통합 테스트 (12개)
```

## 실행
```bash
cd server
python3 tests/test_amhs.py            # 테스트 (12/12 PASS)
python3 sim_grid.py --ticks 2000      # 기본 4x4·3대·hungarian+static
python3 sim_grid.py --ab --rate 0.3   # 배차×경로 4조합 A/B
python3 sim_grid.py --watch --rate 0.2  # 격자에 로봇 위치 애니메이션
```

## 이게 왜 중요한가 (하드웨어 전 검증)
- 부품 없이 **경로·배차·예약·데드락 회피**를 100% 결정론으로 검증 → 실물에선 튜닝만.
- `sim.py` 가 매 tick 만드는 **회전 명령 로그(turn_log)** 가 실물 펌웨어로 보낼 `Command.turn` 과 동일.
  관제가 (노드, heading, 경로)를 추적해 직진/좌/우/유턴을 계산하는 로직이 그대로 하드웨어로 간다.
- 점유 불변식(노드당 1대)을 매 tick assert → 충돌 0 보장.

## ★ 관찰: 추상(BPR) vs 물리(점유) 모델 차이 재현
moderate~고부하에서 **congestion 라우팅이 static 보다 오히려 나빴다**(데드락↑·완료율↓).
원인: 단일-노드 점유 예약 모델에선 혼잡 회피 경로가 서로 충돌해 **경로 thrashing** 을 유발.
→ 이는 시뮬의 `실험결과_물리모델비교.md` 가 말한 *"추상 결론이 물리에선 데드락으로 드러난다"* 와 일치.
→ **실물 함대 기본값은 static + 예약**. (congestion 은 실험 옵션으로 유지)

3대/16노드(밀도 19%)에서 static+예약: 완료율 ~0.94–0.97, 데드락 소수 → 안정.

## 다음 (부품 도착 후)
Phase 1 회전 캘리브레이션(`firmware/diagnostics/06_turn_test`) → Phase 2 1대 격자주행
(`firmware/robot_grid` + `fleet_grid_server.py`) → Phase 3 3대 예약/데드락 → Phase 4 풀 AMHS.
설계 전체는 플랜 파일 참조.
