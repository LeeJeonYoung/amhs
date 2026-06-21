"""
amhs — 격자형 AMHS(자동반송시스템) 두뇌.

~/Downloads/nix 시뮬레이터(JS)의 핵심 알고리즘을 파이썬으로 포팅한 것.
오프라인 시뮬(sim.py)과 실물 관제(fleet_grid_server.py)가 공유한다.

  graph     격자 그래프 + Dijkstra + all-pairs 거리캐시
  geometry  노드/heading → 상대회전(직진/우/유턴/좌) 변환
  router    정적 / 혼잡 인지 경로
  dispatch  Hungarian(최적 배차) / greedy
  traffic   노드 점유 예약 + 회피경로용 blocked 집합
  rng       결정론적 RNG(mulberry32) + Poisson 작업 생성
"""
