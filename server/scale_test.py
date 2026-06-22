#!/usr/bin/env python3
"""
스케일 벤치마크 — 오프라인 시뮬(amhs.sim)을 1~100대까지 돌려 거동을 본다.

발견(16x16 기준): 처리량은 50~75대에서 정점, 100대(밀도 39%)에서 혼잡 붕괴.
            붕괴는 '대수 한계'가 아니라 '밀도' 문제 — 밀도 ≲25%면 100대도 완료율 0.96+.
            전 구간에서 점유 불변식(한 칸 한 대)은 유지 → 크래시 아니라 graceful degradation.

실행:
  python scale_test.py              # 둘 다 (대수 스윕 + 100대 밀도 스윕)
  python scale_test.py --sweep      # 격자 고정, 대수 1→100
  python scale_test.py --density    # 100대 고정, 격자 키워 밀도↓
"""
import argparse
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from amhs.sim import GridAMHS                                        # noqa: E402


def run(side, veh, ticks, rate, seed=1):
    """한 번 돌리고 (kpis, 초, 점유불변식OK) 반환. step() 내부가 매 틱 불변식 assert."""
    t0 = time.time()
    try:
        sim = GridAMHS(rows=side, cols=side, n_veh=veh, seed=seed, rate=rate,
                       dispatcher="hungarian", routing="static")
        sim.run(ticks)
        k = sim.kpis()
        occ = sim.traffic.occ
        inv = (len(set(occ.values())) == len(occ)) and (len(occ) == veh)
        return k, time.time() - t0, inv, None
    except AssertionError:
        return None, time.time() - t0, False, "점유불변식 위반"
    except Exception as e:                                          # noqa: BLE001
        return None, time.time() - t0, False, repr(e)


def sweep_vehicles(side, ticks):
    nodes = side * side
    print(f"\n[대수 스윕] 격자 {side}x{side}={nodes}칸 · {ticks}틱 · hungarian+static\n")
    print(f"{'대수':>4} {'밀도%':>5} {'완료건':>6} {'thrpt/1k':>8} {'cycle':>7} "
          f"{'완료율':>6} {'가동률':>6} {'데드락':>6} {'재경로':>7} {'충돌0':>5} {'초':>6}")
    for veh in [1, 3, 10, 25, 50, 75, 100]:
        if veh > nodes:
            continue
        rate = max(0.15, veh * 0.04)                                # 대수에 맞춰 작업 투입↑
        k, sec, inv, err = run(side, veh, ticks, rate)
        if err:
            print(f"{veh:>4}  오류: {err}")
            continue
        print(f"{veh:>4} {100*veh/nodes:>5.0f} {k['completed']:>6} {k['throughput_per_1k']:>8} "
              f"{k['avg_cycle']:>7} {k['completion_rate']:>6} {k['utilization']:>6} "
              f"{k['deadlocks']:>6} {k['reroutes']:>7} {'OK' if inv else 'FAIL':>5} {sec:>6.2f}")


def sweep_density(veh, ticks, rate):
    print(f"\n[밀도 스윕] 로봇 {veh}대 고정 · {ticks}틱 · 작업 {rate}/틱 · 격자만 키움\n")
    print(f"{'격자':>7} {'칸수':>5} {'밀도%':>5} {'완료건':>6} {'thrpt/1k':>8} {'cycle':>7} "
          f"{'완료율':>6} {'데드락':>6} {'충돌0':>5} {'초':>6}")
    for side in [16, 20, 24, 32]:
        nodes = side * side
        if veh > nodes:
            continue
        k, sec, inv, err = run(side, veh, ticks, rate)
        if err:
            print(f"{side}x{side}  오류: {err}")
            continue
        print(f"{side}x{side:<4} {nodes:>5} {100*veh/nodes:>5.0f} {k['completed']:>6} "
              f"{k['throughput_per_1k']:>8} {k['avg_cycle']:>7} {k['completion_rate']:>6} "
              f"{k['deadlocks']:>6} {'OK' if inv else 'FAIL':>5} {sec:>6.2f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", action="store_true", help="격자 고정, 대수 1→100")
    ap.add_argument("--density", action="store_true", help="100대 고정, 밀도↓")
    ap.add_argument("--side", type=int, default=16, help="대수 스윕용 격자 한 변")
    ap.add_argument("--ticks", type=int, default=1200)
    ap.add_argument("--veh", type=int, default=100, help="밀도 스윕용 로봇 수")
    ap.add_argument("--rate", type=float, default=2.5, help="밀도 스윕용 작업/틱")
    a = ap.parse_args()
    both = not (a.sweep or a.density)
    if a.sweep or both:
        sweep_vehicles(a.side, a.ticks)
    if a.density or both:
        sweep_density(a.veh, a.ticks, a.rate)


if __name__ == "__main__":
    main()
