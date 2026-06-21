#!/usr/bin/env python3
"""
격자 AMHS 오프라인 시뮬 러너 (하드웨어 불필요).

예)
  python sim_grid.py                          # 4x4, 3대, hungarian+static, 2000틱
  python sim_grid.py --rows 4 --cols 4 --veh 3 --disp greedy --route congestion
  python sim_grid.py --watch                  # 격자에 로봇 위치 애니메이션
  python sim_grid.py --ab                     # greedy/hungarian × static/congestion A/B 표
"""
import argparse
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from amhs.sim import GridAMHS, STATE_NAME   # noqa: E402


def run_once(args, disp, route):
    sim = GridAMHS(rows=args.rows, cols=args.cols, n_veh=args.veh, seed=args.seed,
                   dispatcher=disp, routing=route, rate=args.rate)
    return sim.run(args.ticks)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rows", type=int, default=4)
    ap.add_argument("--cols", type=int, default=4)
    ap.add_argument("--veh", type=int, default=3)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--ticks", type=int, default=2000)
    ap.add_argument("--rate", type=float, default=0.15)
    ap.add_argument("--disp", choices=["hungarian", "greedy"], default="hungarian")
    ap.add_argument("--route", choices=["static", "congestion"], default="static")
    ap.add_argument("--watch", action="store_true", help="격자 애니메이션")
    ap.add_argument("--ab", action="store_true", help="배차×경로 4조합 A/B 표")
    args = ap.parse_args()

    if args.ab:
        print(f"== {args.rows}x{args.cols} 격자, {args.veh}대, {args.ticks}틱, seed={args.seed} ==")
        hdr = f"{'조합':28} {'thrpt/1k':>9} {'cycle':>7} {'queue':>7} {'완료율':>6} {'데드락':>6} {'재경로':>6}"
        print(hdr)
        for disp in ("greedy", "hungarian"):
            for route in ("static", "congestion"):
                k = run_once(args, disp, route)
                print(f"{disp+'+'+route:28} {k['throughput_per_1k']:>9} {k['avg_cycle']:>7} "
                      f"{k['avg_queue']:>7} {k['completion_rate']:>6} {k['deadlocks']:>6} {k['reroutes']:>6}")
        return

    sim = GridAMHS(rows=args.rows, cols=args.cols, n_veh=args.veh, seed=args.seed,
                   dispatcher=args.disp, routing=args.route, rate=args.rate)

    if args.watch:
        try:
            for _ in range(args.ticks):
                sim.step()
                print("\033[2J\033[H", end="")   # 화면 클리어
                print(f"tick {sim.now}  완료 {sim.completed}  대기 {len(sim.pending)}  "
                      f"데드락 {sim.deadlocks}")
                print(sim.render())
                print("\n로봇:", ", ".join(
                    f"#{v.id}@{v.pos}({STATE_NAME[v.state]})" for v in sim.veh))
                time.sleep(0.08)
        except KeyboardInterrupt:
            pass
        print()
    else:
        k = sim.run(args.ticks)

    k = sim.kpis()
    print(f"\n== 결과 ({args.disp}+{args.route}, {args.rows}x{args.cols}, "
          f"{args.veh}대, {args.ticks}틱, seed={args.seed}) ==")
    for key, val in k.items():
        print(f"  {key:18}: {val}")


if __name__ == "__main__":
    main()
