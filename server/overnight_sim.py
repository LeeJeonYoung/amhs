#!/usr/bin/env python3
"""
밤샘 종합 시뮬 스윕 — 격자 AMHS 오프라인 시뮬을 여러 조합으로 돌려 KPI를 모은다.
하드웨어 불필요. 결과를 마크다운 리포트로 저장.

스윕: 격자 × 차량수 × 배차(greedy/hungarian) × 라우팅(static/congestion) × 시드
출력: ~/Downloads/nix/시뮬_종합결과.md  (자소서/포트폴리오용)
"""
import sys, os, time, statistics
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from amhs.sim import GridAMHS  # noqa

GRIDS  = [(4, 4), (6, 6), (8, 8)]
VEHS   = [3, 5, 10]
DISPS  = ["greedy", "hungarian"]
ROUTES = ["static", "congestion"]
SEEDS  = list(range(1, 11))      # 10 시드 평균
TICKS  = 4000
RATE   = 0.15

OUT = os.path.expanduser("~/Downloads/nix/시뮬_종합결과.md")

def avg(xs): return round(statistics.mean(xs), 3) if xs else 0

def run_cell(rows, cols, veh, disp, route):
    """시드 평균 KPI"""
    acc = {}
    for sd in SEEDS:
        sim = GridAMHS(rows=rows, cols=cols, n_veh=veh, seed=sd,
                       dispatcher=disp, routing=route, rate=RATE)
        k = sim.run(TICKS)
        for key, v in k.items():
            acc.setdefault(key, []).append(v)
    return {key: avg(v) for key, v in acc.items()}

def main():
    t0 = time.time()
    lines = []
    lines.append("# AMHS 격자 시뮬 — 종합 스윕 결과")
    lines.append(f"> 생성 {time.strftime('%Y-%m-%d %H:%M')} · {TICKS}틱 · 시드 {len(SEEDS)}개 평균 · 도착률 {RATE}")
    lines.append("> 배차 greedy↔hungarian, 라우팅 static↔congestion 비교. 하드웨어 불필요(알고리즘 검증).\n")

    findings = []
    for (rows, cols) in GRIDS:
        for veh in VEHS:
            lines.append(f"\n## {rows}×{cols} 격자 · 차량 {veh}대")
            lines.append("| 배차 | 라우팅 | 완료율 | 처리량/1k | 평균사이클 | 평균대기 | 가동률 | 데드락 | 재경로 |")
            lines.append("|---|---|---|---|---|---|---|---|---|")
            base = None
            best = None
            for disp in DISPS:
                for route in ROUTES:
                    k = run_cell(rows, cols, veh, disp, route)
                    lines.append(f"| {disp} | {route} | {k['completion_rate']} | {k['throughput_per_1k']} | "
                                 f"{k['avg_cycle']} | {k['avg_queue']} | {k['utilization']} | "
                                 f"{k['deadlocks']} | {k['reroutes']} |")
                    if disp == "greedy" and route == "static":
                        base = k
                    if best is None or k['avg_cycle'] < best[1]['avg_cycle']:
                        best = ((disp, route), k)
            # 개선폭(greedy+static 기준 → hungarian+best route)
            if base and best and base['avg_cycle']:
                k = best[1]
                dcyc = round((k['avg_cycle'] - base['avg_cycle']) / base['avg_cycle'] * 100, 1)
                dq = round((k['avg_queue'] - base['avg_queue']) / base['avg_queue'] * 100, 1) if base['avg_queue'] else 0
                findings.append(f"- {rows}×{cols}/{veh}대: 최적({best[0][0]}+{best[0][1]}) vs greedy+static → "
                                f"사이클 {dcyc:+}% · 대기 {dq:+}% · 데드락 {base['deadlocks']}→{k['deadlocks']}")
            # 중간 저장(진행 중에도 결과 보이게)
            with open(OUT, "w") as f:
                f.write("\n".join(lines) + "\n\n## 핵심 발견\n" + "\n".join(findings))

    lines.append("\n## 핵심 발견")
    lines.extend(findings)
    lines.append(f"\n> 총 {len(GRIDS)*len(VEHS)*len(DISPS)*len(ROUTES)*len(SEEDS)}회 실행 · 소요 {round(time.time()-t0)}초")
    with open(OUT, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"완료 → {OUT}  ({round(time.time()-t0)}초)")

if __name__ == "__main__":
    main()
