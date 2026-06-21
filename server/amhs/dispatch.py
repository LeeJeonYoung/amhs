"""배차 알고리즘 — Hungarian(최적) / greedy(근시안). nix index.html 의 hungarian 포팅."""
INF = float("inf")
BIG = 1e7   # 미배정(유휴 유지) 더미 비용


def hungarian(cost):
    """
    n×m 비용행렬의 최소비용 할당. 반환 a[i]=j (로봇 i → 작업 j, 없으면 -1).
    Kuhn–Munkres O(k³), k=max(n,m) 패딩(더미 BIG).
    """
    n = len(cost)
    if n == 0:
        return []
    m = len(cost[0])
    k = max(n, m)
    C = [[(cost[i][j] if i < n and j < m else BIG) for j in range(k)] for i in range(k)]

    u = [0.0] * (k + 1)
    v = [0.0] * (k + 1)
    p = [0] * (k + 1)      # p[j] = j 열에 배정된 행(1-indexed)
    way = [0] * (k + 1)

    for i in range(1, k + 1):
        p[0] = i
        j0 = 0
        minv = [INF] * (k + 1)
        used = [False] * (k + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = INF
            j1 = -1
            for j in range(1, k + 1):
                if not used[j]:
                    cur = C[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j
            for j in range(k + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1

    a = [-1] * n
    for j in range(1, k + 1):
        i = p[j]
        if 1 <= i <= n and (j - 1) < m:
            a[i - 1] = j - 1
    return a


def greedy(idle, cand, dist):
    """작업별로 가장 가까운 유휴 로봇 배정. dist(robot, task)->비용. 반환 [(robot, task), ...]."""
    pairs = []
    pool = list(idle)
    for t in cand:
        if not pool:
            break
        best = min(pool, key=lambda v: (dist(v, t), v.id))
        pairs.append((best, t))
        pool.remove(best)
    return pairs
