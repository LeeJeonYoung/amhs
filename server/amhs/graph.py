"""격자 그래프 + 최단경로 (nix index.html 의 makeGrid/dijkstra/allPairs 포팅)."""
import heapq


class RailGraph:
    """rows×cols 4-연결 격자. 노드 id = r*cols + c."""

    def __init__(self, rows, cols, edge_len=1.0):
        self.rows, self.cols, self.el = rows, cols, edge_len
        n = rows * cols
        self.adj = {i: [] for i in range(n)}
        self.edge_len = {}

        def conn(a, b):
            if b not in self.adj[a]:
                self.adj[a].append(b)
                self.edge_len[(a, b)] = edge_len
            if a not in self.adj[b]:
                self.adj[b].append(a)
                self.edge_len[(b, a)] = edge_len

        for r in range(rows):
            for c in range(cols):
                if c + 1 < cols:
                    conn(self.nid(r, c), self.nid(r, c + 1))   # 가로
                if r + 1 < rows:
                    conn(self.nid(r, c), self.nid(r + 1, c))   # 세로

        self.stations = list(range(n))

    def nid(self, r, c):
        return r * self.cols + c

    def rc(self, n):
        return divmod(n, self.cols)        # (row, col)

    def length(self, u, v):
        return self.edge_len.get((u, v), float("inf"))


def dijkstra(g, src, dst, weight=None, blocked=None):
    """src→dst 최단경로(노드 리스트). 도달 불가면 []. blocked 노드는 회피(단, dst 는 허용)."""
    if src == dst:
        return [src]
    weight = weight or (lambda u, v: g.length(u, v))
    blocked = blocked or set()
    dist = {src: 0.0}
    prev = {}
    visited = set()
    pq = [(0.0, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if u in visited:
            continue
        visited.add(u)
        if u == dst:
            break
        for v in g.adj[u]:
            if v in blocked and v != dst:
                continue
            nd = d + weight(u, v)
            if v not in dist or nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    if dst not in dist:
        return []
    path = [dst]
    while path[-1] != src:
        path.append(prev[path[-1]])
    path.reverse()
    return path


def all_pairs(g):
    """D[s][t] = s→t 최단거리. 배차 비용 계산용 1회 캐시."""
    D = {}
    for s in g.stations:
        dist = {s: 0.0}
        seen = set()
        pq = [(0.0, s)]
        while pq:
            d, u = heapq.heappop(pq)
            if u in seen:
                continue
            seen.add(u)
            for v in g.adj[u]:
                nd = d + g.length(u, v)
                if v not in dist or nd < dist[v]:
                    dist[v] = nd
                    heapq.heappush(pq, (nd, v))
        D[s] = dist
    return D
