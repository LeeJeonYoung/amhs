"""경로 — 정적 최단 / 혼잡 인지. 반환은 '다음 노드들'(현재 노드 제외)."""
from .graph import dijkstra


def static_route(g, src, dst, blocked=None):
    p = dijkstra(g, src, dst, blocked=blocked)
    return p[1:] if len(p) > 1 else []


def congestion_route(g, src, dst, edge_load, alpha=0.5, blocked=None):
    """간선 가중치 = 길이 × (1 + α·간선부하). 막히는 간선을 피해 우회."""
    def w(u, v):
        load = edge_load.get((u, v), 0) + edge_load.get((v, u), 0)
        return g.length(u, v) * (1 + alpha * load)
    p = dijkstra(g, src, dst, weight=w, blocked=blocked)
    return p[1:] if len(p) > 1 else []
