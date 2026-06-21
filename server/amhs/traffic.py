"""
노드 점유 예약 — 한 노드에 한 로봇(상호배제). 8자 구역락의 전(全)노드 일반화.
nix physical.mjs 의 occ[node]=vehId 모델 포팅. 오프라인 시뮬과 실물 관제가 공유.
"""


class Traffic:
    def __init__(self):
        self.occ = {}     # node -> robot_id

    def owner(self, node):
        return self.occ.get(node)

    def free(self, node, rid):
        """node 가 비었거나 내가 점유 중이면 True."""
        o = self.occ.get(node)
        return o is None or o == rid

    def reserve(self, node, rid):
        """예약 성공 시 True. 이미 남이 점유면 False."""
        if self.free(node, rid):
            self.occ[node] = rid
            return True
        return False

    def release(self, node, rid):
        if self.occ.get(node) == rid:
            del self.occ[node]

    def blocked_set(self, rid, dest):
        """회피경로 계산용 — 남이 점유한 노드(목적지 제외)."""
        return {n for n, o in self.occ.items() if o != rid and n != dest}

    def place(self, node, rid):
        """초기 배치 등 강제 점유."""
        self.occ[node] = rid
