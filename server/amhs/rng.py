"""결정론적 RNG + Poisson 작업 생성 (nix 의 mulberry32/poisson 포팅). seed 고정 = 재현 가능."""
import math


def _mulberry32(seed):
    """동일 seed → 동일 난수열. 알고리즘 A/B 비교를 재현 가능하게."""
    state = {"a": seed & 0xFFFFFFFF}

    def rnd():
        state["a"] = (state["a"] + 0x6D2B79F5) & 0xFFFFFFFF
        a = state["a"]
        t = (a ^ (a >> 15)) * (1 | a) & 0xFFFFFFFF
        t = ((t + ((t ^ (t >> 7)) * (61 | t) & 0xFFFFFFFF)) ^ t) & 0xFFFFFFFF
        return ((t ^ (t >> 14)) & 0xFFFFFFFF) / 4294967296.0

    return rnd


class RNG:
    """결정론 RNG 래퍼."""

    def __init__(self, seed):
        self._r = _mulberry32(seed)

    def random(self):
        return self._r()

    def randint(self, lo, hi):
        """[lo, hi) 정수."""
        return lo + int(self.random() * (hi - lo))

    def poisson(self, lam):
        """Poisson(lam) 표본(Knuth). 매 tick 작업 도착 수."""
        if lam <= 0:
            return 0
        L = math.exp(-lam)
        k = 0
        p = 1.0
        while True:
            k += 1
            p *= self.random()
            if p <= L:
                break
        return k - 1
