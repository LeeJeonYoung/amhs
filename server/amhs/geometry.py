"""
heading(방위) ↔ 상대회전 변환.

노드 id = r*cols + c, row 는 아래로 증가. 따라서:
  r-1 = 북(N), r+1 = 남(S), c+1 = 동(E), c-1 = 서(W)

방위 코드 : N=0, E=1, S=2, W=3 (시계방향)
회전 코드 : STRAIGHT=0, RIGHT=1, UTURN=2, LEFT=3
  → 펌웨어 Command.turn 으로 그대로 전송.
  → 회전 후 새 heading = 목표 방위.
"""
N, E, S, W = 0, 1, 2, 3
DIR_NAME = {N: "N", E: "E", S: "S", W: "W"}

STRAIGHT, RIGHT, UTURN, LEFT = 0, 1, 2, 3
TURN_NAME = {STRAIGHT: "STRAIGHT", RIGHT: "RIGHT", UTURN: "UTURN", LEFT: "LEFT"}


def abs_dir(g, a, b):
    """인접 노드 a→b 의 절대 방위."""
    ra, ca = g.rc(a)
    rb, cb = g.rc(b)
    dr, dc = rb - ra, cb - ca
    if (dr, dc) == (-1, 0):
        return N
    if (dr, dc) == (1, 0):
        return S
    if (dr, dc) == (0, 1):
        return E
    if (dr, dc) == (0, -1):
        return W
    raise ValueError(f"노드 {a}->{b} 는 4-인접이 아님")


def relative_turn(heading, target_dir):
    """현재 heading 에서 target_dir 로 가려면 어떤 회전인지. (target-heading) mod 4."""
    return (target_dir - heading) % 4


def turn_for(g, heading, cur, nxt):
    """현재 노드 cur(heading) 에서 다음 노드 nxt 로 갈 때 (회전코드, 새 heading)."""
    d = abs_dir(g, cur, nxt)
    return relative_turn(heading, d), d
