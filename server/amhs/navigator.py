"""
교차로 회전결정 알고리즘 — '경로(노드 열) → 노드별 좌/우/직진/유턴 명령'의 단일 정본.

핵심 아이디어
-------------
로봇은 자기가 격자의 몇 번 노드에 있는지 모른다. 라인을 따라가다 교차로(가로선)에
닿으면 "도착했고 명령을 기다린다(ST_WAIT_NODE)"고 보고할 뿐이다.
그래서 '어느 방향으로 틀지'는 위(관제 서버)가 결정해 내려보낸다.

방향 결정은 두 정보만 있으면 순수 함수로 풀린다.
  1) 지금 바라보는 절대 방위(heading): N/E/S/W
  2) 다음에 가야 할 노드의 절대 방위(target_dir)
회전 = (target_dir - heading) mod 4  →  STRAIGHT/RIGHT/UTURN/LEFT  (geometry.relative_turn)

이 모듈은 그 한 스텝 계산(geometry.turn_for)을 '경로 전체'로 확장한다.
경로를 따라가며 heading 을 갱신해, 각 교차로에서 내려야 할 명령을 미리 다 만든다.
하드웨어/시뮬 공용 — 같은 plan() 결과를 실물 로봇에도, 오프라인 시뮬에도 그대로 쓴다.

펌웨어 명령 매핑
----------------
관제 서버는 plan() 이 만든 회전코드(STRAIGHT/RIGHT/UTURN/LEFT)를 그대로 쓰지 않고
펌웨어가 아는 mode 번호로 바꿔 보낸다. 그 변환표가 FW_MODE 다(아래 한 곳에서만 정의).
"""
from .geometry import turn_for, abs_dir, TURN_NAME, DIR_NAME, STRAIGHT, RIGHT, UTURN, LEFT

# ── 펌웨어(robot_grid.ino) 명령 프로토콜: "C <id> <mode> <speed>" ──
# 0,1 은 주행 제어, 2~5 는 교차로에서의 회전. geometry 회전코드와 1:1 로 매핑된다.
MODE_STOP     = 0   # 정지(모터 release)
MODE_RUN      = 1   # 라인추종 재개 — 다음 교차로까지 직진 추종
MODE_STRAIGHT = 2   # 교차로 직진 통과
MODE_RIGHT    = 3   # 교차로 우회전
MODE_UTURN    = 4   # 교차로 유턴(180°)
MODE_LEFT     = 5   # 교차로 좌회전

# geometry 회전코드 → 펌웨어 mode (단일 정본; 서버/시뮬/문서가 모두 여기를 참조)
FW_MODE = {STRAIGHT: MODE_STRAIGHT, RIGHT: MODE_RIGHT, UTURN: MODE_UTURN, LEFT: MODE_LEFT}
MODE_NAME = {MODE_STOP: "STOP", MODE_RUN: "RUN", MODE_STRAIGHT: "STRAIGHT",
             MODE_RIGHT: "RIGHT", MODE_UTURN: "UTURN", MODE_LEFT: "LEFT"}


class Step:
    """경로 한 칸: cur 노드(heading 으로 진입) 에서 nxt 노드로 갈 때 내릴 명령."""
    __slots__ = ("frm", "to", "turn", "turn_name", "new_heading", "fw_mode")

    def __init__(self, frm, to, turn, new_heading):
        self.frm = frm
        self.to = to
        self.turn = turn                       # geometry 회전코드
        self.turn_name = TURN_NAME[turn]
        self.new_heading = new_heading         # 회전 후 바라보는 방위
        self.fw_mode = FW_MODE[turn]           # 실제로 로봇에 보낼 mode

    def __repr__(self):
        return (f"Step({self.frm}->{self.to} {self.turn_name}"
                f"->{DIR_NAME[self.new_heading]} fw={self.fw_mode})")

    def as_dict(self):
        return {"from": self.frm, "to": self.to, "turn": self.turn_name,
                "new_heading": DIR_NAME[self.new_heading], "fw_mode": self.fw_mode}


def plan(g, start, heading, route):
    """
    경로를 노드별 회전 명령으로 펼친다.

    매개변수
      g       : RailGraph
      start   : 현재 노드
      heading : 현재 방위(N/E/S/W)
      route   : 다음 노드들의 리스트(현재 노드 제외). router.static_route 반환과 동일.

    반환 : [Step, ...]  (route 길이만큼. route 가 비면 [])
    인접하지 않은 두 노드가 연달아 오면 ValueError(abs_dir).
    """
    steps = []
    cur, h = start, heading
    for nxt in route:
        turn, d = turn_for(g, h, cur, nxt)     # 순수 함수: (회전코드, 새 heading)
        steps.append(Step(cur, nxt, turn, d))
        cur, h = nxt, d                        # 다음 칸은 회전 후 방위로 진입
    return steps


def final_heading(steps, heading):
    """경로 주행이 끝났을 때 로봇이 바라보는 방위(steps 가 비면 그대로)."""
    return steps[-1].new_heading if steps else heading


def fw_commands(steps):
    """plan() 결과를 로봇에 순서대로 보낼 mode 번호 리스트로. [2,5,2,...]"""
    return [s.fw_mode for s in steps]


def explain(g, start, heading, route):
    """plan() 을 사람이 읽는 줄글로. 데모/로그/면접 설명용."""
    steps = plan(g, start, heading, route)
    lines = []
    h = heading
    for i, s in enumerate(steps, 1):
        rs, cs = g.rc(s.frm)
        rt, ct = g.rc(s.to)
        lines.append(
            f"{i:2}. 노드 {s.frm}{(rs, cs)} 에서 {DIR_NAME[h]} 보는 중 → "
            f"{s.turn_name:8} → 노드 {s.to}{(rt, ct)} ({DIR_NAME[s.new_heading]} 방향), "
            f"로봇에 보낼 명령 mode={s.fw_mode}({MODE_NAME[s.fw_mode]})")
        h = s.new_heading
    return "\n".join(lines) if lines else "(이동 없음 — 이미 목적지)"
