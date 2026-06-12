# protocol.py — RISC-V(FPGA) ↔ PC 통신 프레임 정의 + 월드 상태
#
# 설계 결정: 베어메탈 RISC-V가 1바이트씩(PC_TX_DATA) 내보내기 쉬운
#   "줄 단위 ASCII 프레임" 을 쓴다.  형식:  TYPE:payload\n
# 풀 JSON 직렬화는 MCU에 부담이므로 RISC-V는 가벼운 프레임만 보내고,
# LLM 입력용 JSON 조립은 PC(agent_ai.py)가 담당한다.
#
# ── RISC-V → PC 프레임 ──────────────────────────────────────
#   MAP:<36자리>           맵 전체 스냅샷. 셀값(0~5) 36개를 행우선으로 나열
#                          예) MAP:411111... (idx = (y-1)*6 + (x-1))
#   CELL:x,y,v             셀 1칸만 갱신 (스캔 중 점진적 공개에 사용)
#   CAR:x,y,h              차량 현재 셀 + heading(0북1동2남3서)
#   DIR:G|L|R|S|B          라인트래킹 방향 명령
#   LINE:cx                라인 중심 X (0~320)
#   STATE:n                FSM 상태 0~5
#   CAND:id,x,y,score;...  후보 구획 목록(빈 칸 후보)
#   PATH:x,y x,y ...       A* 경로(셀 시퀀스). 비어있으면 경로없음
#   PATHFAIL:              경로 탐색 실패(Path Not Found) 신호
#   TARGET:x,y             목표 구획 좌표(하이라이트용)
#   AI:<텍스트>            판단 이유 한 줄(보통은 PC가 채우지만 포워딩도 허용)
#   OBJ:tag                특수 객체 감지(예: 긴급차량) — 재판단 트리거
#
# ── PC → RISC-V 프레임 ──────────────────────────────────────
#   CMD:action,slot,x,y\n  LLM 판단 결과 (ASSIGN/WAIT/REROUTE)

# FSM 상태
ST_IDLE, ST_SCANNING, ST_ANALYZING, ST_WAITING_AI, ST_EXECUTING, ST_DONE = range(6)
STATE_NAME = {
    ST_IDLE: "IDLE", ST_SCANNING: "SCANNING", ST_ANALYZING: "ANALYZING",
    ST_WAITING_AI: "WAITING_AI", ST_EXECUTING: "EXECUTING", ST_DONE: "DONE",
}

DIR_NAME = {"G": "GO", "L": "LEFT", "R": "RIGHT", "S": "STOP", "B": "BACK"}


class WorldState:
    """대시보드가 그리는 모든 상태를 담는 단일 소스 오브 트루스."""
    def __init__(self, n=6):
        self.n = n
        # map[y][x] (0-based 내부 저장). 외부 좌표는 1-based로 주고받는다.
        self.map = [[0] * n for _ in range(n)]
        self.car = (1, 1, 1)          # (x, y, heading)
        self.direction = "S"          # 라인트래킹 방향
        self.line_x = 160             # 라인 중심
        self.state = ST_IDLE
        self.candidates = []          # [(id, x, y, score), ...]
        self.path = []                # [(x, y), ...] A* 결과
        self.path_failed = False
        self.target = None            # (x, y) 목표 구획
        self.ai_reason = ""           # Agent AI 판단 이유
        self.last_obj = ""            # 마지막 특수객체 태그
        self.scan_done = False        # LEDR[7] 상응

    # 내부 저장은 0-based, 외부 좌표는 1-based
    def set_cell(self, x, y, v):
        if 1 <= x <= self.n and 1 <= y <= self.n:
            self.map[y - 1][x - 1] = v

    def get_cell(self, x, y):
        return self.map[y - 1][x - 1]


def _to_int(s, default=0):
    try:
        return int(s)
    except (ValueError, TypeError):
        return default


def parse_line(line):
    """원시 텍스트 한 줄 → (TYPE, payload_str). 형식 안 맞으면 (None, raw)."""
    line = line.strip()
    if not line or ":" not in line:
        return (None, line)
    t, _, payload = line.partition(":")
    return (t.strip().upper(), payload.strip())


def apply_frame(state: WorldState, t, payload):
    """파싱된 프레임을 WorldState에 반영. 반환: 이벤트 문자열(없으면 None)."""
    if t == "MAP":
        digits = [c for c in payload if c.isdigit()]
        n = state.n
        if len(digits) >= n * n:
            for y in range(n):
                for x in range(n):
                    state.map[y][x] = int(digits[y * n + x])
    elif t == "CELL":
        p = payload.split(",")
        if len(p) == 3:
            state.set_cell(_to_int(p[0]), _to_int(p[1]), _to_int(p[2]))
    elif t == "CAR":
        p = payload.split(",")
        if len(p) >= 2:
            h = _to_int(p[2], state.car[2]) if len(p) >= 3 else state.car[2]
            state.car = (_to_int(p[0]), _to_int(p[1]), h)
    elif t == "DIR":
        if payload[:1].upper() in ("G", "L", "R", "S", "B"):
            state.direction = payload[:1].upper()
    elif t == "LINE":
        state.line_x = _to_int(payload, state.line_x)
    elif t == "STATE":
        state.state = _to_int(payload, state.state)
        if state.state == ST_DONE:
            state.scan_done = True
    elif t == "CAND":
        cands = []
        for chunk in payload.split(";"):
            f = chunk.split(",")
            if len(f) == 4:
                cands.append((_to_int(f[0]), _to_int(f[1]), _to_int(f[2]), _to_int(f[3])))
        state.candidates = cands
    elif t == "PATH":
        pts = []
        for tok in payload.split():
            f = tok.split(",")
            if len(f) == 2:
                pts.append((_to_int(f[0]), _to_int(f[1])))
        state.path = pts
        state.path_failed = (len(pts) == 0)
        return "PATH"
    elif t == "PATHFAIL":
        state.path = []
        state.path_failed = True
        return "PATHFAIL"
    elif t == "TARGET":
        f = payload.split(",")
        if len(f) == 2:
            state.target = (_to_int(f[0]), _to_int(f[1]))
    elif t == "AI":
        state.ai_reason = payload
    elif t == "OBJ":
        state.last_obj = payload
        return "OBJ"
    return None


def encode_cmd(action, slot, x, y):
    """PC → RISC-V 명령 프레임 생성."""
    return f"CMD:{action},{slot},{x},{y}\n"
