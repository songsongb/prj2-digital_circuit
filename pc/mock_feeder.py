# mock_feeder.py — FPGA 없이 전체 발렛파킹 데모를 재현하는 가짜 데이터 시뮬레이터
#
# RISC-V가 보낼 프레임(MAP/CELL/CAR/DIR/STATE/CAND/PATH/TARGET/AI/OBJ)을
# 실제 데모 흐름과 동일한 순서·타이밍으로 emit()한다.
# 경로(PATH)는 여기서 BFS로 직접 계산 → 항상 인접 유효 경로가 나오도록 보장.

import time
from collections import deque

import protocol as P

# 데모 속도 배수 (1.0=실시간 느낌, 키우면 빨라짐). 테스트 시 크게.
SPEED = 1.0

# 주차장 6x6 (y행 1~6, x열 1~6). 값: 1통로 2빈주차 3점유 4입구 5출구
# 통로/입구/출구가 연결되도록 설계.
_MAP = [
    [4, 2, 1, 3, 2, 1],   # y=1
    [1, 3, 1, 2, 3, 1],   # y=2
    [1, 1, 1, 1, 1, 1],   # y=3  (가로 간선)
    [1, 2, 3, 2, 1, 3],   # y=4
    [1, 3, 2, 3, 1, 2],   # y=5
    [1, 1, 1, 1, 1, 5],   # y=6  (가로 간선 → 출구)
]
N = 6
ENTRY = (1, 1)
EXIT = (6, 6)


def cell(x, y):
    return _MAP[y - 1][x - 1]


def _traversable(v):
    return v in (1, 4, 5)


def bfs(start, goal):
    """start→goal 최단경로(셀 리스트). goal이 주차칸이면 마지막 스텝만 진입 허용."""
    sx, sy = start
    gx, gy = goal
    seen = {(sx, sy)}
    prev = {}
    q = deque([(sx, sy)])
    while q:
        x, y = q.popleft()
        if (x, y) == (gx, gy):
            # 역추적
            path = [(x, y)]
            while (x, y) in prev:
                x, y = prev[(x, y)]
                path.append((x, y))
            return path[::-1]
        for dx, dy in ((0, -1), (1, 0), (0, 1), (-1, 0)):
            nx, ny = x + dx, y + dy
            if 1 <= nx <= N and 1 <= ny <= N and (nx, ny) not in seen:
                v = cell(nx, ny)
                if _traversable(v) or (nx, ny) == (gx, gy):
                    seen.add((nx, ny))
                    prev[(nx, ny)] = (x, y)
                    q.append((nx, ny))
    return []


def _heading(dx, dy):
    if dy < 0: return 0   # 북
    if dx > 0: return 1   # 동
    if dy > 0: return 2   # 남
    if dx < 0: return 3   # 서
    return 1


def _nap(stop, sec):
    """중단 가능한 sleep."""
    t = sec / SPEED
    end = time.time() + t
    while time.time() < end:
        if stop.is_set():
            return True
        time.sleep(0.01)
    return stop.is_set()


def valet_scenario(emit, stop):
    """전체 데모 시퀀스를 한 번 재생."""

    def send(line):
        emit(line)

    # ── 0) 초기화: 빈 맵 + 입구/출구만 표시, IDLE ─────────────
    blank = ["0"] * (N * N)
    blank[(ENTRY[1] - 1) * N + (ENTRY[0] - 1)] = "4"
    blank[(EXIT[1] - 1) * N + (EXIT[0] - 1)] = "5"
    send("MAP:" + "".join(blank))
    send(f"CAR:{ENTRY[0]},{ENTRY[1]},1")
    send("STATE:0")
    send("AI:대기 중 — KEY[1]로 스캔을 시작하세요")
    if _nap(stop, 1.0): return

    # ── 1) SCANNING: RC카가 통로를 따라 돌며 맵을 점진 공개 ───
    send("STATE:1")
    send("AI:환경 스캔 중 — HuskyLens 3모드 순환 폴링")
    # 스네이크(보ustrophedon) 순서로 셀 방문 → 한 칸씩 공개
    order = []
    for y in range(1, N + 1):
        xs = range(1, N + 1) if y % 2 == 1 else range(N, 0, -1)
        for x in xs:
            order.append((x, y))
    px, py = ENTRY
    for (x, y) in order:
        if stop.is_set(): return
        dx, dy = x - px, y - py
        h = _heading(dx, dy)
        v = cell(x, y)
        send(f"CELL:{x},{y},{v}")
        send(f"CAR:{x},{y},{h}")
        # 라인트래킹 방향/중심 흉내
        if v == 3:
            send("DIR:S"); send("LINE:160")
        elif h == 1:
            send("DIR:R"); send("LINE:205")
        elif h == 3:
            send("DIR:L"); send("LINE:115")
        else:
            send("DIR:G"); send("LINE:160")
        px, py = x, y
        if _nap(stop, 0.12): return
    send("DIR:S")
    send("AI:맵 스캔 완료 — 빈 구획 후보 추출")
    if _nap(stop, 0.6): return

    # ── 2) ANALYZING: 빈 구획 후보 + 스코어 ─────────────────
    send("STATE:2")
    empties = [(x, y) for y in range(1, N + 1) for x in range(1, N + 1) if cell(x, y) == 2]
    cands = []
    for i, (x, y) in enumerate(empties, start=1):
        # 스코어 = 출구까지 맨해튼 거리(작을수록 좋음) 흉내
        score = abs(EXIT[0] - x) + abs(EXIT[1] - y)
        cands.append((i, x, y, score))
    send("CAND:" + ";".join(f"{i},{x},{y},{s}" for (i, x, y, s) in cands))
    if _nap(stop, 0.8): return

    # ── 3) WAITING_AI: LLM 판단(여기선 흉내) ────────────────
    send("STATE:3")
    send("AI:Agent AI 판단 중 — 출구 인접·혼잡도 종합 평가")
    if _nap(stop, 1.2): return
    # 후보 동점 처리: 출구거리 최소 슬롯 선택
    best = min(cands, key=lambda c: c[3])
    tid, tx, ty, _ = best
    send(f"TARGET:{tx},{ty}")
    send(f"AI:구획 #{tid} 배정 — 출구 최단·진입 용이로 선택 (ASSIGN_SLOT)")

    # ── 4) EXECUTING: A* 경로 → 가상차량 자율주행 ───────────
    send("STATE:4")
    path = bfs(ENTRY, (tx, ty))
    if not path:
        send("PATHFAIL:")
        send("AI:경로 탐색 실패 — Agent AI에 데드락 전달")
    else:
        send("PATH:" + " ".join(f"{x},{y}" for (x, y) in path))
    if _nap(stop, 3.2): return     # 대시보드가 경로 따라 차량 이동시키는 시간
    send("STATE:5")                # DONE
    send("AI:주차 완료 — 목표 구획 도달")
    if _nap(stop, 1.6): return

    # ── 5) 긴급차량 시나리오: 재판단 + 리루팅 ───────────────
    send("OBJ:ambulance")
    send("STATE:3")
    send("AI:긴급차량 진입 감지 — 기존 배정 취소·전용 구획 재배치")
    if _nap(stop, 1.4): return
    # 다른 빈 슬롯으로 재배정 (입구에서 가장 가까운 곳)
    alt = min((c for c in cands if (c[1], c[2]) != (tx, ty)),
              key=lambda c: abs(ENTRY[0] - c[1]) + abs(ENTRY[1] - c[2]))
    aid, ax, ay, _ = alt
    send(f"TARGET:{ax},{ay}")
    send(f"AI:긴급차량 전용 구획 #{aid} 확보 — 경로 재계획 (REROUTE)")
    send("STATE:4")
    path2 = bfs(ENTRY, (ax, ay))
    send("PATH:" + " ".join(f"{x},{y}" for (x, y) in path2))
    if _nap(stop, 3.2): return
    send("STATE:5")
    send("AI:재배치 완료 — 긴급차량 우선 처리 종료")

    # 데모 종료 후 대기
    while not stop.is_set():
        time.sleep(0.1)
