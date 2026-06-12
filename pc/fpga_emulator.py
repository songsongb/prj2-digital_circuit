# fpga_emulator.py — FPGA(RISC-V) 역할을 흉내내는 프레임 송신기
#
# 목적: 실제 보드가 아직 데이터를 안 보내도, PC의 실데이터 처리 경로
#   (수신→시각화→LLM 판단→CMD 송신→PATH 수신→애니메이션)를 그대로 검증한다.
#   나중에 config.DATA_SOURCE="serial"로 바꾸면 이 자리에 실제 FPGA가 들어온다.
#
# === 이 파일이 곧 RISC-V가 구현해야 할 "송신 프레임 명세" ===
#   FPGA→PC : STATE / CAR / CELL / SCANDONE / PATH / DONE
#   PC→FPGA : CMD:action,slot,x,y   (LLM 최적주차 결과)

import math
import time

import config as C
import world as W

SPEED = 1.0   # 데모 속도 배수(테스트 시 크게)


def _open(truth, a, b):
    """벽 통과 가능? (World.is_open과 동일 규칙 + 수동벽)"""
    if frozenset((a, b)) in W.EXTRA_WALLS:
        return False
    ta, tb = truth.get(a), truth.get(b)
    if ta is None or tb is None or ta == W.WALL or tb == W.WALL:
        return False
    return ta in (W.PATH, W.ENTRY, W.EXIT) or tb in (W.PATH, W.ENTRY, W.EXIT)


def _bfs(truth, start, goal):
    from collections import deque
    seen = {start}; prev = {}; q = deque([start])
    while q:
        cur = q.popleft()
        if cur == goal:
            path = [cur]
            while cur in prev:
                cur = prev[cur]; path.append(cur)
            return path[::-1]
        cx, cy = cur
        for dx, dy in ((0, -1), (1, 0), (0, 1), (-1, 0)):
            nb = (cx + dx, cy + dy)
            if nb in seen or nb not in truth or not _open(truth, cur, nb):
                continue
            t = truth[nb]
            if t in (W.EMPTY, W.OCCUPIED) and nb != goal:
                continue
            seen.add(nb); prev[nb] = cur; q.append(nb)
    return []


def _heading(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    if dy < 0: return 0
    if dx > 0: return 1
    if dy > 0: return 2
    return 3


def _nap(stop, sec):
    end = time.time() + sec / SPEED
    while time.time() < end:
        if stop.is_set():
            return True
        time.sleep(0.01)
    return False


def emulator_scenario(emit, get_cmd, stop):
    """FPGA 한 사이클: 스캔 송신 → CMD 대기 → A* 경로 송신 → 주차.
    컴팩트 프레임: S<n> / P<x><y> / M<x><y><v> / D / G<cells>."""
    truth = W._build_truth(int(time.time()) % 100000)   # 점유 포함 ground truth

    emit("S1")                       # SCANNING
    reported = set()
    for cell in W.SCAN_ROUTE:
        if stop.is_set():
            return
        emit(f"P{cell[0]}{cell[1]}")  # 차량 위치
        for (px, py), t in truth.items():
            if t in (W.EMPTY, W.OCCUPIED) and (px, py) not in reported:
                if math.hypot(px - cell[0], py - cell[1]) <= C.SCAN_RADIUS:
                    emit(f"M{px}{py}{t}")   # 맵 셀 점유
                    reported.add((px, py))
        if _nap(stop, 0.22):
            return
    emit("D")                        # SCANDONE
    emit("S2")                       # ANALYZING

    # PC(LLM)의 목표 프레임 대기:  T<x><y>
    cmd = None
    while cmd is None and not stop.is_set():
        cmd = get_cmd()
        if cmd is None:
            time.sleep(0.02)
    if stop.is_set():
        return
    try:
        s = cmd.strip()
        target = (int(s[1]), int(s[2]))
    except Exception:
        return
    emit("S4")                       # EXECUTING

    # FPGA가 A* 계산(여기선 BFS) → 경로(G) 송신 후 주행(P)
    cur = W.SCAN_ROUTE[-1]
    path = _bfs(truth, cur, target)
    if path:
        emit("G" + "".join(f"{x}{y}" for (x, y) in path))
        for cell in path:
            if stop.is_set():
                return
            emit(f"P{cell[0]}{cell[1]}")
            if _nap(stop, 0.32):
                return
    emit("S5")                       # DONE
    while not stop.is_set():
        time.sleep(0.1)
