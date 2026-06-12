# world.py — PC 자율주행 시뮬레이터 (발렛파킹 맵 · 벽 인지 · 스캔루프 · LLM 최적주차)
#
# 흐름:
#   ① IN(1,1) 진입 → 회색 통로 링을 한 바퀴 주행(SCANNING)
#      주행 중 HuskyLens가 좌우를 스캔 → 주차칸(빈/점유)과 벽을 점진적으로 인지
#   ② 한 바퀴 완료(맵 인지 완료) → Agent AI(LLM)가 최적 주차 위치 판단(DECIDING)
#   ③ 선택한 구획으로 주행→주차(PARKING→DONE). 주행 중 동적 장애물에 반응.
#
# 좌표: 셀 (x,y) 1-based, x=열(→), y=행(↓). ego는 연속좌표(fx,fy).

import heapq
import math
import random
import threading

import config as C
import agent_ai

# 셀 타입
PATH, EMPTY, OCCUPIED, ENTRY, EXIT, WALL = 1, 2, 3, 4, 5, 6

# 그림과 동일한 6x6 발렛파킹 맵
#   E 입구 / X 출구 / . 통로 / o 주차칸 / B 막힌셀
_LAYOUT = [
    "E . o o o o",   # y=1  (3~6열 상단 주차)
    "o . . . . .",   # y=2  (통로: 행2)
    "o . o o o .",   # y=3  (1열 주차, 3~5열 중앙블록, 6열 통로)
    "o . o o o .",   # y=4
    "o . . . . .",   # y=5  (통로: 행5)
    "B o o o o X",   # y=6  (1열 막힘, 2~5열 하단 주차, 출구)
]
_CH = {"E": ENTRY, "X": EXIT, ".": PATH, "o": EMPTY, "B": WALL}

# 통로 링 스캔 경로(IN→시계방향 한 바퀴)
SCAN_ROUTE = [(1, 1), (2, 1), (2, 2), (3, 2), (4, 2), (5, 2), (6, 2),
              (6, 3), (6, 4), (6, 5), (5, 5), (4, 5), (3, 5), (2, 5),
              (2, 4), (2, 3), (2, 2)]

# 수동 추가 벽(셀 경계). 그림의 핑크선 → 통로↔주차칸 칸막이.
EXTRA_WALLS = {
    frozenset({(2, 1), (3, 1)}),   # 상단 점유칸 왼쪽
    frozenset({(1, 1), (1, 2)}),   # IN 아래
    frozenset({(2, 3), (3, 3)}),   # 중앙블록 좌
    frozenset({(2, 4), (3, 4)}),
    frozenset({(5, 3), (6, 3)}),   # 중앙블록 우
    frozenset({(5, 4), (6, 4)}),
}


def _build_layout():
    """정적 구조만(통로/주차칸 위치/벽/IN/OUT). 점유는 미포함 — 런타임에 발견."""
    g = {}
    for y, row in enumerate(_LAYOUT, start=1):
        for x, ch in enumerate(row.split(), start=1):
            g[(x, y)] = _CH[ch]
    return g


def _build_truth(seed):
    g = _build_layout()
    parking = [k for k, v in g.items() if v == EMPTY]
    # 주차칸 일부를 점유로(HuskyLens가 발견할 차량)
    rng = random.Random(seed)
    rng.shuffle(parking)
    for cell in parking[:int(len(parking) * C.OCCUPIED_RATIO)]:
        g[cell] = OCCUPIED
    return g


# FSM 상태값 → 행동 표시 매핑(실데이터 모드)
_STATE_BEHAVIOR = {0: "IDLE", 1: "SCANNING", 2: "DECIDING", 3: "DECIDING",
                   4: "CRUISING", 5: "DONE"}


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _drivable(t):  return t in (PATH, ENTRY, EXIT)
def _parking(t):   return t in (EMPTY, OCCUPIED)
def _blocked(t):   return t == WALL or t is None and False


# ──────────────────────────────────────────────────────────────
class DynObstacle:
    """동적 장애물: 웨이포인트를 따라 움직이며 미래 궤적을 예측당한다."""
    def __init__(self, kind, waypoints, speed, loop=True):
        self.kind = kind
        self.wps = [(float(x), float(y)) for x, y in waypoints]
        self.fx, self.fy = self.wps[0]
        self.speed = speed
        self.loop = loop
        self.i = 1
        self.alive = True
        self.dir = 1

    def update(self, dt):
        if not self.alive:
            return
        tx, ty = self.wps[self.i] if 0 <= self.i < len(self.wps) else self.wps[-1]
        dx, dy = tx - self.fx, ty - self.fy
        d = math.hypot(dx, dy)
        step = self.speed * dt
        if d <= step:
            self.fx, self.fy = tx, ty
            self.i += self.dir
            if self.i >= len(self.wps):
                if self.loop: self.dir = -1; self.i = len(self.wps) - 2
                else: self.alive = False
            elif self.i < 0:
                self.dir = 1; self.i = 1
        else:
            self.fx += dx / d * step; self.fy += dy / d * step

    def cell(self):
        return (int(round(self.fx)), int(round(self.fy)))

    def predict(self, horizon, steps):
        pts, fx, fy, i, dr = [], self.fx, self.fy, self.i, self.dir
        dt = horizon / steps
        for _ in range(steps):
            tx, ty = (self.wps[i] if 0 <= i < len(self.wps) else self.wps[-1])
            dx, dy = tx - fx, ty - fy
            d = math.hypot(dx, dy); s = self.speed * dt
            if d <= s:
                fx, fy = tx, ty; i += dr
                if i >= len(self.wps): i = len(self.wps) - 2; dr = -1
                elif i < 0: i = 1; dr = 1
            else:
                fx += dx / d * s; fy += dy / d * s
            pts.append((fx, fy))
        return pts


# ──────────────────────────────────────────────────────────────
class World:
    def __init__(self, n=6, seed=None, live=False):
        self.n = n
        self.live = live
        self.seed = seed if seed is not None else random.randint(0, 10 ** 6)
        # live: 정적 구조만 안다(점유는 프레임으로 수신). sim: 점유 포함 ground truth.
        self.truth = _build_layout() if live else _build_truth(self.seed)
        self.entry = next(k for k, v in self.truth.items() if v == ENTRY)
        self.exit = next(k for k, v in self.truth.items() if v == EXIT)

        # 공통 상태
        self.belief = {}                 # (x,y) -> [type, last_seen_t]
        self.newly_seen = set()
        self.known_walls = set()
        self.fx, self.fy = float(self.entry[0]), float(self.entry[1])
        self.heading = 0.0
        self.speed = 0.0
        self.path = []
        self.wp = 1
        self.parked = False
        self.cost_field = {}
        self.candidate_paths = []
        self.goal = None
        self.target_slot = None
        self.replan_flash = 0.0
        self.replan_count = 0
        self.obstacles = []
        self.predictions = []
        self.nearest_obs = 9.9
        self.safety_hit = False
        self.ai_provider = ""
        self._ai_pending = False
        self._ai_result = None
        self.t = 0.0
        self._replan_timer = 0.0
        self._done_t = 0.0

        if live:
            # 실데이터 모드: 프레임 수신으로 구동
            self.state_num = 0
            self.behavior = "IDLE"
            self.scan_done_flag = False
            self._car_target = (self.fx, self.fy)
            self._seen_time = {}
            self.ai_reason = "FPGA 연결 — 데이터 수신 대기"
            return

        # ── 시뮬레이션 모드 전용 ──
        self.path = list(SCAN_ROUTE)
        self.wp = 1
        self.behavior = "SCANNING"
        self._phase = "scan"
        self.ai_reason = "IN 진입 — 통로를 따라 주차장을 스캔합니다"
        self.perceive()
        if len(self.path) > 1:
            tx, ty = self.path[1]
            self.heading = math.atan2(ty - self.fy, tx - self.fx)

    # ── 좌표/타입 헬퍼 ─────────────────────────────────────
    def in_grid(self, x, y):
        return 1 <= x <= self.n and 1 <= y <= self.n

    def ego_cell(self):
        return (int(clamp(round(self.fx), 1, self.n)),
                int(clamp(round(self.fy), 1, self.n)))

    def believed_type(self, x, y):
        b = self.belief.get((x, y))
        return b[0] if b else None

    def truth_type(self, x, y):
        return self.truth.get((x, y))

    # ── 벽(셀 경계) ────────────────────────────────────────
    def is_open(self, a, b):
        """두 인접 셀 사이 통행 가능? (통로↔통로, 통로↔주차칸은 열림)"""
        if frozenset((a, b)) in EXTRA_WALLS:   # 수동 추가 벽
            return False
        ta, tb = self.truth_type(*a), self.truth_type(*b)
        if ta is None or tb is None:
            return False
        if _blocked(ta) or _blocked(tb):
            return False
        return _drivable(ta) or _drivable(tb)

    def is_opening(self, x, y, side):
        """경계측이 IN/OUT 개구부인지."""
        if (x, y) == self.entry and side == "L":
            return True
        if (x, y) == self.exit and side == "R":
            return True
        return False

    # ── 인지(Perception): 주행 중 좌우 스캔 + 벽 인지 ───────
    def perceive(self):
        self.newly_seen.clear()
        for (x, y), v in self.truth.items():
            if math.hypot(x - self.fx, y - self.fy) <= C.SCAN_RADIUS:
                was = (x, y) not in self.belief
                self.belief[(x, y)] = [v, self.t]
                if was:
                    self.newly_seen.add((x, y))
        self._update_known_walls()

    def _update_known_walls(self):
        # 인지된 셀의 벽 경계(내부 + 외곽)를 known_walls에 등록
        for (x, y) in list(self.belief.keys()):
            for side, (nx, ny) in (("R", (x + 1, y)), ("L", (x - 1, y)),
                                    ("D", (x, y + 1)), ("U", (x, y - 1))):
                if self.in_grid(nx, ny):
                    if not self.is_open((x, y), (nx, ny)):
                        self.known_walls.add(self._wall_key(x, y, side))
                else:
                    if not self.is_opening(x, y, side):
                        self.known_walls.add(self._wall_key(x, y, side))

    @staticmethod
    def _wall_key(x, y, side):
        # 같은 경계를 양쪽 셀에서 중복 등록하지 않도록 정규화
        if side == "R":  return ("V", x, y)
        if side == "L":  return ("V", x - 1, y)
        if side == "D":  return ("H", x, y)
        if side == "U":  return ("H", x, y - 1)

    def perceived_pct(self):
        return len(self.belief) / (self.n * self.n)

    def scan_complete(self):
        # 모든 주차칸을 인지했으면 스캔 완료로 간주
        for (x, y), t in self.truth.items():
            if _parking(t) and (x, y) not in self.belief:
                return False
        return True

    # ── 계획(Planning): 벽을 존중하는 코스트 A* ────────────
    def astar(self, start, goal, blocked=frozenset()):
        if start == goal:
            return [start]
        openh = [(0.0, start)]; g = {start: 0.0}; prev = {}
        while openh:
            _, cur = heapq.heappop(openh)
            if cur == goal:
                path = [cur]
                while cur in prev:
                    cur = prev[cur]; path.append(cur)
                return path[::-1]
            cx, cy = cur
            for dx, dy in ((0, -1), (1, 0), (0, 1), (-1, 0)):
                nb = (cx + dx, cy + dy)
                if not self.in_grid(*nb) or (nb in blocked and nb != goal):
                    continue
                if not self.is_open(cur, nb):       # 벽 통과 금지
                    continue
                t = self.believed_type(*nb)
                # 주차칸은 목표일 때만 진입
                if _parking(t) and nb != goal:
                    continue
                if t == OCCUPIED and nb != goal:
                    continue
                step = 1.0
                if t is None:
                    step += C.COST_W_UNKNOWN
                step += C.COST_W_OBST * self._obst_penalty(*nb) * 0.25
                ng = g[cur] + step
                if ng < g.get(nb, 1e9):
                    g[nb] = ng; prev[nb] = cur
                    h = (abs(goal[0] - nb[0]) + abs(goal[1] - nb[1])) * C.COST_W_GOAL
                    heapq.heappush(openh, (ng + h, nb))
        return []

    def _obst_penalty(self, x, y):
        p = 0.0
        for o in self.obstacles:
            if o.alive and math.hypot(o.fx - x, o.fy - y) < 1.2:
                p += 1.5
        return p

    def compute_cost_field(self, goal):
        from collections import deque
        dist = {goal: 0}; q = deque([goal])
        while q:
            cur = q.popleft(); cx, cy = cur
            for dx, dy in ((0, -1), (1, 0), (0, 1), (-1, 0)):
                nb = (cx + dx, cy + dy)
                if nb in dist or not self.in_grid(*nb):
                    continue
                if not self.is_open(cur, nb):
                    continue
                t = self.believed_type(*nb)
                if _parking(t) and nb != goal:
                    continue
                dist[nb] = dist[cur] + 1; q.append(nb)
        raw = {k: C.COST_W_GOAL * d + C.COST_W_OBST * self._obst_penalty(*k)
               for k, d in dist.items()}
        if raw:
            lo, hi = min(raw.values()), max(raw.values()); rng = (hi - lo) or 1.0
            self.cost_field = {k: (v - lo) / rng for k, v in raw.items()}

    def known_empty_slots(self):
        return [k for k, b in self.belief.items() if b[0] == EMPTY]

    # ── 판단(Agent AI): 비동기 LLM 최적주차 ─────────────────
    def _start_decision(self):
        self._ai_pending = True
        self.behavior = "DECIDING"
        self.ai_reason = "최적 주차 위치 분석 중 — Agent AI 판단"
        ctx = {
            "n": self.n,
            "entry": list(self.entry),
            "exit": list(self.exit),
            "pos": list(self.ego_cell()),
            "empty": [list(s) for s in self.known_empty_slots()],
            "occupied": [list(k) for k, b in self.belief.items() if b[0] == OCCUPIED],
            "dist": self._slot_distances(),
        }
        threading.Thread(target=self._run_decision, args=(ctx,), daemon=True).start()

    def _slot_distances(self):
        """후보별 (현재→슬롯 경로길이, 슬롯→출구 거리) — LLM/휴리스틱 입력."""
        start = self.ego_cell(); out = {}
        for s in self.known_empty_slots():
            p = self.astar(start, s)
            if p:
                out[f"{s[0]},{s[1]}"] = {
                    "path_len": len(p) - 1,
                    "exit_dist": abs(self.exit[0] - s[0]) + abs(self.exit[1] - s[1]),
                }
        return out

    def _run_decision(self, ctx):
        res = agent_ai.decide_parking(ctx)
        self._ai_result = res

    def _apply_decision(self):
        res = self._ai_result; self._ai_result = None; self._ai_pending = False
        slot = tuple(res.get("slot", [])) if res else ()
        if not slot or slot not in self.known_empty_slots():
            empties = self.known_empty_slots()
            slot = empties[0] if empties else None
        self.ai_provider = res.get("provider", "") if res else ""
        if slot:
            self.goal = self.target_slot = slot
            self.ai_reason = (res.get("reason") if res else "") or f"구획 {slot} 선택"
            self.behavior = "CRUISING"      # DECIDING 래치 해제
            self._build_candidates()
            self._phase = "park"
            self.replan()
            self.spawn_pedestrian_on_path()
        else:
            self.ai_reason = "빈 주차칸을 찾지 못함"
            self._phase = "end"; self.behavior = "DONE"

    def _build_candidates(self):
        start = self.ego_cell(); scored = []
        for s in self.known_empty_slots():
            p = self.astar(start, s)
            if p:
                sc = (len(p) - 1) + (abs(self.exit[0] - s[0]) + abs(self.exit[1] - s[1])) * 0.4
                scored.append((p, sc, s))
        scored.sort(key=lambda e: e[1])
        # 채택 경로를 맨 앞으로
        scored.sort(key=lambda e: 0 if e[2] == self.target_slot else 1)
        self.candidate_paths = scored[:5]

    def replan(self):
        self._replan_timer = 0.0
        start = self.ego_cell()
        blocked = {o.cell() for o in self.obstacles if o.alive}
        if not self.goal:
            return
        self.compute_cost_field(self.goal)
        new_path = self.astar(start, self.goal, frozenset(blocked))
        if new_path and new_path != self.path:
            self.replan_flash = 0.25; self.replan_count += 1
        if new_path:
            self.path = new_path
            self.wp = 1 if len(new_path) > 1 else 0

    # ── 동적 장애물 ─────────────────────────────────────────
    def spawn_pedestrian_on_path(self):
        if len(self.path) < 3:
            return
        idx = min(self.wp + 3, len(self.path) - 2)
        cx, cy = self.path[idx]; nx, ny = self.path[idx + 1]
        dx, dy = nx - cx, ny - cy
        if abs(dx) >= abs(dy):
            wp = [(cx, cy - 1.5), (cx, cy + 1.5)]
        else:
            wp = [(cx - 1.5, cy), (cx + 1.5, cy)]
        self.obstacles.append(DynObstacle("ped", wp, speed=1.1, loop=True))

    def update_predictions(self):
        self.predictions = [(o.kind, o.predict(C.PRED_HORIZON, C.PRED_STEPS))
                            for o in self.obstacles if o.alive]

    # ── 반응(Reaction) ──────────────────────────────────────
    def assess_hazard(self):
        self.nearest_obs = 9.9; self.safety_hit = False
        for o in self.obstacles:
            if not o.alive: continue
            d = math.hypot(o.fx - self.fx, o.fy - self.fy)
            self.nearest_obs = min(self.nearest_obs, d)
            if d < C.SAFETY_R: self.safety_hit = True
        conflict = False
        ego_future = self._ego_future(C.YIELD_HORIZON, C.PRED_STEPS)
        for o in self.obstacles:
            if not o.alive: continue
            opred = o.predict(C.YIELD_HORIZON, C.PRED_STEPS)
            for (ax, ay), (bx, by) in zip(ego_future, opred):
                if math.hypot(ax - bx, ay - by) < 0.75:
                    conflict = True; break
        return conflict

    def _ego_future(self, horizon, steps):
        pts = []; fx, fy = self.fx, self.fy
        wps = [(float(x), float(y)) for x, y in self.path]
        idx = min(self.wp, len(wps) - 1) if wps else 0
        v = max(self.speed, C.V_CRUISE * 0.6); dt = horizon / steps
        for _ in range(steps):
            if not wps or idx >= len(wps):
                pts.append((fx, fy)); continue
            tx, ty = wps[idx]; dx, dy = tx - fx, ty - fy
            d = math.hypot(dx, dy); s = v * dt
            if d <= s: fx, fy = tx, ty; idx = min(idx + 1, len(wps) - 1)
            else: fx += dx / d * s; fy += dy / d * s
            pts.append((fx, fy))
        return pts

    # ── 제어(Control) ──────────────────────────────────────
    def steer_and_move(self, dt, target_speed):
        wps = [(float(x), float(y)) for x, y in self.path]
        if not wps:
            self.speed = max(0.0, self.speed - C.ACCEL * dt); return
        self.wp = min(self.wp, len(wps) - 1)
        tx, ty = wps[self.wp]
        dx, dy = tx - self.fx, ty - self.fy; dist = math.hypot(dx, dy)
        if dist < 0.32 and self.wp < len(wps) - 1:
            self.wp += 1; tx, ty = wps[self.wp]
            dx, dy = tx - self.fx, ty - self.fy; dist = math.hypot(dx, dy)
        desired = math.atan2(dy, dx)
        dang = (desired - self.heading + math.pi) % (2 * math.pi) - math.pi
        self.heading += clamp(dang, -C.TURN_RATE * dt, C.TURN_RATE * dt)
        turn_factor = 1.0 - clamp(abs(dang) / math.pi, 0, 0.7)
        tv = target_speed * turn_factor
        if tv > self.speed: self.speed = min(tv, self.speed + C.ACCEL * dt)
        else: self.speed = max(tv, self.speed - C.ACCEL * dt)
        self.fx += math.cos(self.heading) * self.speed * dt
        self.fy += math.sin(self.heading) * self.speed * dt

    # ════════════════════════════════════════════════════════
    #  실데이터(live) 모드 — FPGA 프레임으로 구동
    # ════════════════════════════════════════════════════════
    def apply_frame(self, t, payload):
        """FPGA→PC 프레임을 상태에 반영(protocol.py 형식)."""
        if t == "STATE":
            try:
                self.state_num = int(payload)
            except ValueError:
                return
            self.behavior = _STATE_BEHAVIOR.get(self.state_num, "IDLE")
        elif t == "CAR":
            p = payload.split(",")
            if len(p) >= 2:
                try:
                    self._car_target = (int(p[0]), int(p[1]))
                except ValueError:
                    pass
        elif t == "CELL":            # 주차칸 점유 발견: x,y,v (2빈 3점유)
            p = payload.split(",")
            if len(p) == 3:
                try:
                    x, y, v = int(p[0]), int(p[1]), int(p[2])
                    self.belief[(x, y)] = [v, self.t]
                    self._seen_time[(x, y)] = self.t
                except ValueError:
                    pass
        elif t == "PATH":
            pts = []
            for tok in payload.split():
                f = tok.split(",")
                if len(f) == 2:
                    pts.append((int(f[0]), int(f[1])))
            self.path = pts
            self.wp = 1 if len(pts) > 1 else 0
        elif t == "TARGET":
            f = payload.split(",")
            if len(f) == 2:
                self.target_slot = self.goal = (int(f[0]), int(f[1]))
        elif t == "SCANDONE":
            self.scan_done_flag = True
        elif t == "AI":
            self.ai_reason = payload
        elif t == "DONE":
            self.behavior = "DONE"; self.parked = True

    def apply_live(self, line):
        """FPGA 컴팩트 프레임 파싱.  S<n> / P<x><y> / M<x><y><v> / D / G<...>"""
        line = line.strip()
        if not line:
            return
        c = line[0]
        try:
            if c == "S" and len(line) >= 2:
                self.state_num = ord(line[1]) - 48
                self.behavior = _STATE_BEHAVIOR.get(self.state_num, "IDLE")
            elif c == "P" and len(line) >= 3:
                self._car_target = (int(line[1]), int(line[2]))
            elif c == "M" and len(line) >= 4:
                x, y, v = int(line[1]), int(line[2]), int(line[3])
                self.belief[(x, y)] = [v, self.t]
                self._seen_time[(x, y)] = self.t
            elif c == "D":
                self.scan_done_flag = True
            elif c == "G":                       # 경로: 셀 좌표 2자리씩 나열
                digits = line[1:].replace(" ", "")
                pts = [(int(digits[i]), int(digits[i + 1]))
                       for i in range(0, len(digits) - 1, 2)]
                if pts:
                    self.path = pts
                    self.wp = 1 if len(pts) > 1 else 0
        except (ValueError, IndexError):
            pass

    def _update_live(self, dt):
        dt = min(dt, 0.05); self.t += dt
        # 차량을 보고된 목표 셀로 부드럽게 보간 이동
        tx, ty = self._car_target
        dx, dy = tx - self.fx, ty - self.fy
        d = math.hypot(dx, dy)
        if d > 1e-3:
            self.heading = math.atan2(dy, dx)
            step = min(d, C.V_CRUISE * dt)
            self.fx += dx / d * step; self.fy += dy / d * step
            self.speed = step / dt
        else:
            self.speed = 0.0
        self.perceive_structure()
        return None

    def perceive_structure(self):
        """차량 주변의 정적 구조(통로/벽/IN/OUT)를 인지. 점유는 CELL 프레임으로만."""
        for (x, y), v in self.truth.items():
            if v == EMPTY:               # 주차칸 점유는 프레임으로만 갱신
                continue
            if math.hypot(x - self.fx, y - self.fy) <= C.SCAN_RADIUS:
                if (x, y) not in self.belief:
                    self.belief[(x, y)] = [v, self.t]
                    self._seen_time[(x, y)] = self.t
        self._update_known_walls()
        self.newly_seen = {c for c, tt in self._seen_time.items() if self.t - tt < 0.5}

    # ── 메인 틱(시뮬레이션) ─────────────────────────────────
    def update(self, dt):
        if self.live:
            return self._update_live(dt)
        dt = min(dt, 0.05); self.t += dt
        self.replan_flash = max(0.0, self.replan_flash - dt)

        for o in self.obstacles: o.update(dt)
        self.obstacles = [o for o in self.obstacles if o.alive]
        self.update_predictions()
        self.perceive()

        self._replan_timer += dt
        if self._replan_timer >= C.REPLAN_PERIOD and self.goal:
            self.replan()

        conflict = self.assess_hazard()
        self._scenario(dt)

        # 행동 결정(우선순위)
        if self.behavior in ("DONE", "DECIDING"):
            target_v = 0.0
        elif self.safety_hit:
            self.behavior = "EMERGENCY"; target_v = 0.0
            self.ai_reason = "긴급 정지 — 안전 반경 내 장애물 감지"
        elif conflict:
            self.behavior = "YIELDING"; target_v = 0.0
            self.ai_reason = "양보 — 동적 장애물과 충돌 예측, 진행 보류"
        elif self._phase == "scan":
            self.behavior = "SCANNING"; target_v = C.V_CRUISE
        elif self.goal and self._near_goal():
            self.behavior = "PARKING"; target_v = C.V_PARK
        elif self.replan_flash > 0:
            self.behavior = "REPLANNING"; target_v = C.V_TURN
        else:
            self.behavior = "CRUISING"; target_v = C.V_CRUISE

        if self.behavior != "DECIDING":
            self.steer_and_move(dt, target_v)

        if self.goal and self._phase == "park" and self._reached_goal():
            self._arrive()

    # ── 시나리오 ────────────────────────────────────────────
    def _scenario(self, dt):
        if self._phase == "scan":
            # 스캔 루프 종료 조건: 경로 끝 도달 또는 전 주차칸 인지
            at_end = self.wp >= len(self.path) - 1 and \
                     math.hypot(self.path[-1][0] - self.fx, self.path[-1][1] - self.fy) < 0.4
            if (at_end or self.scan_complete()) and not self._ai_pending:
                self._phase = "decide"
                self._start_decision()
        elif self._phase == "decide":
            if self._ai_result is not None:
                self._apply_decision()
        elif self._phase == "park":
            pass

    def _near_goal(self):
        return self.goal is not None and self._dist(self.goal) < 1.3

    def _reached_goal(self):
        return self.goal is not None and self._dist(self.goal) < 0.28

    def _dist(self, cell):
        return math.hypot(cell[0] - self.fx, cell[1] - self.fy)

    def _arrive(self):
        self.fx, self.fy = float(self.goal[0]), float(self.goal[1])
        self.speed = 0.0; self.parked = True
        self.behavior = "DONE"; self._phase = "end"
        self.ai_reason = f"주차 완료 — 구획 {self.target_slot} (Agent AI 선택)"

    # ── 텔레메트리 ──────────────────────────────────────────
    def telemetry(self):
        return {
            "behavior": self.behavior, "speed": self.speed,
            "nearest": self.nearest_obs, "perceived": self.perceived_pct(),
            "replans": self.replan_count, "target": self.target_slot,
            "provider": self.ai_provider,
        }
