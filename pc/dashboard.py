# dashboard.py — 자율주행 월드 시각화 (인지/계획/예측/제어 레이어를 그린다)
#
# 레이어(아래→위):
#   1) 격자 + 인지 fog-of-war + 신뢰도 감쇠
#   2) 코스트 히트맵(계획 비용)            [H 토글]
#   3) 후보 경로(스코어링) + 채택 경로     [C 토글]
#   4) 센서 FOV 부채꼴                      [V 토글]
#   5) 동적 장애물 + 예측 궤적
#   6) ego 차량 + 안전 버블 + heading
#   7) 우측 텔레메트리/Agent AI 패널

import math
import pygame

import config as C
import world as W


def _lerp(a, b, t):
    return (int(a[0] + (b[0] - a[0]) * t),
            int(a[1] + (b[1] - a[1]) * t),
            int(a[2] + (b[2] - a[2]) * t))


class Fonts:
    def __init__(self):
        prefer = ["malgungothic", "malgun gothic", "applegothic", "applesdgothicneo",
                  "notosanscjkkr", "notosanskr", "nanumgothic", "notosanscjk",
                  "dejavusans", "arial"]
        avail = set(pygame.font.get_fonts())
        name = next((p.replace(" ", "") for p in prefer if p.replace(" ", "") in avail), None)
        self.name = name
        self.big   = pygame.font.SysFont(name, 28, bold=True)
        self.h1    = pygame.font.SysFont(name, 22, bold=True)
        self.lbl   = pygame.font.SysFont(name, 14, bold=True)
        self.body  = pygame.font.SysFont(name, 16)
        self.small = pygame.font.SysFont(name, 12)
        self.huge  = pygame.font.SysFont(name, 34, bold=True)


_TYPE_COL = {
    W.PATH: C.COL_PATHCELL, W.EMPTY: C.COL_EMPTY, W.OCCUPIED: C.COL_OCCUPIED,
    W.ENTRY: C.COL_ENTRY, W.EXIT: C.COL_EXIT, W.WALL: C.COL_BLOCKED,
}


class Dashboard:
    def __init__(self, surface, fonts):
        self.surf = surface
        self.f = fonts
        self.show_heat = False
        self.show_cand = True
        self.show_fog = True
        self.show_fov = True
        self.paused = False
        self._trail = []

    # ── 좌표 변환 ───────────────────────────────────────────
    def gpix(self, gx, gy):
        return (C.GRID_X + (gx - 0.5) * C.CELL, C.GRID_Y + (gy - 0.5) * C.CELL)

    def cell_rect(self, x, y):
        return pygame.Rect(C.GRID_X + (x - 1) * C.CELL, C.GRID_Y + (y - 1) * C.CELL,
                           C.CELL, C.CELL)

    # ── 메인 그리기 ─────────────────────────────────────────
    def draw(self, world: W.World):
        s = self.surf
        s.fill(C.COL_BG)
        self._title()
        self._grid(world)
        if self.show_heat:
            self._heatmap(world)
        self._walls(world)
        if self.show_cand:
            self._candidates(world)
        self._path(world)
        if self.show_fov:
            self._fov(world)
        self._obstacles(world)
        self._ego(world)
        self._panel(world)
        self._controls()

    def _title(self):
        self.surf.blit(self.f.big.render("FPGA Valet Parking — 자율주행 대시보드", True, C.COL_TITLE),
                       (C.GRID_X, 24))
        self.surf.blit(self.f.small.render(
            "인지(부분관측) · 계획(코스트 A*) · 예측 · 반응 · 제어 — 자율주행 파이프라인 실시간",
            True, C.COL_SUBTLE), (C.GRID_X, 60))

    def _grid(self, world):
        for (x, y), truth in world.truth.items():
            r = self.cell_rect(x, y)
            b = world.belief.get((x, y))
            if b is None:
                # 미관측: fog
                pygame.draw.rect(self.surf, C.COL_FOG, r)
                pygame.draw.rect(self.surf, C.COL_GRID_LINE, r, 1)
                if self.show_fog:
                    q = self.f.h1.render("?", True, (44, 48, 58))
                    self.surf.blit(q, (r.centerx - q.get_width() // 2, r.centery - q.get_height() // 2))
                continue
            col = _TYPE_COL.get(b[0], C.COL_UNKNOWN)
            pygame.draw.rect(self.surf, col, r)
            # 신뢰도 감쇠: 오래 안 본 셀은 흐려짐
            age = world.t - b[1]
            stale = max(0.0, min(0.6, age / C.BELIEF_FADE * 0.6))
            if stale > 0.02:
                ov = pygame.Surface((r.w, r.h), pygame.SRCALPHA)
                ov.fill((*C.COL_FOG, int(stale * 255)))
                self.surf.blit(ov, r.topleft)
            pygame.draw.rect(self.surf, C.COL_GRID_LINE, r, 1)
            if b[0] == W.ENTRY:
                self._ctext(self.f.lbl, "IN", C.COL_TITLE, r)
            elif b[0] == W.EXIT:
                self._ctext(self.f.lbl, "OUT", C.COL_TITLE, r)
            elif b[0] == W.WALL:
                # 막힌 셀(X)
                pygame.draw.line(self.surf, C.COL_WALL, r.topleft, r.bottomright, 3)
                pygame.draw.line(self.surf, C.COL_WALL, r.topright, r.bottomleft, 3)
            # 새로 인지한 셀 강조
            if (x, y) in world.newly_seen:
                pygame.draw.rect(self.surf, C.COL_FOV, r, 3)

    def _walls(self, world):
        """인지된 벽(셀 경계)을 굵은 선으로. 점진적으로 드러난다."""
        for key in world.known_walls:
            kind, a, b = key
            if kind == "V":            # 열 a 오른쪽 경계, 행 b
                xp = C.GRID_X + a * C.CELL
                y0 = C.GRID_Y + (b - 1) * C.CELL
                pygame.draw.line(self.surf, C.COL_WALL, (xp, y0), (xp, y0 + C.CELL), 4)
            else:                      # "H": 행 b 아래 경계, 열 a
                yp = C.GRID_Y + b * C.CELL
                x0 = C.GRID_X + (a - 1) * C.CELL
                pygame.draw.line(self.surf, C.COL_WALL, (x0, yp), (x0 + C.CELL, yp), 4)
        # IN/OUT 개구부 화살표
        ex, ey = world.entry
        self.surf.blit(self.f.lbl.render("IN▸", True, C.COL_INOUT),
                       (C.GRID_X - 34, C.GRID_Y + (ey - 1) * C.CELL + C.CELL // 2 - 8))
        ox, oy = world.exit
        self.surf.blit(self.f.lbl.render("▸OUT", True, C.COL_INOUT),
                       (C.GRID_X + ox * C.CELL + 4, C.GRID_Y + (oy - 1) * C.CELL + C.CELL // 2 - 8))

    def _heatmap(self, world):
        for (x, y), v in world.cost_field.items():
            r = self.cell_rect(x, y).inflate(-C.CELL // 2, -C.CELL // 2)
            col = _lerp(C.COL_HEAT_LOW, C.COL_HEAT_HIGH, v)
            ov = pygame.Surface((r.w, r.h), pygame.SRCALPHA)
            ov.fill((*col, 150))
            self.surf.blit(ov, r.topleft)

    def _candidates(self, world):
        for (path, score, slot) in world.candidate_paths[1:]:
            if len(path) >= 2:
                pts = [self.gpix(x, y) for (x, y) in path]
                pygame.draw.lines(self.surf, C.COL_CAND, False, pts, 2)
            sx, sy = self.gpix(*slot)
            self.surf.blit(self.f.small.render(f"{score:.1f}", True, C.COL_CAND),
                           (sx - 10, sy - 6))

    def _path(self, world):
        if len(world.path) >= 2:
            pts = [self.gpix(x, y) for (x, y) in world.path]
            pygame.draw.lines(self.surf, C.COL_CHOSEN, False, pts, 4)
            for a, b in zip(pts[:-1], pts[1:]):
                self._arrow(a, b)
        if world.goal:
            r = self.cell_rect(*world.goal)
            pygame.draw.rect(self.surf, C.COL_TARGET, r, 4)

    def _arrow(self, a, b):
        ang = math.atan2(b[1] - a[1], b[0] - a[0])
        mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        s = 7
        p2 = (mx - s * math.cos(ang - 0.5), my - s * math.sin(ang - 0.5))
        p3 = (mx - s * math.cos(ang + 0.5), my - s * math.sin(ang + 0.5))
        pygame.draw.polygon(self.surf, C.COL_CHOSEN, [(mx, my), p2, p3])

    def _fov(self, world):
        cx, cy = self.gpix(world.fx, world.fy)
        # 근거리 센서 원
        nr = C.NEAR_RADIUS * C.CELL
        ov = pygame.Surface((C.WIDTH, C.HEIGHT), pygame.SRCALPHA)
        pygame.draw.circle(ov, (*C.COL_FOV, 22), (int(cx), int(cy)), int(nr))
        # 전방 부채꼴
        half = math.radians(C.FOV_HALF_DEG)
        R = C.FOV_RADIUS * C.CELL
        pts = [(cx, cy)]
        steps = 16
        for i in range(steps + 1):
            a = world.heading - half + (2 * half) * i / steps
            pts.append((cx + R * math.cos(a), cy + R * math.sin(a)))
        pygame.draw.polygon(ov, (*C.COL_FOV, 34), pts)
        self.surf.blit(ov, (0, 0))

    def _obstacles(self, world):
        # 예측 궤적
        for kind, pred in world.predictions:
            for i, (gx, gy) in enumerate(pred):
                px, py = self.gpix(gx, gy)
                a = int(150 * (1 - i / max(1, len(pred))))
                srf = pygame.Surface((10, 10), pygame.SRCALPHA)
                pygame.draw.circle(srf, (*C.COL_PRED, a), (5, 5), 4)
                self.surf.blit(srf, (px - 5, py - 5))
        # 장애물 본체
        for o in world.obstacles:
            px, py = self.gpix(o.fx, o.fy)
            if o.kind == "ped":
                pygame.draw.circle(self.surf, C.COL_PED, (int(px), int(py)), 9)
                pygame.draw.circle(self.surf, (20, 20, 24), (int(px), int(py)), 9, 2)
            else:
                col = (235, 70, 70) if o.kind == "amb" else C.COL_DYNCAR
                rr = pygame.Rect(0, 0, 22, 14); rr.center = (px, py)
                pygame.draw.rect(self.surf, col, rr, border_radius=3)
                pygame.draw.rect(self.surf, (20, 20, 24), rr, 2, border_radius=3)
                if o.kind == "amb":
                    self.surf.blit(self.f.small.render("AMB", True, (255, 255, 255)),
                                   (px - 13, py - 6))

    def _ego(self, world):
        px, py = self.gpix(world.fx, world.fy)
        # trail
        self._trail.append((px, py))
        if len(self._trail) > C.TRAIL_MAX:
            self._trail.pop(0)
        for i, (tx, ty) in enumerate(self._trail):
            a = int(90 * (i + 1) / len(self._trail))
            srf = pygame.Surface((8, 8), pygame.SRCALPHA)
            pygame.draw.circle(srf, (*C.COL_EGO, a), (4, 4), 3)
            self.surf.blit(srf, (tx - 4, ty - 4))
        # 안전 버블
        sr = C.SAFETY_R * C.CELL
        scol = C.COL_SAFETY_HIT if world.safety_hit else C.COL_SAFETY
        ov = pygame.Surface((C.WIDTH, C.HEIGHT), pygame.SRCALPHA)
        pygame.draw.circle(ov, (*scol, 40), (int(px), int(py)), int(sr))
        pygame.draw.circle(ov, (*scol, 130), (int(px), int(py)), int(sr), 2)
        self.surf.blit(ov, (0, 0))
        # 차체(heading 방향 삼각형)
        a = world.heading
        L, Wd = 16, 10
        nose = (px + L * math.cos(a), py + L * math.sin(a))
        bl = (px - 8 * math.cos(a) - Wd * math.sin(a), py - 8 * math.sin(a) + Wd * math.cos(a))
        br = (px - 8 * math.cos(a) + Wd * math.sin(a), py - 8 * math.sin(a) - Wd * math.cos(a))
        pygame.draw.polygon(self.surf, C.COL_EGO, [nose, bl, br])
        pygame.draw.polygon(self.surf, (15, 18, 24), [nose, bl, br], 2)

    # ── 우측 패널 ───────────────────────────────────────────
    def _panel(self, world):
        x, y = C.PANEL_X, 24
        tel = world.telemetry()

        # 행동 상태
        y = self._label(x, y, "BEHAVIOR (주행 거동)")
        bcol = C.COL_BEHAVIOR.get(world.behavior, C.COL_TITLE)
        self.surf.blit(self.f.huge.render(world.behavior, True, bcol), (x, y))
        y += 46

        # 속도 게이지
        y = self._label(x, y, "SPEED")
        self._bar(x, y, tel["speed"] / max(0.1, C.V_CRUISE), C.COL_EMPTY,
                  f"{tel['speed']:.2f} cell/s")
        y += 34

        # 최근접 장애물
        y = self._label(x, y, "NEAREST OBSTACLE")
        near = tel["nearest"]
        ncol = C.COL_SAFETY_HIT if near < C.SAFETY_R else C.COL_TITLE
        txt = "—" if near > 9 else f"{near:.2f} cell"
        self.surf.blit(self.f.h1.render(txt, True, ncol), (x, y))
        y += 34

        # 인지율
        y = self._label(x, y, "PERCEIVED MAP")
        self._bar(x, y, tel["perceived"], C.COL_FOV, f"{int(tel['perceived']*100)}%")
        y += 34

        # 재계획 횟수
        self.surf.blit(self.f.small.render(f"REPLANS: {tel['replans']}    "
                                           f"TARGET: {tel['target'] or '-'}",
                                           True, C.COL_SUBTLE), (x, y))
        y += 28

        # Agent AI 판단
        prov = tel.get("provider")
        plabel = "AGENT AI 판단" + (f"  ({prov})" if prov else "")
        y = self._label(x, y, plabel)
        y = self._wrap(self.f.body, world.ai_reason, C.COL_TITLE, x, y, 300, 22)

        self._legend(x, C.HEIGHT - 92)

    def _bar(self, x, y, frac, col, label):
        w = 240
        pygame.draw.rect(self.surf, C.COL_UNKNOWN, (x, y, w, 14), border_radius=3)
        fw = int(w * max(0.0, min(1.0, frac)))
        pygame.draw.rect(self.surf, col, (x, y, fw, 14), border_radius=3)
        self.surf.blit(self.f.small.render(label, True, C.COL_SUBTLE), (x + w + 8, y))

    def _legend(self, x, y):
        items = [("빈 슬롯", C.COL_EMPTY), ("점유", C.COL_OCCUPIED), ("미관측", C.COL_FOG),
                 ("벽", C.COL_WALL), ("보행자", C.COL_PED), ("목표", C.COL_TARGET)]
        self.surf.blit(self.f.lbl.render("LEGEND", True, C.COL_SUBTLE), (x, y))
        y += 20
        for i, (name, col) in enumerate(items):
            ox = x + (i % 3) * 110
            oy = y + (i // 3) * 24
            pygame.draw.rect(self.surf, col, (ox, oy, 14, 14))
            self.surf.blit(self.f.small.render(name, True, C.COL_SUBTLE), (ox + 20, oy + 1))

    def _controls(self):
        flags = [("H 히트맵", self.show_heat), ("C 후보", self.show_cand),
                 ("F fog", self.show_fog), ("V FOV", self.show_fov),
                 ("Space 일시정지", self.paused)]
        x = C.GRID_X
        y = C.GRID_Y + C.GRID_N * C.CELL + 12
        parts = []
        for name, on in flags:
            parts.append(f"[{name}:{'ON' if on else 'off'}]")
        self.surf.blit(self.f.small.render("  ".join(parts) + "   [R 재시작]  [ESC 종료]",
                                           True, C.COL_SUBTLE), (x, y))

    # ── 텍스트 유틸 ─────────────────────────────────────────
    def _label(self, x, y, text):
        self.surf.blit(self.f.lbl.render(text, True, C.COL_SUBTLE), (x, y))
        return y + 20

    def _ctext(self, font, text, color, rect):
        t = font.render(text, True, color)
        self.surf.blit(t, (rect.centerx - t.get_width() // 2, rect.centery - t.get_height() // 2))

    def _wrap(self, font, text, color, x, y, width, lh):
        if not text:
            return y
        line = ""
        for w in text.split(" "):
            test = (line + " " + w).strip()
            if font.size(test)[0] > width and line:
                self.surf.blit(font.render(line, True, color), (x, y)); y += lh; line = w
            else:
                line = test
        if line:
            self.surf.blit(font.render(line, True, color), (x, y)); y += lh
        return y
