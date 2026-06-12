# main.py — PC 자율주행 대시보드 진입점
#
# config.DATA_SOURCE 로 데이터 소스를 고른다:
#   "sim"      — PC 단독 자율주행 시뮬레이션
#   "emulator" — FPGA 흉내 에뮬레이터 프레임으로 구동(실연동 구조 검증)
#   "serial"   — 실제 FPGA와 UART 통신
#
# 토글 키: H 히트맵 / C 후보 / F fog / V FOV / Space 일시정지 / R 재시작 / ESC 종료

import sys
import threading

import pygame

import config as C
import protocol as P
import agent_ai
from world import World
from dashboard import Dashboard, Fonts


def _handle_keys(dash):
    """공통 키 처리. 반환: ('quit'|'restart'|None)"""
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            return "quit"
        if e.type == pygame.KEYDOWN:
            if e.key == pygame.K_ESCAPE: return "quit"
            if e.key == pygame.K_r:      return "restart"
            if e.key == pygame.K_h: dash.show_heat = not dash.show_heat
            elif e.key == pygame.K_c: dash.show_cand = not dash.show_cand
            elif e.key == pygame.K_f: dash.show_fog = not dash.show_fog
            elif e.key == pygame.K_v: dash.show_fov = not dash.show_fov
            elif e.key == pygame.K_SPACE: dash.paused = not dash.paused
    return None


# ── 시뮬레이션 모드 ─────────────────────────────────────────
def run_sim(screen, clock, dash):
    world = World(C.GRID_N)
    while True:
        dt = clock.tick(C.FPS) / 1000.0
        act = _handle_keys(dash)
        if act == "quit": return
        if act == "restart": world = World(C.GRID_N); dash._trail = []
        if not dash.paused:
            world.update(dt)
        dash.draw(world)
        pygame.display.flip()


# ── 실데이터 모드(emulator/serial) ──────────────────────────
def run_live(screen, clock, dash):
    from serial_comm import make_link

    def new_session():
        w = World(C.GRID_N, live=True)
        lk = make_link(); lk.start()
        return w, lk, {"decided": False, "pending": False, "box": {}}

    world, link, st = new_session()

    while True:
        dt = clock.tick(C.FPS) / 1000.0
        act = _handle_keys(dash)
        if act == "quit":
            link.stop(); return
        if act == "restart":
            link.stop(); world, link, st = new_session()

        # 1) FPGA 컴팩트 프레임 수신 → 상태 반영
        for line in link.poll():
            world.apply_live(line)

        # 2) 스캔 완료 → LLM 최적주차 판단(비동기)
        if world.scan_done_flag and not st["decided"] and not st["pending"]:
            st["pending"] = True
            world.behavior = "DECIDING"
            world.ai_reason = "최적 주차 위치 분석 중 — Agent AI 판단"
            ctx = _context(world)
            threading.Thread(target=lambda: st["box"].__setitem__("res", agent_ai.decide_parking(ctx)),
                             daemon=True).start()

        # 3) 판단 완료 → CMD 송신
        if st["pending"] and "res" in st["box"]:
            res = st["box"].pop("res"); st["pending"] = False; st["decided"] = True
            slot = res.get("slot")
            if slot:
                world.target_slot = world.goal = tuple(slot)
                world.ai_reason = res.get("reason", "") + f"  [{res.get('provider','')}]"
                world.ai_provider = res.get("provider", "")
                # PC→FPGA 컴팩트 목표 프레임:  T<x><y>
                link.send(f"T{slot[0]}{slot[1]}\n")
            else:
                world.ai_reason = "빈 주차칸을 찾지 못함"

        if not dash.paused:
            world.update(dt)
        dash.draw(world)
        pygame.display.flip()


def _context(world):
    empty = [list(k) for k, b in world.belief.items() if b[0] == 2]
    occ = [list(k) for k, b in world.belief.items() if b[0] == 3]
    return {"n": world.n, "entry": list(world.entry), "exit": list(world.exit),
            "pos": list(world.ego_cell()), "empty": empty, "occupied": occ, "dist": {}}


def main():
    pygame.init()
    pygame.display.set_caption(C.TITLE)
    screen = pygame.display.set_mode((C.WIDTH, C.HEIGHT))
    clock = pygame.time.Clock()
    dash = Dashboard(screen, Fonts())

    src = getattr(C, "DATA_SOURCE", "sim")
    if src == "sim":
        run_sim(screen, clock, dash)
    else:
        run_live(screen, clock, dash)

    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
