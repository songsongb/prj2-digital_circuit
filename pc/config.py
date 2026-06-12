# config.py — PC 대시보드 전역 설정 (한 곳에서 관리)
# 값 하나만 바꾸면 통신/화면/애니메이션이 일괄 반영되도록 상수만 모아둔다.

# ── 시리얼 통신 ──────────────────────────────────────────────
# baud는 FPGA의 UART_TX.v / UART_RX.v 설정과 반드시 동일해야 한다.
# 9600 사용 → FPGA도 9600으로 맞출 것 (50MHz / 9600 ≈ 5208 분주).
SERIAL_PORT = "COM3"      # Windows 예: "COM3" / Linux 예: "/dev/ttyUSB0"
BAUD        = 9600

# 데이터 소스 선택:
#   "sim"      — PC 단독 자율주행 시뮬레이션(인지·계획·예측·제어 전부 PC)
#   "emulator" — FPGA를 흉내내는 에뮬레이터가 프레임 전송 → 실제 연동 구조를 HW 없이 검증
#   "serial"   — 실제 FPGA와 UART 통신(config.SERIAL_PORT)
DATA_SOURCE = "sim"
USE_MOCK    = True         # (구버전 호환) 미사용

# ── 화면 ────────────────────────────────────────────────────
WIDTH   = 1080
HEIGHT  = 640
FPS     = 60
TITLE   = "FPGA Valet Parking — Autonomous Driving Dashboard"

# ── 6x6 그리드 기하 ─────────────────────────────────────────
GRID_N  = 6
CELL    = 78               # 셀 한 변(px)
GRID_X  = 40               # 그리드 좌상단 origin x
GRID_Y  = 120              # 그리드 좌상단 origin y
# 좌표계 약속: 셀 (x,y)는 1-based. x=열(→오른쪽), y=행(↓아래).
# 입구(1,1)=좌상단, 출구(6,6)=우하단. RISC-V 비트맵과 1:1 정합.

PANEL_X = GRID_X + GRID_N * CELL + 40   # 우측 정보 패널 시작 x

# ── 애니메이션 ──────────────────────────────────────────────
CAR_SPEED      = 2.6       # 가상차량 이동 속도(셀/초). 경로 따라 부드럽게 보간 이동
TRAIL_MAX      = 64        # 지나온 경로 trail 점 최대 개수

# ── 맵 셀 값 (RISC-V map[6][6]와 동일) ──────────────────────
CELL_UNKNOWN  = 0          # 미탐색
CELL_PATH     = 1          # 주행 통로
CELL_EMPTY    = 2          # 빈 주차칸
CELL_OCCUPIED = 3          # 점유 주차칸
CELL_ENTRY    = 4          # 입구
CELL_EXIT     = 5          # 출구

# ── 색상 (R,G,B) ────────────────────────────────────────────
COL_BG        = (18, 20, 26)
COL_TITLE     = (235, 238, 245)
COL_SUBTLE    = (120, 128, 140)
COL_GRID_LINE = (52, 58, 70)

COL_UNKNOWN   = (34, 38, 48)
COL_PATHCELL  = (70, 76, 90)
COL_EMPTY     = (46, 170, 96)     # 빈 주차 = 초록
COL_OCCUPIED  = (210, 72, 72)     # 점유 = 빨강
COL_ENTRY     = (60, 140, 210)    # 입구 = 파랑
COL_EXIT      = (150, 96, 210)    # 출구 = 보라
COL_TARGET    = (90, 180, 255)    # 목표 구획 하이라이트 테두리

COL_PATH_ARROW = (245, 205, 60)   # A* 경로 = 노랑
COL_CAR       = (250, 250, 252)   # 가상차량 본체
COL_CAR_EDGE  = (20, 22, 28)
COL_RCCAR     = (255, 150, 40)    # 실제 RC카(스캔 단계) 아이콘 = 주황
COL_TRAIL     = (245, 205, 60)

# 방향 명령 표시색
COL_DIR = {
    "G": (46, 170, 96),
    "L": (245, 205, 60),
    "R": (245, 205, 60),
    "S": (210, 72, 72),
    "B": (210, 130, 60),
}

# ════════════════════════════════════════════════════════════
#  자율주행 시뮬레이션 (PC가 인지·계획·예측·제어를 매 틱 수행)
# ════════════════════════════════════════════════════════════

# ── 인지(Perception) ────────────────────────────────────────
FOV_RADIUS   = 2.4         # 센서 인지 반경(셀). 이 안의 셀만 "관측"됨
FOV_HALF_DEG = 62          # 전방 시야각 절반(도). 이 부채꼴 안을 멀리 봄
NEAR_RADIUS  = 1.4         # 진행방향과 무관하게 가까이는 다 인지(근거리 센서)
SCAN_RADIUS  = 1.7         # 통로 주행 중 좌우 주차칸 스캔 반경(HuskyLens 좌우 회전)
BELIEF_FADE  = 6.0         # 관측 후 신뢰도 감쇠 시간(초). 오래되면 흐려짐
OCCUPIED_RATIO = 0.45      # 주차칸 중 점유 비율(HuskyLens가 발견할 차량)

# ── 판단(Agent AI / LLM) ────────────────────────────────────
USE_LLM   = False          # True면 실제 LLM API로 최적 주차 판단. 키 없으면 자동 fallback
LLM_PROVIDER = "anthropic" # "anthropic" 또는 "openai"
LLM_MODEL = "claude-3-5-haiku-latest"   # openai 예: "gpt-4o-mini"

# ── 계획(Planning) ──────────────────────────────────────────
COST_W_GOAL    = 1.0       # 목표까지 거리 가중치
COST_W_OBST    = 3.0       # 장애물 인접 비용 가중치
COST_W_UNKNOWN = 1.2       # 미관측 셀 통과 페널티(낙관적 탐색)
REPLAN_PERIOD  = 0.4       # 최소 재계획 주기(초)

# ── 제어/모션(Control) ──────────────────────────────────────
V_CRUISE     = 1.9         # 순항 속도(셀/초)
V_TURN       = 0.9         # 코너 진입 감속 속도
V_PARK       = 0.7         # 주차 접근 속도
ACCEL        = 3.2         # 가/감속도(셀/초^2)
TURN_RATE    = 4.5         # 조향 각속도(rad/초)
SAFETY_R     = 0.9         # 안전 버블 반경(셀). 이 안에 장애물 → 긴급정지
YIELD_HORIZON= 1.6         # 충돌 예측 시간지평(초). 이 안에 충돌예상 → 양보

# ── 동적 장애물(Dynamic obstacle) ───────────────────────────
PRED_HORIZON = 1.6         # 예측 궤적 표시 시간(초)
PRED_STEPS   = 8           # 예측 점 개수

# ── 자율주행 색상 ───────────────────────────────────────────
COL_FOV       = (90, 200, 255)    # 센서 시야 부채꼴
COL_FOG       = (12, 13, 17)      # 미관측 영역
COL_EGO       = (90, 200, 255)    # 자율주행 ego 차량
COL_SAFETY    = (90, 200, 255)    # 안전 버블(정상)
COL_SAFETY_HIT= (235, 70, 70)     # 안전 버블(침범)
COL_PED       = (255, 110, 200)   # 보행자
COL_DYNCAR    = (255, 150, 40)    # 다른 차량
COL_PRED      = (255, 180, 90)    # 예측 궤적
COL_CAND      = (120, 130, 150)   # 후보 경로(비채택)
COL_CHOSEN    = (245, 205, 60)    # 채택 경로
# 코스트 히트맵 (낮음=파랑 → 높음=빨강)
COL_HEAT_LOW  = (40, 70, 140)
COL_HEAT_HIGH = (170, 60, 60)

COL_WALL      = (224, 228, 236)   # 벽(굵은선) — 인지된 부분만 표시
COL_BLOCKED   = (40, 42, 50)      # 막힌 셀(X)
COL_INOUT     = (235, 90, 70)     # IN/OUT 마커

# ── 행동 상태(Behavior) 표시색 ──────────────────────────────
COL_BEHAVIOR = {
    "SCANNING":   (90, 200, 255),
    "EXPLORING":  (90, 200, 255),
    "DECIDING":   (180, 150, 255),
    "CRUISING":   (46, 170, 96),
    "REPLANNING": (245, 205, 60),
    "YIELDING":   (245, 160, 60),
    "EMERGENCY":  (235, 70, 70),
    "PARKING":    (150, 96, 210),
    "DONE":       (120, 200, 140),
}
