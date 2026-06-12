// main_vision.c — HuskyLens 비전 기반 6x6 맵 빌드 + PC 프레임 송신
//
// 동작: TAG↔OBJECT 모드를 번갈아 폴링하며
//   - TAG 인식 → AprilTag ID로 주차칸(x,y) 식별 → 빈칸(EMPTY)으로 표시
//   - OBJECT 인식('car' 학습 ID) → 해당 칸 점유(OCCUPIED)로 갱신
//   각 갱신을 컴팩트 프레임으로 PC에 송신. 전 주차칸 발견 시 SCANDONE.
//
// 베어메탈 제약(link.ld): IMEM 1KB, const 배열/문자열/점프테이블 금지.
//   → 프레임은 문자 상수로 1바이트씩 송신, tag→cell은 if/else(점프테이블 X).
//
// ── 컴팩트 프레임 (FPGA→PC, 모두 '\n' 종료) ──
//   S<n>        FSM 상태 (1=SCANNING, 2=ANALYZING)
//   P<x><y>     차량(스캔 중인 칸) 위치  (x,y = '1'~'6')
//   M<x><y><v>  맵 셀 점유  (v: '2'=빈칸, '3'=점유)
//   D           스캔 완료(SCANDONE)
//
// ※ 빌드: Makefile 의 SRCS 를 `startup.s main_vision.c` 로 바꾸거나
//         이 파일을 main.c 로 교체하세요. (현재 main.c 는 백업 권장)

#include "mmio.h"

// ===== 물리 배치에 맞게 사용자가 채울 값 =====
#define TOTAL_PARKING 18        // 전체 주차 스톨 수(스캔 완료 판정)
#define CAR_OBJ_ID    1         // Object recognition에서 'car'로 학습한 ID
#define PC_TX_DELAY   20000u    // PC 바이트 간 지연(9600 기준 ~1.6ms). 누락 시 키우기

// 폴링 타이밍 (25ms 틱)
#define DWELL_TICKS  32         // 모드당 ~0.8s
#define SETTLE_TICKS  6         // 전환 후 모델 로딩 대기
#define REQ_PERIOD    4         // ~100ms마다 결과 요청

#define M_NONE  0
#define M_EMPTY 2
#define M_OCC   3

static unsigned char map6[6][6];   // [y-1][x-1] : 0 미발견 / 2 빈칸 / 3 점유 (BSS=0초기화)
static unsigned char seen_count;
static unsigned char cur_x, cur_y; // 마지막 태그가 가리킨 칸(1~6, 0=미설정)
static unsigned char scan_done_sent;

// ── HuskyLens 송신(busy 폴링) ── (sw/main.c와 동일 검증 로직)
static void hl_put(unsigned char b) {
    while (MMIO_READ(HL_TXBYTE) & 1u) ;
    MMIO_WRITE(HL_TXBYTE, b);
}
static void hl_set_algo(unsigned char algo) {
    unsigned char s = (unsigned char)(0x55 + 0xAA + 0x11 + 0x02 + 0x2D + algo);
    hl_put(0x55); hl_put(0xAA); hl_put(0x11);
    hl_put(0x02); hl_put(0x2D); hl_put(algo); hl_put(0x00); hl_put(s);
}
static void hl_request(void) {
    unsigned char s = (unsigned char)(0x55 + 0xAA + 0x11 + 0x00 + 0x20);
    hl_put(0x55); hl_put(0xAA); hl_put(0x11);
    hl_put(0x00); hl_put(0x20); hl_put(s);
}

// ── PC 송신(busy 신호 없음 → 지연으로 페이싱) ──
static void pc_putc(unsigned char c) {
    volatile unsigned int i;
    MMIO_WRITE(PC_TX_DATA, c);
    MMIO_WRITE(PC_TX_SEND, 1);
    for (i = 0; i < PC_TX_DELAY; i++) ;
}
static void send_state(unsigned char n) {
    pc_putc('S'); pc_putc((unsigned char)('0' + n)); pc_putc('\n');
}
static void send_pos(unsigned char x, unsigned char y) {
    pc_putc('P'); pc_putc((unsigned char)('0' + x));
    pc_putc((unsigned char)('0' + y)); pc_putc('\n');
}
static void send_cell(unsigned char x, unsigned char y, unsigned char v) {
    pc_putc('M'); pc_putc((unsigned char)('0' + x));
    pc_putc((unsigned char)('0' + y));
    pc_putc((unsigned char)('0' + v)); pc_putc('\n');
}
static void send_done(void) { pc_putc('D'); pc_putc('\n'); }

// ── AprilTag ID → 셀.  반환 (y<<4)|x, 미등록=0 ──
//   ★ 아래 ID(1~18)를 "실제 부착한 태그 ID"로 바꾸세요.
//   좌표는 PC 대시보드 맵과 동일(x=열,y=행, 1-based).
static unsigned char tag_cell(unsigned char id) {
    if (id == 1)  return 0x13;   // (x=3,y=1)
    if (id == 2)  return 0x14;   // (4,1)
    if (id == 3)  return 0x15;   // (5,1)
    if (id == 4)  return 0x16;   // (6,1)
    if (id == 5)  return 0x21;   // (1,2)
    if (id == 6)  return 0x31;   // (1,3)
    if (id == 7)  return 0x33;   // (3,3)
    if (id == 8)  return 0x34;   // (4,3)
    if (id == 9)  return 0x35;   // (5,3)
    if (id == 10) return 0x41;   // (1,4)
    if (id == 11) return 0x43;   // (3,4)
    if (id == 12) return 0x44;   // (4,4)
    if (id == 13) return 0x45;   // (5,4)
    if (id == 14) return 0x51;   // (1,5)
    if (id == 15) return 0x62;   // (2,6)
    if (id == 16) return 0x63;   // (3,6)
    if (id == 17) return 0x64;   // (4,6)
    if (id == 18) return 0x65;   // (5,6)
    return 0;
}

static void mark(unsigned char x, unsigned char y, unsigned char v) {
    unsigned char old = map6[y - 1][x - 1];
    map6[y - 1][x - 1] = v;
    if (old == M_NONE && v != M_NONE) seen_count++;
    send_cell(x, y, v);
}

int main(void) {
    unsigned char mode = 0;        // 0=TAG, 1=OBJECT
    unsigned char dwell = 0, settle = SETTLE_TICKS, reqc = 0, entering = 0;

    hl_set_algo(ALGO_TAG);
    MMIO_WRITE(HL_STATUS, 1);
    send_state(1);                 // SCANNING

    while (1) {
        while (MMIO_READ(TIMER_FLAG) == 0) ;
        MMIO_WRITE(TIMER_FLAG, 1);

        if (entering) {
            hl_set_algo(mode ? ALGO_OBJECT : ALGO_TAG);
            MMIO_WRITE(HL_STATUS, 1);
            settle = SETTLE_TICKS; reqc = 0; entering = 0;
        } else if (settle != 0) {
            settle--;
        } else {
            unsigned int st = MMIO_READ(HL_STATUS);
            if (st & 1u) {                       // DATA_VALID
                unsigned int res = MMIO_READ(HL_RESULT);
                unsigned char id = (unsigned char)((res >> 16) & 0xFF);
                if (mode == 0) {                 // TAG 모드
                    unsigned char cell = tag_cell(id);
                    if (cell) {
                        cur_x = cell & 0x0F;
                        cur_y = (cell >> 4) & 0x0F;
                        send_pos(cur_x, cur_y);
                        if (map6[cur_y - 1][cur_x - 1] == M_NONE)
                            mark(cur_x, cur_y, M_EMPTY);
                    }
                } else {                         // OBJECT 모드
                    unsigned int cnt = MMIO_READ(HL_COUNT);
                    if (cnt != 0 && id == CAR_OBJ_ID && cur_x != 0) {
                        if (map6[cur_y - 1][cur_x - 1] != M_OCC)
                            mark(cur_x, cur_y, M_OCC);
                    }
                }
                MMIO_WRITE(HL_STATUS, 1);
            } else if (st & 2u) {                // NO_RESULT
                MMIO_WRITE(HL_STATUS, 1);
            }
            if (reqc == 0) { hl_request(); reqc = REQ_PERIOD; }
            reqc--;

            if (!scan_done_sent && seen_count >= TOTAL_PARKING) {
                scan_done_sent = 1;
                send_done();
                send_state(2);                   // ANALYZING (PC가 LLM 판단)
            }
        }

        dwell++;
        if (dwell >= DWELL_TICKS) {
            dwell = 0; entering = 1; mode ^= 1;  // TAG↔OBJECT 전환
        }
    }
    return 0;
}
