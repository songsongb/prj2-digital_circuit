// main_mmio_vision_test.c — HuskyLens 비전 기반 6x6 맵 빌드 + PC 프레임 송신
//
// 아키텍처(TOP_SOC_MMIO_TEST.v):
//   - huskylens_sensor_core(RTL)가 HuskyLens를 자율 폴링(LINE/LINE/TAG/LINE/LINE/OBJECT)
//     하고 결과 + current_mode 를 MMIO로 노출한다.
//   - CPU(이 펌웨어)는 결과를 "읽어서" 맵을 만들고, PC로 프레임을 보낸다.
//     → HuskyLens UART 타이밍은 하드웨어가, 응용 로직(맵빌드·판단준비·송신)은 RISC-V가.
//
// 점유 판단:
//   - TAG 모드 결과 → AprilTag ID로 칸(x,y) 식별 → 현재칸 확정 + EMPTY 표시
//   - OBJECT 모드 결과('car' 학습 ID, count≥1) → 현재칸 OCCUPIED 갱신
//   (전제: 태그는 차가 가리지 않는 위치(스톨 뒷벽)에 부착 → 점유 시에도 ID 인식 가능)
//
// 베어메탈 제약(link.ld): IMEM 작음, const 배열/문자열/점프테이블 금지.
//   → 프레임은 문자 상수로 1바이트씩, tag→cell은 if/else(점프테이블 X).
//
// ── 컴팩트 프레임 (FPGA→PC, '\n' 종료) ──
//   S<n> 상태(1=SCANNING 2=ANALYZING) / P<x><y> 위치 / M<x><y><v> 맵셀(2빈 3점유) / D 스캔완료
//
// 빌드:  make TEST=vision   (Makefile: MAIN=main_mmio_$(TEST)_test.c)

#include "mmio.h"

// ===== 물리 배치에 맞게 사용자가 채울 값 =====
#define TOTAL_PARKING 18        // 전체 주차 스톨 수(스캔 완료 판정)
#define CAR_OBJ_ID    1         // Object recognition에서 'car'로 학습한 ID

#define M_NONE  0
#define M_EMPTY 2
#define M_OCC   3

static unsigned char map6[6][6];   // [y-1][x-1] : 0 미발견 / 2 빈칸 / 3 점유 (BSS=0초기화)
static unsigned char seen_count;
static unsigned char cur_x, cur_y; // 마지막 태그가 가리킨 칸(1~6, 0=미설정)
static unsigned char scan_done_sent;

// ── PC 송신(PC_TX_BUSY로 페이싱 — HL_TXBYTE read의 bit1) ──
static void pc_putc(unsigned char c) {
    while (MMIO_READ(HL_TXBYTE) & PC_TX_BUSY_BIT) ;
    MMIO_WRITE(PC_TX_DATA, c);
    MMIO_WRITE(PC_TX_SEND, 1);
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
//   ★ 아래 ID(1~18)를 "실제 부착한 태그 ID"로 바꾸세요. 좌표는 PC 맵과 동일(x=열,y=행).
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
    send_state(1);                          // SCANNING

    while (1) {
        unsigned int st = MMIO_READ(HL_STATUS);
        if (st & 1u) {                       // DATA_VALID (센서코어 결과 도착)
            unsigned int mode = MMIO_READ(HL_MODE) & 0x3u;
            unsigned int res  = MMIO_READ(HL_RESULT);
            unsigned char id  = (unsigned char)((res >> 16) & 0xFF);

            if (mode == MODE_TAG) {
                unsigned char cell = tag_cell(id);
                if (cell) {
                    cur_x = cell & 0x0F;
                    cur_y = (cell >> 4) & 0x0F;
                    send_pos(cur_x, cur_y);
                    if (map6[cur_y - 1][cur_x - 1] == M_NONE)
                        mark(cur_x, cur_y, M_EMPTY);
                }
            } else if (mode == MODE_OBJECT) {
                unsigned int cnt = MMIO_READ(HL_COUNT);
                if (cnt != 0 && id == CAR_OBJ_ID && cur_x != 0) {
                    if (map6[cur_y - 1][cur_x - 1] != M_OCC)
                        mark(cur_x, cur_y, M_OCC);
                }
            }
            // mode == MODE_LINE 결과는 맵빌드에 무시(차량은 사람이 운전)

            MMIO_WRITE(HL_STATUS, 1);        // 래치 클리어
        } else if (st & 2u) {                // NO_RESULT
            MMIO_WRITE(HL_STATUS, 1);
        }

        if (!scan_done_sent && seen_count >= TOTAL_PARKING) {
            scan_done_sent = 1;
            send_done();
            send_state(2);                   // ANALYZING (PC가 LLM 판단)
        }
    }
    return 0;
}
