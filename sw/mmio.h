// mmio.h  — MMIO 주소/상수 정의 (RTL과 1:1 정합)
#ifndef MMIO_H
#define MMIO_H
#include <stdint.h>

#define MMIO_WRITE(addr, val)  (*(volatile uint32_t*)(addr) = (uint32_t)(val))
#define MMIO_READ(addr)        (*(volatile uint32_t*)(addr))

// ── HuskyLens ──
#define HL_RESULT    0x10000000  // [31:24]CMD(2A=BLOCK/2B=ARROW) [23:16]ID [15:8]Xlow [7:0]Ylow (RO)
#define HL_BBOX      0x10000004  // [31:16]W [15:0]H  (ARROW: W=xTarget, H=yTarget) (RO)
#define HL_TXBYTE    0x10000008  // write=1바이트 송신 / read[0]=HL_TX_BUSY [1]=PC_TX_BUSY
#define HL_STATUS    0x1000000C  // [1]NO_RESULT [0]DATA_VALID (write=clear)
#define HL_COUNT     0x10000010  // 감지 객체 수 (RO)
#define OV_WALL      0x10000014  // OV7670 벽감지 [0]=wall [15:8]=front_avg [23:16]=floor_avg (RO)
#define HL_MODE      0x10000018  // 센서코어 현재 모드 [1:0] (0=LINE 1=TAG 2=OBJECT) (RO) ★

// 센서코어 current_mode 값 (huskylens_sensor_core.v와 동일)
#define MODE_LINE    0
#define MODE_TAG     1
#define MODE_OBJECT  2
// PC TX busy 비트 (HL_TXBYTE read의 bit1)
#define PC_TX_BUSY_BIT  0x02

// HuskyLens 공식 알고리즘 코드 (little-endian 2바이트로 전송)
#define ALGO_FACE      0
#define ALGO_OBJTRACK  1
#define ALGO_OBJECT    2   // Object Recognition
#define ALGO_LINE      3   // Line Tracking
#define ALGO_COLOR     4
#define ALGO_TAG       5   // Tag Recognition
#define ALGO_CLASSIFY  6

// ── 하드웨어 출력 ──
#define SEG_OUT      0x10000020  // 7세그(HEX0) raw 패턴 (WO)
#define LED_OUT      0x10000024  // LED (WO)

// ── PC UART ──
#define PC_TX_DATA   0x10000040
#define PC_TX_SEND   0x10000044
#define PC_RX_DATA   0x10000048
#define PC_RX_VALID  0x1000004C

// ── 입력 ──
#define KEY_IN       0x10000050  // [3:0] KEY (RO)  ※ TOP_SOC_MMIO_TEST에서 구현됨
#define SW_IN        0x10000054  // [17:0] SW (RO)

// ── 타이머 ──
#define TIMER_FLAG   0x10000060  // 25ms 틱 (write=clear)
// ※ TOP_SOC_MMIO_TEST.v에서는 TIMER가 CPU에 미연결(항상 0). 폴링 루프로 대체.

// ── LED 비트 ──
#define LED_GO       0x01
#define LED_LEFT     0x02
#define LED_RIGHT    0x04
#define LED_STOP     0x08
#define LED_BACK     0x10
#define LED_SCAN_OK  0x80

// ── 7세그 방향문자 (active-low raw 패턴, HEX0 직결) ──
#define SEG_G   0b0010000
#define SEG_L   0b1000111
#define SEG_R   0b0101111
#define SEG_S   0b0010010
#define SEG_B   0b0000011
#define SEG_0   0b1000000
#define SEG_OFF 0b1111111

#endif
