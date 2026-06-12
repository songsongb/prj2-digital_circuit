// main.c — HuskyLens 3모드(TAG→OBJECT→LINE) 순환 폴링
//   * 25ms 틱 기반. 모드당 일정 시간(DWELL) 유지하여 알고리즘 스래싱 방지.
//   * 멀티바이트 프레임을 바이트별로 직접 송신(busy 폴링) + 체크섬 계산.
//   * 곱셈/나눗셈/나머지/배열/상수테이블/문자열 미사용 (베어메탈 제약 준수).
#include "mmio.h"

// 순환 모드
#define MODE_TAG     0
#define MODE_OBJECT  1
#define MODE_LINE    2

// 타이밍 (25ms 틱 단위)
#define DWELL_TICKS   40   // 모드당 ~1초
#define SETTLE_TICKS   6   // 전환 후 모델 로딩 대기 ~150ms
#define REQ_PERIOD     4   // ~100ms 마다 결과 요청

// 좌표 임계값 (HuskyLens 320폭, 중앙 160)
#define X_LEFT       120
#define X_RIGHT      200

// ── 1바이트 송신: HL_TX_BUSY 가 내려갈 때까지 대기 후 전송 ──
static void hl_put(unsigned char b) {
    while (MMIO_READ(HL_TXBYTE) & 1u) ;     // busy 대기
    MMIO_WRITE(HL_TXBYTE, b);
}

// 알고리즘 전환:  55 AA 11 02 2D <algoLo> <algoHi=00> <chk>
static void hl_set_algo(unsigned char algo) {
    unsigned char sum = (unsigned char)(0x55 + 0xAA + 0x11 + 0x02 + 0x2D + algo);
    hl_put(0x55); hl_put(0xAA); hl_put(0x11);
    hl_put(0x02); hl_put(0x2D);
    hl_put(algo); hl_put(0x00);
    hl_put(sum);
}

// 결과 요청:  55 AA 11 00 20 30
static void hl_request(void) {
    unsigned char sum = (unsigned char)(0x55 + 0xAA + 0x11 + 0x00 + 0x20);
    hl_put(0x55); hl_put(0xAA); hl_put(0x11);
    hl_put(0x00); hl_put(0x20);
    hl_put(sum);
}

static void show(unsigned char led, unsigned char seg) {
    MMIO_WRITE(LED_OUT, led);
    MMIO_WRITE(SEG_OUT, seg);
}

// 좌표 → 좌/중/우 판정 후 LED/7세그 출력
static void steer(unsigned int x) {
    if      (x < X_LEFT)  show(LED_LEFT,  SEG_L);
    else if (x > X_RIGHT) show(LED_RIGHT, SEG_R);
    else                  show(LED_GO,    SEG_G);
}

static void process(unsigned char mode) {
    if (mode == MODE_LINE) {
        unsigned int bb = MMIO_READ(HL_BBOX);     // [31:16]=xTarget
        steer((bb >> 16) & 0xFFFFu);
    } else {
        unsigned int xy = MMIO_READ(HL_XY);       // [31:16]=X
        steer((xy >> 16) & 0xFFFFu);
    }
}

int main(void) {
    unsigned char mode     = MODE_TAG;
    unsigned char algo     = ALGO_TAG;
    unsigned char dwell    = 0;
    unsigned char settle   = SETTLE_TICKS;
    unsigned char reqc     = 0;
    unsigned char entering = 0;

    hl_set_algo(algo);            // 초기 알고리즘
    MMIO_WRITE(HL_STATUS, 1);     // 래치 클리어

    while (1) {
        while (MMIO_READ(TIMER_FLAG) == 0) ;   // 25ms 틱 대기
        MMIO_WRITE(TIMER_FLAG, 1);             // 클리어

        if (entering) {
            hl_set_algo(algo);
            MMIO_WRITE(HL_STATUS, 1);
            settle   = SETTLE_TICKS;
            reqc     = 0;
            entering = 0;
        } else if (settle != 0) {
            settle--;
        } else {
            unsigned int st = MMIO_READ(HL_STATUS);
            if (st & 0x01u) {              // DATA_VALID
                process(mode);
                MMIO_WRITE(HL_STATUS, 1);
            } else if (st & 0x02u) {       // NO_RESULT
                show(LED_STOP, SEG_S);
                MMIO_WRITE(HL_STATUS, 1);
            }
            if (reqc == 0) { hl_request(); reqc = REQ_PERIOD; }
            reqc--;
        }

        dwell++;
        if (dwell >= DWELL_TICKS) {
            dwell    = 0;
            entering = 1;
            if      (mode == MODE_TAG)    { mode = MODE_OBJECT; algo = ALGO_OBJECT; }
            else if (mode == MODE_OBJECT) { mode = MODE_LINE;   algo = ALGO_LINE; }
            else                          { mode = MODE_TAG;    algo = ALGO_TAG; }
        }
    }
    return 0;
}
