#define MMIO32(addr) (*(volatile unsigned int *)(addr))

#define HL_RESULT  MMIO32(0x10000000u)
#define HL_BBOX    MMIO32(0x10000004u)
#define HL_STATUS  MMIO32(0x1000000Cu)
#define HL_COUNT   MMIO32(0x10000010u)
#define OV_WALL    MMIO32(0x10000014u)
#define HEX_DEBUG  MMIO32(0x10000020u)
#define LED_DEBUG  MMIO32(0x10000024u)

#define CMD_STOP   0u
#define CMD_GO     1u
#define CMD_LEFT   2u
#define CMD_RIGHT  3u

int main(void)
{
    unsigned int blink = 0u;

    while (1) {
        unsigned int hl_result = HL_RESULT;
        unsigned int hl_bbox = HL_BBOX;
        unsigned int hl_status = HL_STATUS;
        unsigned int hl_count = HL_COUNT;
        unsigned int ov_wall = OV_WALL;

        unsigned int algo = (hl_result >> 24) & 0xffu;
        unsigned int id = (hl_result >> 16) & 0xffu;
        unsigned int x = (hl_result >> 8) & 0xffu;
        unsigned int y = hl_result & 0xffu;
        unsigned int w = (hl_bbox >> 16) & 0xffffu;
        unsigned int data_valid = hl_status & 1u;
        unsigned int no_result = (hl_status >> 1) & 1u;
        unsigned int wall_detect = ov_wall & 1u;
        unsigned int front_avg = (ov_wall >> 8) & 0xffu;
        unsigned int floor_avg = (ov_wall >> 16) & 0xffu;

        unsigned int final_cmd = CMD_GO;
        if (wall_detect) {
            final_cmd = CMD_STOP;
        } else if (data_valid && !no_result && algo == 0x2bu) {
            if (w >= 130u && w <= 190u) {
                final_cmd = CMD_GO;
            } else if (w > 190u) {
                final_cmd = CMD_LEFT;
            } else {
                final_cmd = CMD_RIGHT;
            }
        }
        /* TODO: expose huskylens_sensor_core line_cmd directly by MMIO later. */

        LED_DEBUG =
            (wall_detect << 0) |
            (data_valid << 1) |
            (no_result << 2) |
            ((final_cmd == CMD_STOP) << 3) |
            ((final_cmd == CMD_GO) << 4) |
            ((final_cmd == CMD_LEFT) << 5) |
            ((final_cmd == CMD_RIGHT) << 6) |
            ((blink >> 20) & 0x80u);

        HEX_DEBUG =
            ((wall_detect & 0xfu) << 0) |
            ((algo & 0xfu) << 4) |
            ((id & 0xfu) << 8) |
            ((x & 0xfu) << 12) |
            ((y & 0xfu) << 16) |
            ((hl_count & 0xfu) << 20) |
            ((front_avg >> 4) << 24) |
            ((floor_avg >> 4) << 28);

        if (hl_status != 0u) {
            HL_STATUS = hl_status & 3u;
        }

        blink++;
    }
}
