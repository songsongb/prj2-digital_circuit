// =============================================================
// TOP_SOC_MMIO_TEST.v
//
// RISC-V + MMIO sensor integration test for Altera DE2.
// One RV32I core reads HuskyLens and OV7670 wall detector values
// through DMEM_MMIO, then writes CPU debug values to LEDR/HEX.
// =============================================================

module TOP_SOC_MMIO_TEST (
    input         CLOCK_50,
    input  [3:0]  KEY,
    input  [17:0] SW,

    input         UART_RXD,
    output        UART_TXD,

    output [6:0]  HEX0,
    output [6:0]  HEX1,
    output [6:0]  HEX2,
    output [6:0]  HEX3,
    output [6:0]  HEX4,
    output [6:0]  HEX5,
    output [6:0]  HEX6,
    output [6:0]  HEX7,

    output [7:0]  LEDG,
    output [17:0] LEDR,

    inout  [35:0] GPIO_0,
    inout  [35:0] GPIO_1
);

    wire rstn;
    wire reset_h;
    wire start_scan;
    wire debug_slow_mode;

    assign rstn            = KEY[0];
    assign reset_h         = ~rstn;
    assign start_scan      = ~KEY[1];
    assign debug_slow_mode = SW[17];

    // RISC-V memory bus
    wire [31:0] pc;
    wire [31:0] instr;
    wire        memwrite;
    wire [31:0] alu_result;
    wire [31:0] write_data;
    wire [31:0] read_data;

    // HuskyLens physical and parsed signals
    wire       hl_uart_rx;
    wire       hl_uart_tx;
    wire [1:0] hl_current_mode;
    wire       hl_mode_tick;
    wire       hl_data_valid;
    wire       hl_no_result;
    wire [7:0] hl_algo;
    wire [7:0] hl_id;
    wire [7:0] hl_x;
    wire [7:0] hl_y;
    wire [15:0] hl_w;
    wire [15:0] hl_h;
    wire [7:0] hl_obj_count;
    wire [2:0] hl_line_cmd;
    wire       hl_uart_rx_activity;
    wire       hl_frame_seen;
    wire       hl_header_seen;
    wire [7:0] hl_rx_count;
    wire [7:0] hl_debug_state;

    // OV7670 / LCM signals
    wire       ov_pclk_alive;
    wire       ov_vsync;
    wire       ov_href;
    wire       wall_detect;
    wire [7:0] front_avg_dbg;
    wire [7:0] floor_avg_dbg;
    wire       wall_update_toggle;
    wire [7:0] camera_pixel_dbg;

    // MMIO debug registers written by firmware
    wire [31:0] hex_mmio_reg;
    wire [17:0] led_mmio_reg;
    wire        cpu_mmio_write_activity;
    reg  [31:0] pc_prev;
    reg  [22:0] pc_activity_count;
    reg         cpu_pc_activity_toggle;

    // HuskyLens 송신 측대역은 미사용(센서코어가 HuskyLens UART 소유).
    // ★ PC UART는 이제 CPU(DMEM_MMIO)가 소유 — 맵 프레임 송신/목표 수신.
    wire [7:0] unused_hl_cmd;
    wire       unused_hl_cmd_valid;
    wire       unused_timer_clear;
    wire [7:0] pc_tx_data;
    wire       pc_tx_send;
    wire       pc_tx_busy;
    wire [7:0] pc_rx_data;
    wire       pc_rx_valid;

    // PC UART 보레이트 (9600 → 50MHz/9600 ≈ 5208). PC config.BAUD와 일치해야 함.
    localparam PC_BAUD_DIV = 5208;

    assign hl_uart_rx = GPIO_1[0];
    assign GPIO_1[1]  = hl_uart_tx;

    always @(posedge CLOCK_50 or negedge rstn) begin
        if (!rstn) begin
            pc_prev                <= 32'd0;
            pc_activity_count      <= 23'd0;
            cpu_pc_activity_toggle <= 1'b0;
        end else begin
            pc_prev <= pc;
            if (pc != pc_prev) begin
                if (pc_activity_count == 23'd4999999) begin
                    pc_activity_count      <= 23'd0;
                    cpu_pc_activity_toggle <= ~cpu_pc_activity_toggle;
                end else begin
                    pc_activity_count <= pc_activity_count + 1'b1;
                end
            end
        end
    end

    RV32I u_rv32i (
        .CLK        (CLOCK_50),
        .RSTN       (rstn),
        .PC         (pc),
        .INSTR      (instr),
        .MEMWRITE   (memwrite),
        .ALURESULT  (alu_result),
        .WRITEDATA  (write_data),
        .READDATA   (read_data)
    );

    IMEM u_imem (
        .A  (pc),
        .RD (instr)
    );

    DMEM_MMIO u_dmem_mmio (
        .CLK              (CLOCK_50),
        .RESET            (reset_h),
        .WE               (memwrite),
        .A                (alu_result),
        .WD               (write_data),
        .RD               (read_data),

        .HL_ALGO_ID       (hl_algo),
        .HL_OBJ_ID        (hl_id),
        .HL_OBJ_X         ({8'd0, hl_x}),
        .HL_OBJ_Y         ({8'd0, hl_y}),
        .HL_OBJ_W         (hl_w),
        .HL_OBJ_H         (hl_h),
        .HL_OBJ_COUNT     (hl_obj_count),
        .HL_DATA_VALID    (hl_data_valid),
        .HL_NO_RESULT     (hl_no_result),
        .HL_CURRENT_MODE  (hl_current_mode),   // ★ TAG/OBJECT 구분용

        .OV_WALL_DETECT   (wall_detect),
        .OV_FRONT_AVG     (front_avg_dbg),
        .OV_FLOOR_AVG     (floor_avg_dbg),

        .HL_CMD           (unused_hl_cmd),
        .HL_CMD_VALID     (unused_hl_cmd_valid),
        .HL_TX_BUSY       (1'b0),

        .PC_RX_DATA       (pc_rx_data),        // ★ CPU가 PC에서 목표(T) 수신
        .PC_RX_VALID      (pc_rx_valid),
        .PC_TX_DATA       (pc_tx_data),        // ★ CPU가 PC로 맵 프레임 송신
        .PC_TX_SEND       (pc_tx_send),
        .PC_TX_BUSY       (pc_tx_busy),

        .TIMER_FLAG       (1'b0),
        .TIMER_CLEAR      (unused_timer_clear),

        .KEY_IN           (KEY),
        .SW_IN            (SW),

        .HEX_OUT          (hex_mmio_reg),
        .LED_OUT          (led_mmio_reg),
        .CPU_MMIO_WRITE_ACTIVITY (cpu_mmio_write_activity)
    );

    huskylens_sensor_core #(
        .DEBUG_SLOW     (0),
        .SLOW_TICK_MAX  (50000000)
    ) u_huskylens_sensor_core (
        .clk              (CLOCK_50),
        .rstn             (rstn),

        .hl_uart_rx       (hl_uart_rx),
        .hl_uart_tx       (hl_uart_tx),

        .pc_uart_rx       (1'b1),       // ★ PC UART는 CPU가 소유 → 센서코어 echo 분리
        .pc_uart_tx       (),           //    (출력 미연결)

        .enable           (1'b1),
        .manual_mode      (1'b0),
        .start_scan       (start_scan),
        .debug_slow_mode  (debug_slow_mode),

        .current_mode     (hl_current_mode),
        .mode_tick        (hl_mode_tick),

        .data_valid       (hl_data_valid),
        .no_result        (hl_no_result),
        .algo             (hl_algo),
        .id               (hl_id),
        .x                (hl_x),
        .y                (hl_y),
        .w                (hl_w),
        .h                (hl_h),
        .obj_count        (hl_obj_count),

        .line_cmd         (hl_line_cmd),

        .uart_rx_activity (hl_uart_rx_activity),
        .frame_seen       (hl_frame_seen),
        .header_seen      (hl_header_seen),
        .rx_count         (hl_rx_count),
        .debug_state      (hl_debug_state)
    );

    ov7670_lcm_wall_core u_ov7670_lcm_wall_core (
        .CLOCK_50           (CLOCK_50),
        .rstn               (rstn),

        .GPIO_0             (GPIO_0),
        .GPIO_1             (GPIO_1),

        .display_mode       (SW[1:0]),
        .byte_phase_sel     (SW[2]),

        .ov_pclk_alive      (ov_pclk_alive),
        .ov_vsync           (ov_vsync),
        .ov_href            (ov_href),
        .wall_detect        (wall_detect),
        .front_avg_dbg      (front_avg_dbg),
        .floor_avg_dbg      (floor_avg_dbg),
        .wall_update_toggle (wall_update_toggle),
        .camera_pixel_dbg   (camera_pixel_dbg)
    );

    // ── CPU 소유 PC UART (맵 프레임 송신 / 목표 수신) ──
    UART_TX #(.BAUD_DIV(PC_BAUD_DIV)) u_cpu_pc_tx (
        .CLK(CLOCK_50), .RESET(reset_h),
        .DATA(pc_tx_data), .SEND(pc_tx_send),
        .TX(UART_TXD), .BUSY(pc_tx_busy)
    );

    UART_RX #(.BAUD_DIV(PC_BAUD_DIV)) u_cpu_pc_rx (
        .CLK(CLOCK_50), .RESET(reset_h), .RX(UART_RXD),
        .DATA(pc_rx_data), .VALID(pc_rx_valid)
    );

    assign LEDG[0]   = ov_vsync;
    assign LEDG[1]   = ov_href;
    assign LEDG[2]   = ov_pclk_alive;
    assign LEDG[3]   = hl_uart_rx_activity;
    assign LEDG[4]   = hl_mode_tick;
    assign LEDG[5]   = rstn;
    assign LEDG[6]   = cpu_pc_activity_toggle;
    assign LEDG[7]   = cpu_mmio_write_activity;

    assign LEDR = led_mmio_reg;

    assign HEX0 = sevenseg_hex(hex_mmio_reg[3:0]);
    assign HEX1 = sevenseg_hex(hex_mmio_reg[7:4]);
    assign HEX2 = sevenseg_hex(hex_mmio_reg[11:8]);
    assign HEX3 = sevenseg_hex(hex_mmio_reg[15:12]);
    assign HEX4 = sevenseg_hex(hex_mmio_reg[19:16]);
    assign HEX5 = sevenseg_hex(hex_mmio_reg[23:20]);
    assign HEX6 = sevenseg_hex(hex_mmio_reg[27:24]);
    assign HEX7 = sevenseg_hex(hex_mmio_reg[31:28]);

    function [6:0] sevenseg_hex;
        input [3:0] value;
        begin
            case (value)
                4'h0: sevenseg_hex = 7'b1000000;
                4'h1: sevenseg_hex = 7'b1111001;
                4'h2: sevenseg_hex = 7'b0100100;
                4'h3: sevenseg_hex = 7'b0110000;
                4'h4: sevenseg_hex = 7'b0011001;
                4'h5: sevenseg_hex = 7'b0010010;
                4'h6: sevenseg_hex = 7'b0000010;
                4'h7: sevenseg_hex = 7'b1111000;
                4'h8: sevenseg_hex = 7'b0000000;
                4'h9: sevenseg_hex = 7'b0010000;
                4'hA: sevenseg_hex = 7'b0001000;
                4'hB: sevenseg_hex = 7'b0000011;
                4'hC: sevenseg_hex = 7'b1000110;
                4'hD: sevenseg_hex = 7'b0100001;
                4'hE: sevenseg_hex = 7'b0000110;
                4'hF: sevenseg_hex = 7'b0001110;
                default: sevenseg_hex = 7'b1111111;
            endcase
        end
    endfunction

endmodule
