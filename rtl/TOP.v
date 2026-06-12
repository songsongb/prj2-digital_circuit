// =============================================================
// TOP.v
// =============================================================
module TOP (
    input        CLK,
    input        RSTN,

    input        HL_RX,
    output       HL_TX,

    input        PC_RX,
    output       PC_TX,

    output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7,
    output [7:0] LEDR
);
    // ── 보레이트 분주값 (50MHz 기준) ──
    //   115200 → 434,  9600 → 5208
    //   ★ HuskyLens 시리얼이 9600이면 HL_BAUD_DIV 를 5208 로 바꾸세요.
    localparam HL_BAUD_DIV = 434;   // HuskyLens
    localparam PC_BAUD_DIV = 434;   // PC

    wire RESET_H = ~RSTN;

    wire [31:0] PC_ADDR, INSTR, ALURESULT, WRITEDATA, READDATA;
    wire        MEMWRITE;

    wire [7:0]  hl_algo_id, hl_obj_id, hl_obj_count;
    wire [15:0] hl_obj_x, hl_obj_y, hl_obj_w, hl_obj_h;
    wire        hl_data_valid, hl_no_result;

    wire [7:0]  hl_cmd;
    wire        hl_cmd_valid;
    wire [7:0]  pc_tx_data;
    wire        pc_tx_send;
    wire [7:0]  seg_out, led_out;
    wire        timer_clear;

    wire [7:0]  hl_rx_data, pc_rx_data;
    wire        hl_rx_valid, pc_rx_valid;

    wire        timer_flag;
    wire        hl_tx_busy;

    RV32I cpu (
        .CLK(CLK), .RSTN(RSTN), .PC(PC_ADDR), .INSTR(INSTR),
        .MEMWRITE(MEMWRITE), .ALURESULT(ALURESULT),
        .WRITEDATA(WRITEDATA), .READDATA(READDATA)
    );

    IMEM imem ( .A(PC_ADDR), .RD(INSTR) );

    DMEM_MMIO dmem (
        .CLK(CLK), .RESET(RESET_H), .WE(MEMWRITE),
        .A(ALURESULT), .WD(WRITEDATA), .RD(READDATA),
        .HL_ALGO_ID(hl_algo_id), .HL_OBJ_ID(hl_obj_id),
        .HL_OBJ_X(hl_obj_x), .HL_OBJ_Y(hl_obj_y),
        .HL_OBJ_W(hl_obj_w), .HL_OBJ_H(hl_obj_h),
        .HL_OBJ_COUNT(hl_obj_count),
        .HL_DATA_VALID(hl_data_valid), .HL_NO_RESULT(hl_no_result),
        .HL_CMD(hl_cmd), .HL_CMD_VALID(hl_cmd_valid),
        .HL_TX_BUSY(hl_tx_busy),
        .PC_RX_DATA(pc_rx_data), .PC_RX_VALID(pc_rx_valid),
        .PC_TX_DATA(pc_tx_data), .PC_TX_SEND(pc_tx_send),
        .TIMER_FLAG(timer_flag), .TIMER_CLEAR(timer_clear),
        .SEG_OUT(seg_out), .LED_OUT(led_out)
    );

    UART_RX #(.BAUD_DIV(HL_BAUD_DIV)) uart_hl_rx (
        .CLK(CLK), .RESET(RESET_H), .RX(HL_RX),
        .DATA(hl_rx_data), .VALID(hl_rx_valid)
    );

    HL_PARSER hl_parser (
        .CLK(CLK), .RESET(RESET_H),
        .RX_DATA(hl_rx_data), .RX_VALID(hl_rx_valid),
        .ALGO_ID(hl_algo_id), .OBJ_ID(hl_obj_id),
        .OBJ_X(hl_obj_x), .OBJ_Y(hl_obj_y),
        .OBJ_W(hl_obj_w), .OBJ_H(hl_obj_h),
        .OBJ_COUNT(hl_obj_count),
        .DATA_VALID(hl_data_valid), .NO_RESULT(hl_no_result)
    );

    UART_TX #(.BAUD_DIV(HL_BAUD_DIV)) uart_hl_tx (
        .CLK(CLK), .RESET(RESET_H), .DATA(hl_cmd), .SEND(hl_cmd_valid),
        .TX(HL_TX), .BUSY(hl_tx_busy)
    );

    UART_RX #(.BAUD_DIV(PC_BAUD_DIV)) uart_pc_rx (
        .CLK(CLK), .RESET(RESET_H), .RX(PC_RX),
        .DATA(pc_rx_data), .VALID(pc_rx_valid)
    );

    // ── 디버그 스니퍼: HuskyLens 수신 바이트를 그대로 PC UART(RS232)로 echo ──
    //   K25로 들어온 모든 바이트가 MobaXterm 시리얼 창에 그대로 뜸 (115200, 8N1)
    //   (원래 CPU의 pc_tx_data/pc_tx_send 연결을 잠시 대체 — 진단용)
    UART_TX #(.BAUD_DIV(PC_BAUD_DIV)) uart_pc_tx (
        .CLK(CLK), .RESET(RESET_H), .DATA(hl_rx_data), .SEND(hl_rx_valid),
        .TX(PC_TX), .BUSY()
    );

    TIMER timer_inst (
        .CLK(CLK), .RESET(RESET_H), .CLEAR(timer_clear), .FLAG(timer_flag)
    );

    // ── 디버그: 수신 가시화 (노이즈 무관 계측) ──
    reg        byte_seen, frame_seen, header_seen;
    reg [7:0]  rx_count;     // 수신 바이트 누적 카운터 → 활동량(LED 변하면 바이트 흐름)
    reg [1:0]  hdr_state;    // 0x55→0xAA→0x11 시퀀스 검출 (노이즈로는 거의 안 뜸)
    always @(posedge CLK) begin
        if (RESET_H) begin
            byte_seen <= 1'b0; frame_seen <= 1'b0; header_seen <= 1'b0;
            rx_count  <= 8'd0; hdr_state <= 2'd0;
        end else begin
            if (hl_rx_valid) begin
                byte_seen <= 1'b1;
                rx_count  <= rx_count + 1'b1;
                case (hdr_state)
                    2'd0: hdr_state <= (hl_rx_data == 8'h55) ? 2'd1 : 2'd0;
                    2'd1: hdr_state <= (hl_rx_data == 8'hAA) ? 2'd2 :
                                       (hl_rx_data == 8'h55) ? 2'd1 : 2'd0;
                    2'd2: begin
                              if (hl_rx_data == 8'h11) header_seen <= 1'b1;
                              hdr_state <= (hl_rx_data == 8'h55) ? 2'd1 : 2'd0;
                          end
                    default: hdr_state <= 2'd0;
                endcase
            end
            if (hl_data_valid | hl_no_result) frame_seen <= 1'b1;
        end
    end

    // 7세그: HEX0 에 raw 7-seg 패턴 직결 (mmio.h 의 SEG_x 상수 그대로 사용)
    //   SEG_OUT[6:0] = {g,f,e,d,c,b,a}, active-low (0=점등)
    assign HEX0 = seg_out[6:0];
    assign HEX1 = 7'b1111111;   // 미사용(off)

    assign HEX2 = 7'b1111111;
    assign HEX3 = 7'b1111111;
    assign HEX4 = 7'b1111111;
    assign HEX5 = 7'b1111111;
    assign HEX6 = 7'b1111111;
    assign HEX7 = 7'b1111111;

    assign LEDR = {byte_seen, frame_seen, header_seen, rx_count[4:0]};
//   LEDR[7]=byte_seen(노이즈오염,무시)  LEDR[6]=frame_seen(유효결과프레임)
//   LEDR[5]=header_seen(55 AA 11 수신★)  LEDR[4:0]=수신바이트 카운터(변하면 흐름有)
    
endmodule
