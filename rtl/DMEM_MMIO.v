// =============================================================
// DMEM_MMIO.v   데이터 메모리 + MMIO 버스
//
// 주소 맵:
//   0x00000000~0x000003FF : 일반 RAM (256 word = 1KB)
//   0x10000000 (RO) : HL 결과 [31:24]=CMD(2A/2B) [23:16]=ID [15:8]=X [7:0]=Y
//   0x10000004 (RO) : HL bbox  [31:16]=W [15:0]=H  (ARROW면 W=xTarget,H=yTarget)
//   0x10000008 (WO) : HL 송신 바이트 (write=1바이트 송신)
//                (RO) : [0]=HL_TX_BUSY  [1]=PC_TX_BUSY (★ 멀티바이트 프레임 페이싱용)
//   0x1000000C (RW) : HL 상태 [1]=NO_RESULT [0]=DATA_VALID (write=clear)
//   0x10000010 (RO) : HL 객체 수
//   0x10000014 (RO) : OV7670 wall [0]=wall [15:8]=front_avg [23:16]=floor_avg
//   0x10000018 (RO) : HL 현재 모드 [1:0] (0=LINE 1=TAG 2=OBJECT) ★
//   0x10000020 (WO) : HEX debug output, one nibble per HEX digit (32-bit)
//   0x10000024 (WO) : LEDR debug output (18-bit)
//   0x10000040 (WO) : PC UART 송신 데이터
//   0x10000044 (WO) : PC UART 송신 트리거
//   0x10000048 (RO) : PC UART 수신 데이터
//   0x1000004C (RW) : PC UART 수신 유효 (write=clear)
//   0x10000050 (RO) : KEY input
//   0x10000054 (RO) : SW input
//   0x10000060 (RW) : 타이머 플래그 (write=clear)
// =============================================================
module DMEM_MMIO (
    input        CLK,
    input        RESET,     // active-high
    input        WE,
    input  [31:0] A,
    input  [31:0] WD,
    output [31:0] RD,

    // HuskyLens 파서 입력
    input  [7:0]  HL_ALGO_ID,
    input  [7:0]  HL_OBJ_ID,
    input  [15:0] HL_OBJ_X,
    input  [15:0] HL_OBJ_Y,
    input  [15:0] HL_OBJ_W,
    input  [15:0] HL_OBJ_H,
    input  [7:0]  HL_OBJ_COUNT,
    input         HL_DATA_VALID,
    input         HL_NO_RESULT,
    input  [1:0]  HL_CURRENT_MODE,   // ★ 센서코어 현재 모드 (0=LINE 1=TAG 2=OBJECT)

    // OV7670 wall detector input
    input         OV_WALL_DETECT,
    input  [7:0]  OV_FRONT_AVG,
    input  [7:0]  OV_FLOOR_AVG,

    // HuskyLens 송신
    output reg [7:0]  HL_CMD,
    output reg        HL_CMD_VALID,
    input             HL_TX_BUSY,     // ★ 추가

    // PC UART
    input  [7:0]  PC_RX_DATA,
    input         PC_RX_VALID,
    output reg [7:0]  PC_TX_DATA,
    output reg        PC_TX_SEND,
    input             PC_TX_BUSY,    // ★ CPU UART_TX busy (멀티바이트 프레임 페이싱용)

    // 타이머
    input         TIMER_FLAG,
    output reg    TIMER_CLEAR,

    // Board inputs
    input  [3:0]  KEY_IN,
    input  [17:0] SW_IN,

    // 하드웨어 출력
    output reg [31:0] HEX_OUT,
    output reg [17:0] LED_OUT,
    output            CPU_MMIO_WRITE_ACTIVITY
);
    reg [31:0] RAM [0:255];
    localparam CPU_MMIO_ACTIVITY_CYCLES = 24'd10000000;

    reg        hl_valid_latch;
    reg        hl_no_result_latch;
    reg [7:0]  pc_rx_latch;
    reg        pc_rx_valid_latch;
    reg [23:0] cpu_mmio_activity_count;

    reg [31:0] rd_reg;
    assign RD = rd_reg;
    assign CPU_MMIO_WRITE_ACTIVITY = (cpu_mmio_activity_count != 24'd0);

    always @(*) begin
        if (A[31:28] == 4'h1) begin
            case (A[7:0])
                8'h00: rd_reg = {HL_ALGO_ID, HL_OBJ_ID,
                                 HL_OBJ_X[7:0], HL_OBJ_Y[7:0]};
                8'h04: rd_reg = {HL_OBJ_W, HL_OBJ_H};
                8'h08: rd_reg = {30'd0, PC_TX_BUSY, HL_TX_BUSY}; // [0]HL_TX_BUSY [1]PC_TX_BUSY
                8'h0C: rd_reg = {30'd0, hl_no_result_latch, hl_valid_latch};
                8'h10: rd_reg = {24'd0, HL_OBJ_COUNT};
                8'h14: rd_reg = {8'd0, OV_FLOOR_AVG, OV_FRONT_AVG,
                                 7'd0, OV_WALL_DETECT};
                8'h18: rd_reg = {30'd0, HL_CURRENT_MODE};   // ★ 현재 모드 0=LINE 1=TAG 2=OBJECT
                8'h48: rd_reg = {24'd0, pc_rx_latch};
                8'h4C: rd_reg = {31'd0, pc_rx_valid_latch};
                8'h50: rd_reg = {28'd0, KEY_IN};
                8'h54: rd_reg = {14'd0, SW_IN};
                8'h60: rd_reg = {31'd0, TIMER_FLAG};
                default: rd_reg = 32'd0;
            endcase
        end else begin
            rd_reg = RAM[A[9:2]];
        end
    end

    always @(posedge CLK) begin
        if (RESET) begin
            hl_valid_latch     <= 1'b0;
            hl_no_result_latch <= 1'b0;
            pc_rx_latch        <= 8'd0;
            pc_rx_valid_latch  <= 1'b0;
            HL_CMD             <= 8'd0;
            HL_CMD_VALID       <= 1'b0;
            PC_TX_DATA         <= 8'd0;
            PC_TX_SEND         <= 1'b0;
            TIMER_CLEAR        <= 1'b0;
            HEX_OUT            <= 32'd0;
            LED_OUT            <= 18'd0;
            cpu_mmio_activity_count <= 24'd0;
        end else begin
            HL_CMD_VALID <= 1'b0;
            PC_TX_SEND   <= 1'b0;
            TIMER_CLEAR  <= 1'b0;

            if (cpu_mmio_activity_count != 24'd0)
                cpu_mmio_activity_count <= cpu_mmio_activity_count - 1'b1;

            if (HL_DATA_VALID) hl_valid_latch     <= 1'b1;
            if (HL_NO_RESULT)  hl_no_result_latch <= 1'b1;

            if (PC_RX_VALID) begin
                pc_rx_latch       <= PC_RX_DATA;
                pc_rx_valid_latch <= 1'b1;
            end

            if (WE) begin
                if (A[31:28] == 4'h1) begin
                    cpu_mmio_activity_count <= CPU_MMIO_ACTIVITY_CYCLES;
                    case (A[7:0])
                        8'h08: begin
                            HL_CMD       <= WD[7:0];
                            HL_CMD_VALID <= 1'b1;
                        end
                        8'h0C: begin
                            hl_valid_latch     <= 1'b0;
                            hl_no_result_latch <= 1'b0;
                        end
                        8'h20: HEX_OUT    <= WD;
                        8'h24: LED_OUT    <= WD[17:0];
                        8'h40: PC_TX_DATA <= WD[7:0];
                        8'h44: PC_TX_SEND <= 1'b1;
                        8'h4C: pc_rx_valid_latch <= 1'b0;
                        8'h60: TIMER_CLEAR <= 1'b1;
                        default: ;
                    endcase
                end else begin
                    RAM[A[9:2]] <= WD;
                end
            end
        end
    end
endmodule
