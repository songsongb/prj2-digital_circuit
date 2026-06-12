// =============================================================
// UART_TX.v  (BAUD_DIV 파라미터화)
//   115200 → 434,  9600 → 5208
// =============================================================
module UART_TX #(
    parameter BAUD_DIV = 434          // 기본 115200
)(
    input        CLK,
    input        RESET,               // active-high
    input  [7:0] DATA,
    input        SEND,                // 1클럭 펄스
    output reg   TX,
    output reg   BUSY
);
    localparam IDLE = 2'd0, START = 2'd1, DATA_BITS = 2'd2, STOP = 2'd3;

    reg [1:0]  state;
    reg [15:0] baud_cnt;     // 16비트 (5208 수용)
    reg [2:0]  bit_idx;
    reg [7:0]  tx_shift;

    always @(posedge CLK) begin
        if (RESET) begin
            state <= IDLE; TX <= 1; BUSY <= 0;
            baud_cnt <= 0; bit_idx <= 0; tx_shift <= 0;
        end else begin
            case (state)
                IDLE: begin
                    TX <= 1; BUSY <= 0;
                    if (SEND) begin
                        tx_shift <= DATA; state <= START;
                        baud_cnt <= 0; BUSY <= 1;
                    end
                end
                START: begin
                    TX <= 0;
                    if (baud_cnt == BAUD_DIV - 1) begin
                        baud_cnt <= 0; bit_idx <= 0; state <= DATA_BITS;
                    end else baud_cnt <= baud_cnt + 1'b1;
                end
                DATA_BITS: begin
                    TX <= tx_shift[bit_idx];
                    if (baud_cnt == BAUD_DIV - 1) begin
                        baud_cnt <= 0;
                        if (bit_idx == 7) state <= STOP;
                        else              bit_idx <= bit_idx + 1'b1;
                    end else baud_cnt <= baud_cnt + 1'b1;
                end
                STOP: begin
                    TX <= 1;
                    if (baud_cnt == BAUD_DIV - 1) begin
                        baud_cnt <= 0; state <= IDLE; BUSY <= 0;
                    end else baud_cnt <= baud_cnt + 1'b1;
                end
                default: begin state <= IDLE; TX <= 1; end
            endcase
        end
    end
endmodule
