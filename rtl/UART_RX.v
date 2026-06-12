// =============================================================
// UART_RX.v  (BAUD_DIV 파라미터화)
//   50MHz 기준 1비트 = 50_000_000 / baud 클럭
//   115200 → 434,  9600 → 5208
//   인스턴스별로 .BAUD_DIV() 로 다른 보레이트 지정 가능
// =============================================================
module UART_RX #(
    parameter BAUD_DIV = 434          // 기본 115200
)(
    input            CLK,
    input            RESET,           // active-high
    input            RX,
    output reg [7:0] DATA,
    output reg       VALID
);
    localparam HALF_BAUD = BAUD_DIV / 2;

    localparam IDLE = 2'd0, START = 2'd1, DATA_BITS = 2'd2, STOP = 2'd3;

    reg [1:0]  state;
    reg [15:0] baud_cnt;     // 9600 대응 위해 16비트로 확장 (5208 수용)
    reg [2:0]  bit_idx;
    reg [7:0]  rx_shift;

    reg rx_sync1, rx_sync2;
    always @(posedge CLK) begin
        rx_sync1 <= RX;
        rx_sync2 <= rx_sync1;
    end

    always @(posedge CLK) begin
        if (RESET) begin
            state <= IDLE; baud_cnt <= 0; bit_idx <= 0;
            rx_shift <= 0; DATA <= 0; VALID <= 0;
        end else begin
            VALID <= 0;
            case (state)
                IDLE: begin
                    if (!rx_sync2) begin state <= START; baud_cnt <= 0; end
                end
                START: begin
                    if (baud_cnt == HALF_BAUD - 1) begin
                        if (!rx_sync2) begin
                            state <= DATA_BITS; baud_cnt <= 0; bit_idx <= 0;
                        end else state <= IDLE;   // 노이즈
                    end else baud_cnt <= baud_cnt + 1'b1;
                end
                DATA_BITS: begin
                    if (baud_cnt == BAUD_DIV - 1) begin
                        baud_cnt <= 0;
                        rx_shift[bit_idx] <= rx_sync2;  // LSB first
                        if (bit_idx == 7) state <= STOP;
                        else              bit_idx <= bit_idx + 1'b1;
                    end else baud_cnt <= baud_cnt + 1'b1;
                end
                STOP: begin
                    if (baud_cnt == BAUD_DIV - 1) begin
                        baud_cnt <= 0;
                        if (rx_sync2) begin DATA <= rx_shift; VALID <= 1; end
                        state <= IDLE;
                    end else baud_cnt <= baud_cnt + 1'b1;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
