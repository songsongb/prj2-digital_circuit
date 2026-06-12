// =============================================================
// TIMER.v   25ms 주기 틱 플래그
//   * FLAG 는 틱마다 1로 set 되고, 소프트웨어가 CLEAR 할 때까지 유지(latch).
//     (기존 코드는 1클럭 뒤 자동 클리어돼서 폴링이 거의 놓쳤음 → 수정)
//   * 카운터는 항상 자유 진행 → 25ms 주기 일정 유지.
// =============================================================
module TIMER (
    input      CLK,
    input      RESET,   // active-high
    input      CLEAR,   // 소프트웨어 클리어 펄스 (MMIO write)
    output reg FLAG
);
    localparam TIMER_MAX = 1_250_000;  // 50MHz × 0.025s
    reg [21:0] counter;                // 2^22 = 4,194,304 > 1,250,000

    wire tick = (counter == TIMER_MAX - 1);

    always @(posedge CLK) begin
        if (RESET) begin
            counter <= 0;
            FLAG    <= 0;
        end else begin
            // 자유 진행 카운터
            if (tick) counter <= 0;
            else      counter <= counter + 1'b1;

            // FLAG: 틱에서 set 우선, 아니면 CLEAR 시 해제
            if (tick)        FLAG <= 1'b1;
            else if (CLEAR)  FLAG <= 1'b0;
        end
    end
endmodule
