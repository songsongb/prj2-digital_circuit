module PC (
    // Outputs
    PC,
    // Inputs
    clk, reset, PCPlus4, PCTarget, PCSrc
);

    input        clk, reset;   // reset = RSTN (active-low)
    input [31:0] PCPlus4, PCTarget;
    input        PCSrc;
    output reg [31:0] PC;

    // 비동기 active-low 리셋
    always @(posedge clk or negedge reset) begin
        if (!reset) PC <= 32'd0;        // reset=0 → 리셋
        else if (PCSrc) PC <= PCTarget; // branch/jump
        else            PC <= PCPlus4;  // 순차 실행
    end

endmodule