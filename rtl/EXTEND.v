module EXTEND (INSTR, IMMSRC, IMMEXT);
  input  [31:7] INSTR;
  input  [2:0]  IMMSRC;   // 2비트 → 3비트 확장
  output [31:0] IMMEXT;

  reg [31:0] IMMEXT;

  always @(*) begin
    case (IMMSRC)
      // I-type
      3'b000: IMMEXT = {{20{INSTR[31]}}, INSTR[31:20]};
      // S-type (stores)
      3'b001: IMMEXT = {{20{INSTR[31]}}, INSTR[31:25], INSTR[11:7]};
      // B-type (branches)
      3'b010: IMMEXT = {{20{INSTR[31]}}, INSTR[7], INSTR[30:25], INSTR[11:8], 1'b0};
      // J-type (jal)
      3'b011: IMMEXT = {{12{INSTR[31]}}, INSTR[19:12], INSTR[20], INSTR[30:21], 1'b0};
      // U-type (lui, auipc)  ← 신규 추가
      3'b100: IMMEXT = {INSTR[31:12], 12'b0};
      default: IMMEXT = 32'bx;
    endcase
  end
endmodule
