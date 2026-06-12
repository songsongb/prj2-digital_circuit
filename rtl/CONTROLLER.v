//=============================================================
// CONTROLLER.v
//  - 모든 분기 타입(beq/bne/blt/bge/bltu/bgeu) 지원
//  - JALR 신호 전달
//=============================================================
module CONTROLLER(
          input  [6:0]  OP,
          input  [2:0]  FUNCT3,
          input         FUNCT7B5,
          input         ZERO,       // ALU result==0   (a==b 판정)
          input         LT,         // ALURESULT[0]     (a<b 판정: SLT/SLTU)
          output [1:0]  RESULTSRC,
          output        MEMWRITE,
          output        PCSRC,
          output        ALUSRC,
          output        REGWRITE,
          output        JUMP,
          output        JALR,
          output [2:0]  IMMSRC,
          output [3:0]  ALUCONTROL  // 3 → 4 비트
          );

  wire [1:0] ALUOP;
  wire       BRANCH;
  reg        branch_taken;

  MAINDEC MD(
          .OP(OP),
          .RESULTSRC(RESULTSRC),
          .MEMWRITE(MEMWRITE),
          .BRANCH(BRANCH),
          .ALUSRC(ALUSRC),
          .REGWRITE(REGWRITE),
          .JUMP(JUMP),
          .JALR(JALR),
          .IMMSRC(IMMSRC),
          .ALUOP(ALUOP)
          );

  ALUDEC AD(
          .OPB5(OP[5]),
          .FUNCT3(FUNCT3),
          .FUNCT7B5(FUNCT7B5),
          .ALUOP(ALUOP),
          .ALUCONTROL(ALUCONTROL)
          );

  // 분기 조건 판정 (BRANCH=1 일 때만 의미)
  always @(*) begin
    case (FUNCT3)
      3'b000: branch_taken = ZERO;   // beq
      3'b001: branch_taken = ~ZERO;  // bne
      3'b100: branch_taken = LT;     // blt  (SLT)
      3'b101: branch_taken = ~LT;    // bge  (~SLT)
      3'b110: branch_taken = LT;     // bltu (SLTU)
      3'b111: branch_taken = ~LT;    // bgeu (~SLTU)
      default: branch_taken = 1'b0;
    endcase
  end

  assign PCSRC = (BRANCH & branch_taken) | JUMP;

endmodule
