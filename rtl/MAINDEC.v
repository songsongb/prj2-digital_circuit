//=============================================================
// MAINDEC.v  (LW/SW/R/BRANCH/I-ALU/JAL/JALR/LUI/AUIPC)
//
// CONTROLS [12:0] =
//  {REGWRITE, IMMSRC[2:0], ALUSRC, MEMWRITE, RESULTSRC[1:0],
//   BRANCH, ALUOP[1:0], JUMP, JALR}
//    12        11:9        8       7         6:5
//    4         3:2          1     0
//
// RESULTSRC : 00=ALU 01=MEM 10=PC+4 11=PCTarget(AUIPC)
// IMMSRC    : 000=I 001=S 010=B 011=J 100=U
// ALUOP     : 00=ADD 01=BRANCH 10=R/I 11=PASSB(LUI)
//=============================================================
module MAINDEC(
         input  [6:0]  OP,
         output [1:0]  RESULTSRC,
         output        MEMWRITE,
         output        BRANCH,
         output        ALUSRC,
         output        REGWRITE,
         output        JUMP,
         output        JALR,
         output [2:0]  IMMSRC,
         output [1:0]  ALUOP
         );

  reg [12:0] CONTROLS;

  always @(*) begin
    case (OP)
      7'b0000011: CONTROLS = 13'b1_000_1_0_01_0_00_0_0; // LW
      7'b0100011: CONTROLS = 13'b0_001_1_1_00_0_00_0_0; // SW
      7'b0110011: CONTROLS = 13'b1_000_0_0_00_0_10_0_0; // R-TYPE
      7'b1100011: CONTROLS = 13'b0_010_0_0_00_1_01_0_0; // BRANCH (beq/bne/blt/bge/bltu/bgeu)
      7'b0010011: CONTROLS = 13'b1_000_1_0_00_0_10_0_0; // I-TYPE ALU
      7'b1101111: CONTROLS = 13'b1_011_0_0_10_0_00_1_0; // JAL
      7'b1100111: CONTROLS = 13'b1_000_1_0_10_0_00_1_1; // JALR
      7'b0110111: CONTROLS = 13'b1_100_1_0_00_0_11_0_0; // LUI   (PASSB)
      7'b0010111: CONTROLS = 13'b1_100_0_0_11_0_00_0_0; // AUIPC (PC+imm)
      default:    CONTROLS = 13'b0_000_0_0_00_0_00_0_0;
    endcase
  end

  assign {REGWRITE, IMMSRC, ALUSRC, MEMWRITE,
          RESULTSRC, BRANCH, ALUOP, JUMP, JALR} = CONTROLS;

endmodule
