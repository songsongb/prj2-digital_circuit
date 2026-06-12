module DATAPATH(
        // Outputs
        ZERO, PC, ALURESULT, WRITEDATA,
        // Inputs
        CLK, RESET, RESULTSRC, PCSRC, ALUSRC, REGWRITE, JALR,
        IMMSRC, ALUCONTROL, INSTR, READDATA
        );

  input         CLK;
  input         RESET;        // = RSTN (active-low)
  input  [31:0] INSTR;
  input  [31:0] READDATA;
  input  [1:0]  RESULTSRC;
  input  [2:0]  IMMSRC;
  input         PCSRC, ALUSRC, REGWRITE, JALR;
  input  [3:0]  ALUCONTROL;   // 3 → 4 비트
  output [31:0] PC;
  output [31:0] ALURESULT;
  output [31:0] WRITEDATA;
  output        ZERO;

  wire [31:0] PCwire;
  wire [31:0] PCPlus4;
  wire [31:0] PCTarget;
  wire [31:0] JumpTarget;
  wire [31:0] result;
  wire [31:0] SrcA, SrcB;
  wire [31:0] rf_rd2;
  wire [31:0] ImmExt;

  assign PC       = PCwire;
  assign PCPlus4  = PCwire + 32'd4;
  assign PCTarget = PCwire + ImmExt;                  // PC 상대 (JAL/branch/AUIPC)

  // JALR 점프 목표 = (rs1 + imm), bit0 클리어. 그 외에는 PC 상대 목표.
  assign JumpTarget = JALR ? {ALURESULT[31:1], 1'b0} : PCTarget;

  // 결과 선택: 00=ALU 01=MEM 10=PC+4 11=PCTarget(AUIPC)
  assign result   = (RESULTSRC == 2'b00) ? ALURESULT
                  : (RESULTSRC == 2'b01) ? READDATA
                  : (RESULTSRC == 2'b10) ? PCPlus4
                  :                        PCTarget;

  assign SrcB     = ALUSRC ? ImmExt : rf_rd2;
  assign WRITEDATA = rf_rd2;

  PC pc (
    .PC       (PCwire),
    .clk      (CLK),
    .reset    (RESET),
    .PCPlus4  (PCPlus4),
    .PCTarget (JumpTarget),    // ← JALR 반영된 목표
    .PCSrc    (PCSRC)
  );

  REGFILE regfile (
    .RD1 (SrcA),
    .RD2 (rf_rd2),
    .CLK (CLK),
    .WE3 (REGWRITE),
    .A1  (INSTR[19:15]),
    .A2  (INSTR[24:20]),
    .A3  (INSTR[11:7]),
    .WD3 (result)
  );

  ALU alu (
    .result     (ALURESULT),
    .zero       (ZERO),
    .a          (SrcA),
    .b          (SrcB),
    .alucontrol (ALUCONTROL)
  );

  EXTEND extend (
    .IMMEXT  (ImmExt),
    .INSTR   (INSTR[31:7]),
    .IMMSRC  (IMMSRC)
  );

endmodule
