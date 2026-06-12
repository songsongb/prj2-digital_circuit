// =============================================================
// ALUDEC.v  (RV32I 완전 디코드)
//   ALUOP 00 : ADD          (LW/SW 주소계산, JALR/AUIPC)
//   ALUOP 01 : BRANCH       (funct3 로 SUB/SLT/SLTU 선택)
//   ALUOP 11 : PASSB        (LUI)
//   ALUOP 10 : R/I-type ALU (funct3 + funct7[5] 로 결정)
// =============================================================
module ALUDEC(
      input            OPB5,      // opcode[5]  (R-type 판별)
      input  [2:0]     FUNCT3,
      input            FUNCT7B5,  // INSTR[30]  (SUB / SRA 판별)
      input  [1:0]     ALUOP,
      output reg [3:0] ALUCONTROL // 3 → 4 비트
      );
   wire RTYPESUB = FUNCT7B5 & OPB5;   // R-type SUB 검출

   always @(*) begin
      case (ALUOP)
        2'b00: ALUCONTROL = 4'b0000;        // ADD
        2'b11: ALUCONTROL = 4'b1010;        // PASSB (LUI)
        2'b01: begin                        // BRANCH 비교 연산
           case (FUNCT3)
             3'b000, 3'b001: ALUCONTROL = 4'b0001; // beq / bne  → SUB (zero 사용)
             3'b100, 3'b101: ALUCONTROL = 4'b0101; // blt / bge  → SLT
             3'b110, 3'b111: ALUCONTROL = 4'b0110; // bltu/ bgeu → SLTU
             default:        ALUCONTROL = 4'b0001;
           endcase
        end
        default: begin                      // 2'b10 : R / I-type
           case (FUNCT3)
             3'b000: ALUCONTROL = (RTYPESUB) ? 4'b0001 : 4'b0000; // SUB / ADD(I)
             3'b001: ALUCONTROL = 4'b0111;                         // SLL / SLLI
             3'b010: ALUCONTROL = 4'b0101;                         // SLT / SLTI
             3'b011: ALUCONTROL = 4'b0110;                         // SLTU/ SLTIU
             3'b100: ALUCONTROL = 4'b0100;                         // XOR / XORI
             3'b101: ALUCONTROL = (FUNCT7B5) ? 4'b1001 : 4'b1000;  // SRA(I)/SRL(I)
             3'b110: ALUCONTROL = 4'b0011;                         // OR  / ORI
             3'b111: ALUCONTROL = 4'b0010;                         // AND / ANDI
             default: ALUCONTROL = 4'b0000;
           endcase
        end
      endcase
   end
endmodule
