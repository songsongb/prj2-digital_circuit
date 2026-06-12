// =============================================================
// ALU.v  (RV32I 완전 지원: 4-bit alucontrol)
//   0000 ADD   0001 SUB   0010 AND   0011 OR    0100 XOR
//   0101 SLT   0110 SLTU  0111 SLL   1000 SRL   1001 SRA
//   1010 PASSB (LUI용: result = b)
// =============================================================
module ALU (
    output reg [31:0] result,
    output            zero,
    input      [31:0] a, b,
    input      [3:0]  alucontrol
);
   wire [31:0] add_r  = a + b;
   wire [31:0] sub_r  = a - b;
   wire        slt_r  = ($signed(a) < $signed(b));   // 부호 비교
   wire        sltu_r = (a < b);                      // 무부호 비교
   wire [4:0]  shamt  = b[4:0];                        // 시프트량은 하위 5비트

   always @(*) begin
      case (alucontrol)
        4'b0000: result = add_r;                  // ADD / ADDI / 주소계산
        4'b0001: result = sub_r;                  // SUB / 분기비교(beq,bne)
        4'b0010: result = a & b;                  // AND / ANDI
        4'b0011: result = a | b;                  // OR  / ORI
        4'b0100: result = a ^ b;                  // XOR / XORI
        4'b0101: result = {31'b0, slt_r};         // SLT / SLTI / blt,bge
        4'b0110: result = {31'b0, sltu_r};        // SLTU/ SLTIU/ bltu,bgeu
        4'b0111: result = a << shamt;             // SLL / SLLI
        4'b1000: result = a >> shamt;             // SRL / SRLI
        4'b1001: result = $signed(a) >>> shamt;   // SRA / SRAI
        4'b1010: result = b;                      // PASSB (LUI)
        default: result = 32'd0;
      endcase
   end

   assign zero = (result == 32'd0);
endmodule
