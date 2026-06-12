module IMEM (A, RD);
  input  [31:0] A;
  output [31:0] RD;

  (* keep = "true" *) reg [31:0] RAM [0:511];

  initial begin
    $readmemh("program.hex", RAM);
  end

  assign RD = RAM[A[10:2]]; // word-aligned, 512 words (A[31:2] → A[10:2] 수정)
endmodule
