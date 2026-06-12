module REGFILE (CLK, WE3, A1, A2, A3, WD3, RD1, RD2);

   input CLK;
   input WE3;
   input [4:0] A1, A2, A3;
   input [31:0] WD3;
   output [31:0] RD1, RD2;

   reg [31:0] 	 RF [31:0];

   always@(posedge CLK) begin
      if (WE3) begin
	 RF[A3] <= WD3;
      end
      else ;
   end

   assign RD1 = (!A1) ? 32'd0 : RF[A1];
   assign RD2 = (!A2) ? 32'd0 : RF[A2];

endmodule
