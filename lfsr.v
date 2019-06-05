module lfsr (
	     output [31:0] lfsrVal, // lfsr current value
	     output [7:0]  psrByte, // psuedo random byte
	     input [31:0]  ldVal, //  load value to LFSR
             input 	   ldLFSR, // load the LFSR, up to 32 bits
             input 	   step,    // advance the LFSR 
	     input 	   rst,
	     input 	   clk);
		  
	wire [30:0] lfsr30;
	wire [30:0] after;
	assign after[30:1] = lfsr30[29:0];
	assign after[0] = lfsr30[2] ^ lfsr30[5] ^ lfsr30[6] ^ lfsr30[12] ^ lfsr30[30];

	regrce #(31) lfsr(
		.q(lfsr30),
		.d(ldLFSR ? ldVal[30:0] : after),
		.ce(ldLFSR || step),
		.rst(rst),
		.clk(clk));
	
	assign lfsrVal = {1'b0, lfsr30};
	assign psrByte = lfsr30[7:0] ^ lfsr30[15:8] ^ lfsr30[23:16] ^ {1'b1, lfsr30[30:24]};

endmodule // lfsr
