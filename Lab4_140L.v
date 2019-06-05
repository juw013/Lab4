module Lab4_140L (
		  input wire   rst, // reset signal (active high)
		  input wire   clk,
		  input        bu_rx_data_rdy, // data from the uart is ready
		  input [7:0]  bu_rx_data, // data from the uart
		  output       L4_tx_data_rdy, // data ready to be sent to UART
		  output [7:0] L4_tx_data, // data to be sent to UART
		  output       L4_PrintBuf,
		  output [4:0] L4_led
		  );

   wire 	   sccDecrypt;    // processing a decrypt command
   wire 	   sccEncrypt;    // processing an encrypt command
   wire            sccEldByte;    // load a byte to encrypt
   wire            sccEmsBitsLd;  // load the msbits of the newly decrypted data
   wire            sccElsBitsLd;  // load the lsbits of the newly decrypted data
   wire 	   sccEmsBitsSl;  // select the ms bits of newly decrypted data
   wire 	   sccDnibble1En; // enable capture of the ms bits of encrypted data
   wire 	   sccDnibble2En; // enable capture of the ls bits of encrypted data
   wire 	   sccDByteValid; // decrypted byte is valid *not used*
   wire [7:0]      sccLdKey;      // load one of 8 key 4-bit registers
   wire 	   sccLdLFSR;     // load the LFSR from the key regsiter 
   wire 	   scdCharIsValid; // bu_rx_data is a printable character
	
	wire [7:0]	sccPtKey;
	wire		sccPt_key;
	wire		sccPtLFSR;
   
   scdp scdp (
	      .L4_tx_data(L4_tx_data),
	      .scdCharIsValid(scdCharIsValid),
	      .bu_rx_data(bu_rx_data),
	      .bu_rx_data_rdy(bu_rx_data_rdy),
	      .sccEncrypt(sccEncrypt),
	      .sccEldByte(sccEldByte),
	      .sccEmsBitsLd(sccEmsBitsLd),
	      .sccElsBitsLd(sccElsBitsLd),
	      .sccEmsBitsSl(sccEmsBitsSl),
	      .sccDecrypt(sccDecrypt),
	      .sccDnibble1En(sccDnibble1En),
	      .sccDnibble2En(sccDnibble2En),

	      .sccLdKey(sccLdKey),
	      .sccLdLFSR(sccLdLFSR),
			
			//Enhencement
			.sccPtKey(sccPtKey),
			.sccPt_key(sccPt_key),
			.sccPtLFSR(sccPtLFSR),

	      .rst(rst),
	      .clk(clk)
	      );
   


   wire 	   de_validAscii;
   wire 	   de_bigD;
   wire 	   de_bigE;
   wire 	   de_bigL;
   wire 	   de_bigP;
   wire 	   de_bigS;
   wire 	   de_hex;
   wire 	   de_cr;
	
	wire		de_littlep;		//small p 
	wire		de_bigR;			//big R

   decodeKeysL4 dk (
		    .de_esc(),
		    .de_validAscii(de_validAscii),
		    .de_bigD(de_bigD),
		    .de_bigE(de_bigE),
		    .de_bigL(de_bigL),
		    .de_bigP(de_bigP),
		    .de_bigS(de_bigS),
		    .de_hex(de_hex),
		    .de_cr(de_cr),
			 .de_littlep(de_littlep),
			 .de_bigR(de_bigR),
		    .charData(bu_rx_data),
		    .charDataValid(bu_rx_data_rdy));

   scctrl scctrl (
		  .sccEncrypt(sccEncrypt),
		  .sccEldByte(sccEldByte),
		  .sccEmsBitsLd(sccEmsBitsLd),
		  .sccElsBitsLd(sccElsBitsLd),
		  .sccEmsBitsSl(sccEmsBitsSl),
		  .sccDecrypt(sccDecrypt),
		  .sccDnibble1En(sccDnibble1En),
		  .sccDnibble2En(sccDnibble2En),
		  .sccLdKey(sccLdKey),
		  .sccLdLFSR(sccLdLFSR),
		  .L4_tx_data_rdy(L4_tx_data_rdy),
		  .L4_PrintBuf(L4_PrintBuf),
		  .sccPtKey(sccPtKey),
		  .sccPt_key(sccPt_key),
		  .sccPtLFSR(sccPtLFSR),
		  .scdCharIsValid(scdCharIsValid),
		  .bu_rx_data_rdy(bu_rx_data_rdy),
		  .dAscii(de_validAscii),
		  .dBigD(de_bigD),
		  .dBigE(de_bigE),
		  .dBigL(de_bigL),
		  .dBigP(de_bigP),
		  .dBigS(de_bigS),
		  .dHex(de_hex),
		  .dCr(de_cr),
		  .dLittlep(de_littlep),
		  .dBigR(de_bigR),
		  .rst(rst),
		  .clk(clk));

	assign L4_led = 5'b00000;			// assign FPGA LED


endmodule // Lab4_140L

//
// scdp - stream cipher datapath
// refer to lab instructions for a block diagram
//
//
module scdp (
	     output [7:0] L4_tx_data,   //     data to be sent to uartTxBuf
	     output wire  scdCharIsValid, // encrypt byte is a valid character

	     input [7:0]  bu_rx_data,   // data from the uart
	     input 	  bu_rx_data_rdy, // data from the uart is valid this cycle 
	     input 	  sccEncrypt,   //     control signal indicating we are in encrypt mode
	     input 	  sccEldByte,   // control signal to load bu_rx_data into encrypt register
	     input 	  sccEmsBitsLd, // load the most significant 4 bits of encrypted data
	                                // as an 8-bit ascii hex number
	     input 	  sccElsBitsLd, // load the least significant 4 bits of encrypted data
	                                  // as an 8-bit ascii hex number
	     input 	  sccEmsBitsSl, // select the hex number for the most significant 4 bits
	                                  // of encrypted data to L4_tx_data
	     input 	  sccDecrypt, // we are in decrypt mode
	     input 	  sccDnibble1En, // load 4 bits of encrypted data (most significant)
	     input 	  sccDnibble2En, // load 4 bits of encryhptd data (least significant)
	     
	     input [7:0]  sccLdKey,      // load 4-bit (nibble) of the key
	     input 	  sccLdLFSR,     // load the LFSR from the key
		  
		  input [7:0] sccPtKey,
		  input		sccPt_key,
		  input		sccPtLFSR,

	     input 	  rst,
	     input 	  clk
	     );


   
   wire [3:0] 		  binVal;               // conversion of ascii hex to bin

   wire [7:0] 		  psrByte;  // pseudo random byte


   asciiHex2Bin a2b (.val(binVal), .inVal(bu_rx_data));
   
   
   //
   // decrypt datapath
   //
   wire [7:0] 		  byteToDecrypt;        // byte we are decrypting
   regrce #(4) u0 (byteToDecrypt[7:4], binVal, sccDnibble1En, rst, clk);
   regrce #(4) u1 (byteToDecrypt[3:0], binVal, sccDnibble2En, rst, clk);
    
   wire [7:0] 		  e2dData;
   wire [7:0] 		  pCharDecrypt;   // printable char
   assign e2dData = byteToDecrypt ^ psrByte;
   printable pinst0 (.pChar(pCharDecrypt), .pValid(), .inByte(e2dData));

   

   //
   // encrypt data path
   //
   wire [7:0]		  byteToEncrypt;        // byte we are encrypting
   regrce #(8) u2 (byteToEncrypt, bu_rx_data, sccEldByte, rst, clk);
   
   printable pinst1 (.pChar(), .pValid(scdCharIsValid), .inByte(bu_rx_data));

   
   wire [7:0] 		  d2eData;
   
   assign d2eData = byteToEncrypt ^ psrByte;
   

   //
   // we are encrypting, convert 
   // to two hex digits
   // will send the digits over two cycles.  MS nibble followed by LS nibble.
   //
   wire [7:0] 		  msBitsD, msBits;
   wire [7:0] 		  lsBitsD, lsBits;
   
   bin2AsciiHex b2a0 (msBits, d2eData[7:4]);
   bin2AsciiHex b2a1 (lsBits, d2eData[3:0]);
   
   regrce #(8) msBitsi (msBitsD, msBits, sccEmsBitsLd, rst, clk);
   regrce #(8) lsBitsi (lsBitsD, lsBits, sccElsBitsLd, rst, clk);

   wire [7:0] 		  key0;// bits 7-0
   wire [7:0] 		  key1;// bits 15-8
   wire [7:0] 		  key2;// bits 23-16
   wire [7:0] 		  key3;// bits 31-24

   wire [3:0] 		 binValD;   // bu_rx_data delayed
   wire                  binVal_ValidD;
   regrce #(4) rxdataD (.q(binValD),
			.d(binVal),
			.ce(1'b1), .rst(rst), .clk(clk));
   regrce #(1) rddataDV (.q(binVal_ValidD), .d(bu_rx_data_rdy),
			 .ce(1'b1), .rst(rst), .clk(clk));
   
   regrce #(4) k0l (.q(key0[3:0]), .d(binValD),
		    .ce(sccLdKey[0] & binVal_ValidD),
		   .rst(rst), .clk(clk));
   regrce #(4) k0h (.q(key0[7:4]), .d(binValD),
		    .ce(sccLdKey[1] & binVal_ValidD),
		   .rst(rst), .clk(clk));
   regrce #(4) k1l (.q(key1[3:0]), .d(binValD),
		    .ce(sccLdKey[2] & binVal_ValidD),
		   .rst(rst), .clk(clk));
   regrce #(4) k1h (.q(key1[7:4]), .d(binValD),
		    .ce(sccLdKey[3] & binVal_ValidD),
		   .rst(rst), .clk(clk));
   regrce #(4) k2l (.q(key2[3:0]), .d(binValD),
		    .ce(sccLdKey[4] & binVal_ValidD),
		   .rst(rst), .clk(clk));
   regrce #(4) k2h (.q(key2[7:4]), .d(binValD),
		    .ce(sccLdKey[5] & binVal_ValidD),
		   .rst(rst), .clk(clk));
   regrce #(4) k3l (.q(key3[3:0]), .d(binValD),
		    .ce(sccLdKey[6] & binVal_ValidD),
		   .rst(rst), .clk(clk));
   regrce #(4) k3h (.q(key3[7:4]), .d(binValD),
		    .ce(sccLdKey[7] & binVal_ValidD),
		   .rst(rst), .clk(clk));


   wire [31:0] 		 lfsrVal;
   lfsr lfsrInst (
		  .lfsrVal(lfsrVal),
		  .psrByte(psrByte),
		  .ldVal({key3, key2, key1, key0}),
		  .ldLFSR(sccLdLFSR),
		  .step(sccDnibble2En | sccEldByte),
		  .rst(rst),
		  .clk(clk)
		  );
   

	// enhancement
	wire [7:0] hex;
	bin2AsciiHex b2A(
		
		.asciiHex(hex),
		
		.hx(sccPtKey[7] ? (sccPt_key ? key3[7:4] : lfsrVal[31:28]) : 
				(sccPtKey[6] ? (sccPt_key ? key3[3:0] : lfsrVal[27:24]) : 
				(sccPtKey[5] ? (sccPt_key ? key2[7:4] : lfsrVal[23:20]) : 
				(sccPtKey[4] ? (sccPt_key ? key2[3:0] : lfsrVal[19:16]) : 
				(sccPtKey[3] ? (sccPt_key ? key1[7:4] : lfsrVal[15:12]) : 
				(sccPtKey[2] ? (sccPt_key ? key1[3:0] : lfsrVal[11:8]) : 
				(sccPtKey[1] ? (sccPt_key ? key0[7:4] : lfsrVal[7:4]) : 
				(sccPtKey[0] ? (sccPt_key ? key0[3:0] : lfsrVal[3:0]) : 4'b0000))))))))
		);
      
   assign L4_tx_data = sccEncrypt ?
		       (sccEmsBitsSl ? msBitsD : lsBitsD ) :
		       (sccPt_key | sccPtLFSR ? hex : pCharDecrypt);
   
endmodule // scdp


//
// scctrl - stream cipher control
//
module scctrl (
			output		sccEncrypt,		// control signal indicating we are in encrypt mode
			output		sccEldByte,		// control signal to load bu_rx_data into encrypt register
			output 		sccEmsBitsLd,
			output 	 	sccElsBitsLd,
			output 	 	sccEmsBitsSl,
			output 	 	sccDecrypt,
			output 	 	sccDnibble1En,
			output 	 	sccDnibble2En,
			output [7:0] sccLdKey,
			output 	 	sccLdLFSR,
			output 	 	L4_tx_data_rdy,
			output 	 	L4_PrintBuf,
			
			output [7:0] sccPtKey,		//
			output	 	sccPt_key,		//
			output	 	sccPtLFSR,		//
			
			input wire scdCharIsValid,	// encrypt byte is a valid character
			input			bu_rx_data_rdy,
			
			input			dEsc,
			input			dAscii,
			input			dBigD,
			input			dBigE,
			input			dBigL,
			input			dBigP,
			input			dBigS,
			input			dHex,
			input			dCr,
			
			input			dLittlep,		//
			input			dBigR,			//
			
			input 		rst,
			input			clk
			);
   
	reg[3:0] cState;
	reg[3:0] nState;
	localparam START=0, ENC=1, LDBYTE=2, LDEBIT=3, LDMSBIT=4, LDLSBIT=5, DEC=6, NB1=7, NB2=8,
					LDD=9, LDK=10, PTK=11, PTLFSR=12, PTB=13;
	
	reg[3:0] cKey;
	reg[3:0] nKey;
	localparam K3h=0, K3l=1, K2h=2, K2l=3, K1h=4, K1l=5, K0h=6, K0l=7, KOF=8;
	
	always @(posedge clk) begin
		if(rst) begin			
			cState <= START;
			cKey <= K3h;
		end
      else begin
			cState <= nState;
			cKey <= nKey;
		end
	end
	
	reg enc, ldbyte, msbld, lsbld, msbsl, dec, nbl1, nbl2, ldlfsr, p_key, p_lfsr, drdy, pbuf;
	reg[7:0] ldkey, pkey;
	
	always @(*) begin
		nState = cState;
		nKey = cKey;
		enc = 0;
		ldbyte = 0;
		msbld = 0;
		lsbld = 0;
		msbsl = 0;
		dec = 0;
		nbl1 = 0;
		nbl2 = 0;
		ldlfsr = 0;
		p_key = 0;
		p_lfsr = 0;
		drdy = 0;
		pbuf = 0;
		ldkey = 8'b00000000;
		pkey = 8'b00000000;
		case(cState)
			START: begin
				nKey = K3h;
				ldlfsr = 0;
				p_key = 0;
				p_lfsr = 0;
				if(dBigE) begin
					nState = ENC;
					enc = 1;
				end
				else if(dBigD) begin
					nState = DEC;
					dec = 1;
				end
				else if(dBigL)
					nState = LDK;
				else if(dBigP) begin
					nState = PTK;
					p_key = 1;
				end
				else if(dLittlep) begin
					nState = PTLFSR;
					p_lfsr = 1;
				end
				else if(dBigR) begin
					ldlfsr = 1;
					pbuf = 1;
				end
				else
					nState = START;
			end
			ENC: begin
				enc = 1;
				if(dCr) begin
					nState = START;
					enc = 0;
					pbuf = 1;
				end
				else if(scdCharIsValid & bu_rx_data_rdy) begin
					nState = LDBYTE;
					ldbyte = 1;
				end
				else
					nState = ENC;
			end
			LDBYTE: begin
				nState = LDEBIT;
				enc = 1;
				msbld = 1;
				lsbld = 1;
			end
			LDEBIT: begin
				nState = LDMSBIT;
				enc = 1;
				msbsl = 1;
				drdy = 1;
			end
			LDMSBIT: begin
				nState = LDLSBIT;
				enc = 1;
				msbsl = 0;
				drdy = 1;
			end
			LDLSBIT: begin
				nState = ENC;
				enc = 1;
			end
			DEC: begin
				dec = 1;
				if(dCr) begin
					nState = START;
					dec = 0;
					pbuf = 1;
				end
				else if(scdCharIsValid & bu_rx_data_rdy) begin
					nState = NB1;
					nbl1 = 1;
				end
				else
					nState = DEC;
			end
			NB1: begin
				dec = 1;
				if(dCr) begin
					nState = START;
					dec = 0;
					pbuf = 1;
				end
				else if(scdCharIsValid & bu_rx_data_rdy) begin
					nState = NB2;
					nbl2 = 1;
				end
				else
					nState = NB1;
			end
			NB2: begin
				nState = DEC;
				dec = 1;
				drdy = 1;
			end
			LDK: begin
				ldkey = 8'b00000000;
				if(dCr) begin
					nState = START;
					ldlfsr = 1;
					pbuf = 1;
				end
				else if(dHex & bu_rx_data_rdy) begin
					case(cKey)
						K3h: begin
							nKey = K3l;
							ldkey = 8'b10000000;
						end
						K3l: begin
							nKey = K2h;
							ldkey = 8'b01000000;
						end
						K2h: begin
							nKey = K2l;
							ldkey = 8'b00100000;
						end
						K2l: begin
							nKey = K1h;
							ldkey = 8'b00010000;
						end
						K1h: begin
							nKey = K1l;
							ldkey = 8'b00001000;
						end
						K1l: begin
							nKey = K0h;
							ldkey = 8'b00000100;
						end
						K0h: begin
							nKey = K0l;
							ldkey = 8'b00000010;
						end
						K0l: begin
							nKey = KOF;
							ldkey = 8'b00000001;
						end
						KOF: begin
							nKey = KOF;
							ldkey = 8'b00000000;
						end
						default: begin
							nKey = K3h;
							ldkey = 8'b00000000;
						end
					endcase
					nState = LDK;
				end
			end
			PTK: begin
				p_key = 1;
				pkey = 8'b00000000;
				case(cKey)
					K3h: begin
						nState = PTK;
						nKey = K3l;
						p_key = 1;
						drdy = 1;
						pkey = 8'b10000000;
					end
					K3l: begin
						nState = PTK;
						nKey = K2h;
						p_key = 1;
						drdy = 1;
						pkey = 8'b01000000;
					end
					K2h: begin
						nState = PTK;
						nKey = K2l;
						p_key = 1;
						drdy = 1;
						pkey = 8'b00100000;
					end
					K2l: begin
						nState = PTK;
						nKey = K1h;
						p_key = 1;
						drdy = 1;
						pkey = 8'b00010000;
					end
					K1h: begin
						nState = PTK;
						nKey = K1l;
						p_key = 1;
						drdy = 1;
						pkey = 8'b00001000;
					end
					K1l: begin
						nState = PTK;
						nKey = K0h;
						p_key = 1;
						drdy = 1;
						pkey = 8'b00000100;
					end
					K0h: begin
						nState = PTK;
						nKey = K0l;
						p_key = 1;
						drdy = 1;
						pkey = 8'b00000010;
					end
					K0l: begin
						nState = PTB;
						nKey = K3h;
						p_key = 1;
						drdy = 1;
						pkey = 8'b00000001;
					end
					default: begin
						nKey = K3h;
						p_key = 1;
						drdy = 0;
						pkey = 8'b00000000;
					end
				endcase
			end
			PTLFSR: begin
				p_lfsr = 1;
				pkey = 8'b00000000;
				case(cKey)
					K3h: begin
						nState = PTLFSR;
						nKey = K3l;
						p_lfsr = 1;
						drdy = 1;
						pkey = 8'b10000000;
					end
					K3l: begin
						nState = PTLFSR;
						nKey = K2h;
						p_lfsr = 1;
						drdy = 1;
						pkey = 8'b01000000;
					end
					K2h: begin
						nState = PTLFSR;
						nKey = K2l;
						p_lfsr = 1;
						drdy = 1;
						pkey = 8'b00100000;
					end
					K2l: begin
						nState = PTLFSR;
						nKey = K1h;
						p_lfsr = 1;
						drdy = 1;
						pkey = 8'b00010000;
					end
					K1h: begin
						nState = PTLFSR;
						nKey = K1l;
						p_lfsr = 1;
						drdy = 1;
						pkey = 8'b00001000;
					end
					K1l: begin
						nState = PTLFSR;
						nKey = K0h;
						p_lfsr = 1;
						drdy = 1;
						pkey = 8'b00000100;
					end
					K0h: begin
						nState = PTLFSR;
						nKey = K0l;
						p_lfsr = 1;
						drdy = 1;
						pkey = 8'b00000010;
					end
					K0l: begin
						nState = PTB;
						nKey = K3h;
						p_lfsr = 1;
						drdy = 1;
						pkey = 8'b00000001;
					end
					default: begin
						nKey = K3h;
						p_lfsr = 1;
						drdy = 0;
						pkey = 8'b00000000;
					end
				endcase
			end
			PTB: begin
				nState = START;
				pbuf = 1;
				p_lfsr = 0;
				drdy = 0;
			end
			default: begin
				nState = cState;
				nKey = K3h;
				enc = 0;
				ldbyte = 0;
				msbld = 0;
				lsbld = 0;
				msbsl = 0;
				dec = 0;
				nbl1 = 0;
				nbl2 = 0;
				ldlfsr = 0;
				p_key = 0;
				p_lfsr = 0;
				drdy = 0;
				pbuf = 0;
				pkey = 8'b00000000;
			end
		endcase
	end
	
	regrce #(8) ldk(
		.q(sccLdKey),
		.d(ldkey),
		.ce(1),
		.rst(rst),
		.clk(clk));
	
	assign sccEncrypt = enc;
	assign sccEldByte = ldbyte;
	assign sccEmsBitsLd = msbld;
	assign sccElsBitsLd = lsbld;
	assign sccEmsBitsSl = msbsl;
	assign sccDecrypt = dec;
	assign sccDnibble1En = nbl1;
	assign sccDnibble2En = nbl2;
	assign sccLdLFSR = ldlfsr;
	assign L4_tx_data_rdy = drdy;
	assign L4_PrintBuf = pbuf;
	assign sccPtKey = pkey;
	assign sccPt_key = p_key;
	assign sccPtLFSR = p_lfsr;


	
endmodule // scctrl

