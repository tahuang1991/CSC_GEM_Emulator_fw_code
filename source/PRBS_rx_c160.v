`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    13:13:55 07/12/2011 
// Design Name: 
// Module Name:    PRBS_rx 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module PRBS_rx_c160(
    input REC_CLK,
	 input CE1,
	 input CE3,
	 input RST,
    input [47:0] RCV_DATA,
	 output STRT_MTCH,
    output reg VALID,
    output reg MATCH
    );
	 
parameter start_pattern = 48'hFFFFFF000000;

reg [47:0] pipe1,pipe2;
reg [47:0] expct;
wire [23:0] lfsr;
reg [23:0] lfsr_a;
reg [23:0] lfsr_b;
reg start_pat;
reg valid_ena;
reg vld1,vld2;
wire ce80;

assign ce80 = CE1 | CE3;
assign STRT_MTCH = (RCV_DATA == start_pattern);

	always @(posedge REC_CLK or posedge RST) begin
	   if(RST) 
			valid_ena <= 1'b0;
		else
			if(start_pat)
				valid_ena <= 1'b1;
			else
				valid_ena <= valid_ena;
   end
	always @(posedge REC_CLK) begin
	   if(ce80) begin
			start_pat <= STRT_MTCH;
			vld1 <= !start_pat && valid_ena;
		end
   end

//
// Linear Feedback Shift Register
// [24,23,22,17] Fibonacci Implementation
//
   lfsr_R24_c160 #(.init_fill(24'h83B62E))
   rx_lfsr1(
	   .CLK(REC_CLK),
		.CE(ce80),
		.RST(start_pat),
		.LFSR(lfsr));

	
	always @(posedge REC_CLK) begin
	   if(CE3) lfsr_a <= lfsr;
	end
	
	always @(posedge REC_CLK) begin
		if(CE1) lfsr_b <= lfsr;
	end
	
	always @(posedge REC_CLK) begin
		if(CE3) begin
         pipe1 <= RCV_DATA;
		   pipe2 <= pipe1;
			expct <= {lfsr_a,lfsr_b};
			MATCH <= (pipe2 == expct);
			vld2 <= vld1;
			VALID <= vld2;
		end
	end
endmodule
