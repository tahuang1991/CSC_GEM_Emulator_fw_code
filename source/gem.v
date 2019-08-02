`timescale 1ns / 1ps
//------------------------------------------------------------------------------------------------------------------
//  2019-08-01, found gem clusters from 56bit data to be transmitted to OTMB
//-------------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------------
// Bus Widths
//------------------------------------------------------------------------------------------------------------------


module gem (
    // Clock
    input          clock,             // 40MHz TMB system clock

    // Global reset
    //input          global_reset,      // 1=Reset everything
    input  [55:0]  gemdata,

    // Status
    //output          gem_sump,        // Unused signals wot must be connected


    output [13:0] cluster0, // cluster0 in GEM coordinates (0-1535)
    output [13:0] cluster1, // cluster1 in GEM coordinates (0-1535)
    output [13:0] cluster2, // cluster2 in GEM coordinates (0-1535)
    output [13:0] cluster3, // cluster3 in GEM coordinates (0-1535)

    //output [ 2:0] cluster0_roll,// eta partition number 
    //output [ 2:0] cluster1_roll,
    //output [ 2:0] cluster2_roll,
    //output [ 2:0] cluster3_roll,

    //output [ 7:0] cluster0_pad, // pad number in one roll, no VFAT boundary
    //output [ 7:0] cluster1_pad,
    //output [ 7:0] cluster2_pad,
    //output [ 7:0] cluster3_pad,

    output vpf0, // cluster0 valid flag
    output vpf1, // cluster1 valid flag
    output vpf2, // cluster2 valid flag
    output vpf3,  // cluster3 valid flag

    output reg [23:0] active_feb_list

);

// Raw hits RAM parameters
parameter RAM_DEPTH = 2048; // Storage bx depth
parameter RAM_ADRB  = 11;   // Address width=log2(ram_depth)
parameter RAM_WIDTH = 14;   // Data width

// Gem Count
parameter IGEM     = 0;
parameter MXCLST   = 4;
parameter CLSTBITS = 14;
parameter MXPAD    = 192;
parameter MXROLL   = 8;
parameter MXFEB    = 24;

//-------------------------------------------------------------------------------------------------------------------
// State machine power-up reset + global reset
//-------------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------------
// Decompose packed GEM data format
//------------------------------------------------------------------------------------------------------------------

  wire [13:0] cluster     [3:0];
  wire [11:0] adr         [3:0];
  wire [ 2:0] cnt         [3:0];
  wire [ 0:0] vpf         [3:0];
  //add vfat, roll,pad in a roll
  reg [ 4:0] cluster_feb [3:0];       
  //reg [ 2:0] cluster_roll[3:0];       
  //reg [ 7:0] cluster_pad [3:0];       


  //wire [MXPAD-1:0] gemhit_roll0 = 0;
  //wire [MXPAD-1:0] gemhit_roll1 = 0;
  //wire [MXPAD-1:0] gemhit_roll2 = 0;
  //wire [MXPAD-1:0] gemhit_roll3 = 0;
  //wire [MXPAD-1:0] gemhit_roll4 = 0;
  //wire [MXPAD-1:0] gemhit_roll5 = 0;
  //wire [MXPAD-1:0] gemhit_roll6 = 0;
  //wire [MXPAD-1:0] gemhit_roll7 = 0;
  //wire [MXPAD*MXROLL-1:0] hits = 0;
  //wire [MXPAD*MXROLL-1:0] hits_cluster [3:0];


  genvar iclst;
  generate
  for (iclst=0; iclst<4; iclst=iclst+1) begin: cluster_assignment
    assign cluster     [iclst] = gemdata[(iclst+1)*14-1 : iclst*14];
    assign adr         [iclst] = cluster[iclst][10:0];
    assign cnt         [iclst] = cluster[iclst][13:11];
    assign vpf         [iclst] = ~(adr[iclst][10:9]==2'b11);
    //assign hits_cluster[iclst] = (vpf[iclst] ? {(1536-cnt[iclst]-1-adr[iclst])*{0},(cnt[iclst]+1)*{1}, adr[iclst]*{0}} : 0);
  always @(*) begin
  case (adr[iclst][10:6]) // adr[10:6] is the "natural" vfatid
    5'd0:    cluster_feb[iclst] = 5'd0;
    5'd1:    cluster_feb[iclst] = 5'd8;
    5'd2:    cluster_feb[iclst] = 5'd16;
    5'd3:    cluster_feb[iclst] = 5'd1;
    5'd4:    cluster_feb[iclst] = 5'd9;
    5'd5:    cluster_feb[iclst] = 5'd17;
    5'd6:    cluster_feb[iclst] = 5'd2;
    5'd7:    cluster_feb[iclst] = 5'd10;
    5'd8:    cluster_feb[iclst] = 5'd18;
    5'd9:    cluster_feb[iclst] = 5'd3;
    5'd10:   cluster_feb[iclst] = 5'd11;
    5'd11:   cluster_feb[iclst] = 5'd19;
    5'd12:   cluster_feb[iclst] = 5'd4;
    5'd13:   cluster_feb[iclst] = 5'd12;
    5'd14:   cluster_feb[iclst] = 5'd20;
    5'd15:   cluster_feb[iclst] = 5'd5;
    5'd16:   cluster_feb[iclst] = 5'd13;
    5'd17:   cluster_feb[iclst] = 5'd21;
    5'd18:   cluster_feb[iclst] = 5'd6;
    5'd19:   cluster_feb[iclst] = 5'd14;
    5'd20:   cluster_feb[iclst] = 5'd22;
    5'd21:   cluster_feb[iclst] = 5'd7;
    5'd22:   cluster_feb[iclst] = 5'd15;
    5'd23:   cluster_feb[iclst] = 5'd23;
    default: cluster_feb[iclst] = 5'd24;
  endcase
  end


   //always @(*) begin
   //case (adr[iclst][10:6]) // adr[10:6] is the "natural" vfatid
   //  5'd0:    begin  cluster_feb[iclst] <= 5'd0 ;  cluster_roll[iclst] <= 3'd0; cluster_pad[iclst] <= {2'b00,adr[iclst][5:0]}; end
   //  5'd1:    begin  cluster_feb[iclst] <= 5'd8 ;  cluster_roll[iclst] <= 3'd0; cluster_pad[iclst] <= {2'b01,adr[iclst][5:0]}; end
   //  5'd2:    begin  cluster_feb[iclst] <= 5'd16;  cluster_roll[iclst] <= 3'd0; cluster_pad[iclst] <= {2'b10,adr[iclst][5:0]}; end
   //  5'd3:    begin  cluster_feb[iclst] <= 5'd1 ;  cluster_roll[iclst] <= 3'd1; cluster_pad[iclst] <= {2'b00,adr[iclst][5:0]}; end
   //  5'd4:    begin  cluster_feb[iclst] <= 5'd9 ;  cluster_roll[iclst] <= 3'd1; cluster_pad[iclst] <= {2'b01,adr[iclst][5:0]}; end
   //  5'd5:    begin  cluster_feb[iclst] <= 5'd17;  cluster_roll[iclst] <= 3'd1; cluster_pad[iclst] <= {2'b10,adr[iclst][5:0]}; end
   //  5'd6:    begin  cluster_feb[iclst] <= 5'd2 ;  cluster_roll[iclst] <= 3'd2; cluster_pad[iclst] <= {2'b00,adr[iclst][5:0]}; end
   //  5'd7:    begin  cluster_feb[iclst] <= 5'd10;  cluster_roll[iclst] <= 3'd2; cluster_pad[iclst] <= {2'b01,adr[iclst][5:0]}; end
   //  5'd8:    begin  cluster_feb[iclst] <= 5'd18;  cluster_roll[iclst] <= 3'd2; cluster_pad[iclst] <= {2'b10,adr[iclst][5:0]}; end
   //  5'd9:    begin  cluster_feb[iclst] <= 5'd3 ;  cluster_roll[iclst] <= 3'd3; cluster_pad[iclst] <= {2'b00,adr[iclst][5:0]}; end
   //  5'd10:   begin  cluster_feb[iclst] <= 5'd11;  cluster_roll[iclst] <= 3'd3; cluster_pad[iclst] <= {2'b01,adr[iclst][5:0]}; end
   //  5'd11:   begin  cluster_feb[iclst] <= 5'd19;  cluster_roll[iclst] <= 3'd3; cluster_pad[iclst] <= {2'b10,adr[iclst][5:0]}; end
   //  5'd12:   begin  cluster_feb[iclst] <= 5'd4 ;  cluster_roll[iclst] <= 3'd4; cluster_pad[iclst] <= {2'b00,adr[iclst][5:0]}; end
   //  5'd13:   begin  cluster_feb[iclst] <= 5'd12;  cluster_roll[iclst] <= 3'd4; cluster_pad[iclst] <= {2'b01,adr[iclst][5:0]}; end
   //  5'd14:   begin  cluster_feb[iclst] <= 5'd20;  cluster_roll[iclst] <= 3'd4; cluster_pad[iclst] <= {2'b10,adr[iclst][5:0]}; end
   //  5'd15:   begin  cluster_feb[iclst] <= 5'd5 ;  cluster_roll[iclst] <= 3'd5; cluster_pad[iclst] <= {2'b00,adr[iclst][5:0]}; end
   //  5'd16:   begin  cluster_feb[iclst] <= 5'd13;  cluster_roll[iclst] <= 3'd5; cluster_pad[iclst] <= {2'b01,adr[iclst][5:0]}; end
   //  5'd17:   begin  cluster_feb[iclst] <= 5'd21;  cluster_roll[iclst] <= 3'd5; cluster_pad[iclst] <= {2'b10,adr[iclst][5:0]}; end
   //  5'd18:   begin  cluster_feb[iclst] <= 5'd6 ;  cluster_roll[iclst] <= 3'd6; cluster_pad[iclst] <= {2'b00,adr[iclst][5:0]}; end
   //  5'd19:   begin  cluster_feb[iclst] <= 5'd14;  cluster_roll[iclst] <= 3'd6; cluster_pad[iclst] <= {2'b01,adr[iclst][5:0]}; end
   //  5'd20:   begin  cluster_feb[iclst] <= 5'd22;  cluster_roll[iclst] <= 3'd6; cluster_pad[iclst] <= {2'b10,adr[iclst][5:0]}; end
   //  5'd21:   begin  cluster_feb[iclst] <= 5'd7 ;  cluster_roll[iclst] <= 3'd7; cluster_pad[iclst] <= {2'b00,adr[iclst][5:0]}; end
   //  5'd22:   begin  cluster_feb[iclst] <= 5'd15;  cluster_roll[iclst] <= 3'd7; cluster_pad[iclst] <= {2'b01,adr[iclst][5:0]}; end
   //  5'd23:   begin  cluster_feb[iclst] <= 5'd23;  cluster_roll[iclst] <= 3'd7; cluster_pad[iclst] <= {2'b10,adr[iclst][5:0]}; end
   //  default: begin  cluster_feb[iclst] <= 5'd24;  cluster_roll[iclst] <= 3'd0; cluster_pad[iclst] <=                   8'd192; end  //invalid case
   //endcase
   //end

  end
  endgenerate



  wire gem_has_data = (vpf[0]|vpf[1]|vpf[2]|vpf[3]);


  // form a 24 bit list of active febs, based on presence of cluster in gemA
  genvar ifeb;
  generate
  for (ifeb=0; ifeb<MXFEB; ifeb=ifeb+1)     begin:   active_feb_loop
    always @(posedge clock) begin
    active_feb_list [ifeb] <= (cluster_feb[0]==ifeb && vpf[0]) |
                              (cluster_feb[1]==ifeb && vpf[1]) |
                              (cluster_feb[2]==ifeb && vpf[2]) |
                              (cluster_feb[3]==ifeb && vpf[3]);
    end
  end
  endgenerate

//----------------------------------------------------------------------------------------------------------------------
// outputs
//----------------------------------------------------------------------------------------------------------------------


  assign vpf0 = vpf[0];
  assign vpf1 = vpf[1];
  assign vpf2 = vpf[2];
  assign vpf3 = vpf[3];

  assign  cluster0      = cluster [0];
  assign  cluster1      = cluster [1];
  assign  cluster2      = cluster [2];
  assign  cluster3      = cluster [3];
  //assign  cluster0_feb  = cluster_feb[0];
  //assign  cluster1_feb  = cluster_feb[1];
  //assign  cluster2_feb  = cluster_feb[2];
  //assign  cluster3_feb  = cluster_feb[3];
  //assign  cluster0_roll = cluster_roll[0];
  //assign  cluster1_roll = cluster_roll[1];
  //assign  cluster2_roll = cluster_roll[2];
  //assign  cluster3_roll = cluster_roll[3];
  //assign  cluster0_pad  = cluster_pad[0];
  //assign  cluster1_pad  = cluster_pad[1];
  //assign  cluster2_pad  = cluster_pad[2];
  //assign  cluster3_pad  = cluster_pad[3];

//-------------------------------------------------------------------------------------------------------------------
endmodule
//-------------------------------------------------------------------------------------------------------------------
