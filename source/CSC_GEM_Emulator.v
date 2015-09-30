`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// *** JRG, firmware for Fiber Tx Bench Board, used for GEM-CSC pattern tests **
//  Only for use on 2010 prototype boards; compile it for the XC6VLX195T FPGA
//
// Create Date:    3/4/15
// Design Name:
// Module Name:    CSC_GEM_emulator
//
// Added BRAMs b00 to bff and associated GbE command functions (F3F3/F7F7 for Read/Write)
// Need to add more command functions:
//   FEFE to send one event... the first data word NUM can specify how many BX to send for the "event"
//    - this comand will not "zero" the Read pointer
//   F0F0 to rewind the Read pointer back to address zero
//    - F7F7 will zero the Read pointer too (F3F3 has a separate read controller for GTX, does not to reset our pointer)
//   FDFD to dump all contents of BRAMs... perhaps a "C" data code sent with this can initiate a continuous repeat loop?
//    - receipt of any "non-FD" function will immediately halt the repeat loop
//   F3F3 will switch the RDCLK source to 62.5 MHz
//    - for all other cases the default RDCLK is 40 MHz (negedge?)
//
// Need to control...
//          .ADDRARDADDR ({1'b1, tx_adr[10:2], 6'h3f}),  // 16 bit RDADDR, but only 14:6 are used w/ECC.  1/3f?
//          .ENARDEN ((cmd_code == 16'hf3f3) & (bk_adr == v) & gtx_ready),  // RDEN    ***REPLACE the f3f3 with a 1-bit signal***
//          .CLKARDCLK   (gbe_txclk2 or ck40 with f3f3 switch), //  RDCLK     ***uses sel_rdclk***
//
//   Make Read Pointer "rd_ptr"
//    - this can be a 16-bit counter that runs at ck40 speed, but only 9 Adr bits are needed for a BRAM (so 7 bits can access extra BRAMs if needed)
//    - the lowest 9 bits must be MUXed with tx_adr[10:2] to form the read address bus (rd_ptr is default, but F3F3 enables tx_adr)
//    - similarly the RDCLK source default is ck40, but F3F3 enables ck625
//   rd_ptr counter behavior:
//    - FDFD will enable the counter and connect DOUT to the GTX TxDat bus for 512 clocks, then afterwards set the GTX TxDat bus to zero
//         -- however, if "C" is set it will enable them until cancelled by a new command
//    - FEFE will enable the counter and connect DOUT to the GTX TxDat bus for NUM clocks, then afterwards set the GTX TxDat bus to zero
//    - rd_ptr is reset by F0F0 or F7F7
//
//
//
//
//  about clocks:
//   QPLL links with tmb_clock05p (no delay), which is disabled when step4=Hi and prevents QPLL lock (bad)
//     -- this is dumb; QPLL should use tmb_clock0 (io600 on B31)? Only disabled by mez_clock_en from TMB/VME boot reg. b12
//     -- tmb_clock1 comes in on K24, has delay AND it stops when step4=Hi
//   qpll_ck40 comes directly from QPLL40, derived from tmb_clk05p that gets stopped bt step4; goes nowhere
//   lhc_ck NOW comes directly from tmb_clock0; slwclk is this divided by 25
//     -- "locked" indicates lock OK from slwclk div25 function
//   ck40 comes from gtx_Tx based on QPLL160
//     -- "lock40" indicates lock OK, and it depends upon "ck160_locked"
//     -- has random phase relative to lhc_ck
//   rx_clk is gtx_Rx 160 MHz reconstructed receiver USR clock
//     -- "ck160_locked" indicates gtx_Tx lock OK, but not Rx!
//     -- it's not clear that rx_clk has any bufg or lock implemented at all!
//   ck125 is not really used, just for Rx FIFO_RD CLK now
//   Later replace GbE comparator readback with VME function?
//
//
//  MTP Fiber Mapping to Signal Name, FPGA GTX channels, Diff. Pin Numbers, verilog name
//    1:   Tx0-Rx0   GTX3-GTX0    AK1/AK2=Q0 - AP5/AP6=Q0     txp/n[0]-rxp/n[0]
//    2:   Tx1-Rx1   GTX4-GTX1    AH1/AH2=Q1 - AM5/AM6=Q0     txp/n[1]-rxp/n[1]
//    3:   Tx2-Rx2   GTX7-GTX2    AB1/AB2=Q1 - AL3/AL4=Q0     txp/n[2]-rxp/n[2]
//    4:   Tx3-Rx3   GTX8-GTX3    Y1/Y2=Q2   - AJ3/AJ4=Q0     txp/n[3]-rxp/n[3]
//    9:   Tx4-Rx8   GTX9-GTX8    V1/V2=Q2   - AA3/AA4=Q2     txp/n[4]-rxp/n[4]
//    10:  Tx5-Rx9   GTX10-GTX9   T1/T2=Q2   - W3/W4=Q2       txp/n[5]-rxn/p[5] --RX swapped!
//    11:  Tx6-Rx10  GTX11-GTX10  P1/P2=Q2   - U3/U4=Q2       txp/n[6]-rxn/p[6] --RX swapped!
//    12:  Tx7-Rx11  GTX2-GTX11   AM1/AM2=Q0 - R3/R4=Q2       txp/n[7]-rxp/n[7] <<-- used this @OSU
//    5:    Rx4        GTX4           AG3/AG4
//    6:    Rx5        GTX5           AF5/AF6
//    7:    Rx6        GTX6           AE3/AE4
//    8:    Rx7        GTX7           AC3/AC4
//      QPLL 160 refclk comes into pins AB6/AB5, Quad 113 refclk 1 (Q1,C1)
//   Q0=quad_112, Q1=quad_113, Q2=quad_114, Q3=quad_115, Q4=quad_116
//
//
//////////////////////////////////////////////////////////////////////////////////
module CSC_GEM_Emulator (
    input 	      ck125n, ck125p,
    input 	      ck160n, ck160p,
    input 	      lhc_ckn, lhc_ckp,
    input  [8:7]  sw,
    input 	      tmb_clock0, pb, vme_cmd10,
    input 	      qpll_lock,
    input  [8:1]  gempad_in,  // new inputs from CFEB Emulator board from "test LED" pins on connector
    input  [3:0]  vstat, // +1.5V TMB, Vcore RPC (driven via loopback), +3.3V TMB, +5V TMB
    input  [28:1] alct_rx,
    input 	      jtag_usr0_tdo, gp_io4, rpc_dsn, rpc_smbrx,
    input 	      prom_d3, prom_d7, jtag_fpga3, sda0, tmb_sn, t_crit,
    input [50:0]  _ccb_rx,  // add 42-47
    input [5:0]   dmb_rx,
    input [15:8]  dmb_i1tx,  // yes, input for loopback tests, only use on MODIFIED TMBs!
    input [31:24] dmb_i2tx,  // yes, input for loopback tests, only use on MODIFIED TMBs!
    input [43:38] dmb_i3tx,  // yes, input for loopback tests, only use on MODIFIED TMBs!
    input [9:0]   rpc_i1rx,
    input [37:26] rpc_i2rx,

    // JG, new ports for GbE:
    input 	      ck_gben, ck_gbep,
    input 	      gbe_rxn, gbe_rxp,
    input 	      gbe_fok,
    output 	      gbe_txn, gbe_txp,
    output 	      f_sclk,
    // ^^^^^^^^^^^^^
    output [3:0]      sel_usr,
    output [3:1]      jtag_usr, // [0] is Input, see above
    output [17:5]     alct_txa,
    output [23:19]    alct_txb,
    output [7:0]      dmb_tx,
    output [25:10]    rpc_orx,
    output [23:16]    dmb_o1tx,
    output [37:32]    dmb_o2tx,
    output [48:44]    dmb_o3tx,
    output            smb_clk, alct_loop, alct_txoe, alct_clock_en, alct_rxoe, smb_data,
    output            gtl_loop, dmb_loop, rpc_loop, ccb_status_oe, _dmb_oe, // set normal safe bidir bus modes
    output [26:0]    _ccb_tx,  // add 20-26
    output           _hard_reset_tmb_fpga, _hard_reset_alct_fpga,
    output 	          rst_qpll, fcs,
    output [3:0]      rpc_tx, // [3] is alct_tx29 on TMB!
    output [6:0]      vme_reply,
    output [4:0]      step, // step4 enables STEP mode for 3:0 = cfeb, rpc, dmb, alct.  step4 Low makes all free-running clocks.
    output reg [7:0]  led_low,
    output reg [15:8] led_hi,
    output reg [10:9] test_led,
    input 	          t12_fault, r12_fok,
  //input  [7:1]      rxp,rxn,
    output [7:0]      txp,txn,
    output 	      t12_rst, t12_sclk, r12_sclk
    )/* synthesis syn_useioff = 1 */;

    // snap12 GTX signals
    wire        synced_snapt;
    wire        snap_clk2, ck160_locked, ck160_rst;
    wire [7:0]  tx_begin, tx_fc;
    reg  [7:0]  ferr_f;
    reg  [15:0] err_count;
    reg  [7:0]  time_r_snap;
    reg  [7:0]  time_snap;
    wire stopped, locked, lock40, dmbfifo_step1ck;

    parameter SEEDBASE = 64'h8731c6ef4a5b0d29;
    parameter SEEDSTEP = 16'hc01d;
    reg [5:0]   ireg;  // not used
    reg [22:0]  free_count;
    reg 	      free_tc, free_tc_r;
    reg 	      slow_tc, slow_tc_r;
    reg [7:0]   time_count;
    reg [7:0]   time_40_r;
    reg [7:0]   time_40_rr;
  //wire        gtx_ready;
    wire [13:0] test_in;  // not used?
    reg 	      l_lock40, ck160_locklost, qpll_lock_lost;
    reg 	      bc0_r, bc0_rr, bc0_r3, bc0_led;
  //wire        reset, gtx_reset;
    wire        ext_rst, force_err, bc0, stat0;  // stat[3:0] =  cfebemul_in[2,3,1,4] on Emul board!
    wire        ck125, ck160, lhc_ck, lhc_clk, qpll_ck40, slwclk;   // ext 125 usr, QPLL160, ccb_ck40, QPLL40, ccb_ck40/25=1.6MHz
    wire        lhc_ck0, lhc_ck90, lhc_ck180, lhc_ck270, lhcckfbout, lhcckfbout_buf, lhc_clk90;
    wire        zero = 1'b0;
    wire        one = 1'b1;
    wire [1:0]  zero2 = 0;
    wire [31:0] zero32 = 0;
    wire [12:0] low = 0;
  //wire [3:0]  ignore;  // outputs from GTX we don't care about

    wire        ck40, ck40buf, rdclk;
    reg 	      sel_rdclk;

    // Add-ons for Ben dCFEB testing:
    wire    tx_clk_out, tx_clk;
    wire    rx_strt, rx_valid, rx_match, rx_fc;
    wire [3:0]  word;
    wire [3:1]  nz;
    wire [47:0] comp_dat, comp_dout;
    reg  [47:0] comp_dat_r, comp_dat_2r, comp_dat_3r, comp_dat_4r, comp_dat_5r;
    reg  [3:0]  itriad;
    reg 	       save_triad, comp_dav_r, send_triad;
    wire        push_fifo, no_comp_dav, comp_overflow;
    reg  [7:0]  triad_word, triad_word_r;
    reg  [7:0]  triad_word_l1, triad_word_l2, triad_word_l3, triad_word_l4, triad_word_l5, triad_word_l6;
    reg  [63:0] triad_counter;
    reg 	       clct_pattern;  //change clct_pattern when pushing the reset button
    reg fiber_reset;
    //  set itriad=9  if ( !sw[8] && itriad==0 && |comp_dat_r > 0 )  can we pick a few specific bits?  TRIAD FORMAT!
    //      -- comp_dat is 48 bits, takes 3-step logic....pipeline some before _r and finish before _2r
    //  48-bit wide, 5-deep pipeline; word0 is EN.  comp_dat_r, comp_dat_2r, .... comp_dat_5r
    //      -- goes to 48-bit FIFO with  PUSH=word0&itriad>0; runs on rx_clk (160 MHz)
    //      -- PUSH controls itriad-1 countdown
    //  FIFO output is 16-bits wide(?) for GbE tx_dat...NOT ALLOWED!   runs on clk
    //      -- !empty triggers GbE data dump of three triads (3 * 48 bits each...54 bytes total)


    // 27 registers for inputs from DMB Loopback, 47 for RPCloop, plus 5 (7?) for SLOWloop
    reg  [46:0] shft_seq, rnd_word;  // slow seq .1 sec shift, loads to flag on pb_pulse  shft_seq = 47'h00000001;
    reg 	       hold_bit;    // debounced !pb signal, held until button release
    reg 	       debounced_bit;    // sets one pulse for 200 ns  (5 MHz clock)
    reg 	       pb_pulse;  //  <- sw7 & !pb, clears on pb high after 2nd seq. shift (debounce), lasts a while!
    reg 	       err_wait;   // pb_pulse & tc & !wait -> load rnd_word, set wait.  !pb_pulse & tc & wait -> clear wait

    // JG, new declarations for GbE and BRAM use:
    wire  gbe_refck; // GTXE1 ref clk, used for MGTref and DoubleReset clock
    wire  gbe_tx_outclk; // out from Tx PLL
    wire  gbe_txclk2, gbe_txclk2_buf;   // drives logic for Tx, the "internal" choice for USR clock
    wire  txoutclk_mmcm_lk;

    parameter BRAM_LIM = 12'h007; // was bff for 256 Bram's: bff, or 8 -> b07.
    parameter GBE_LIM = 16'h080b;  // WORDcount limit. allow extra bytes for MAC preamble, addr's...
    reg [15:0] pkt_lim; // BytecountLimit/2. Plus extra 22 bytes for MAC preamble etc.
    reg [15:0] counter;  // ^^^ 1st data word @ counter == 12; need 11 extra word counts, 22 bytes.
    reg [10:0] tx_adr, rx_adr, rx_adr_r;
    reg [15:0] gbe_rxcount;
    reg [7:0]  pkt_id;
    reg [15:0] cmd_code, prev_cmd[3:0];
    reg [11:0] bk_adr;
    reg   tx_resetdone_r, rx_resetdone_r, rx_resetdone_r2, rx_resetdone_r3, pkt_send;
    wire  gtx_ready, tx_resetdone, rx_resetdone;
    reg [7:0]   ovfl_packet;
    reg 	rx_timeout;
    reg 	good_rx_cmd;
    wire reset, gtx_reset;
    wire ckgbe;
    wire [3:0]  ignore;  // outputs from GTX we don't care about
    wire        rxpll_lk;
    wire [2:0]  rx_bufstat;
    wire [1:0]  rx_comma;
    wire [1:0]  rx_disp;  // other signals for GTX
    wire [1:0]  rx_lostsync;
    reg  [1:0]  gbe_kout, tx_kout, tx_kout_r;
    reg 	comma_align;
    wire [5:0]  rxer;
    reg  [15:0] gbe_txdat, tx_out, tx_out_r;
    wire [15:0] gbe_rxdat;
    reg  [15:0] l_gbe_rxdat, ll_gbe_rxdat;
    reg 	l_rxdv, l_kchar, ll_kchar, mac_seek, mac_sync, mac_ack, counter_send;
    reg  [1:0]  kchar_r;
    reg  [1:0]  sync_state, data_state;
    wire        rxdv;

    reg  [15:0] data_bram, data_bram_r;  // these are the BRAM MUX bus & register
    wire [63:0] data_oram[BRAM_LIM:12'h000];
    reg  [63:0] data_iram;
    wire [BRAM_LIM:12'h000] sbiterr_oram,  dbiterr_oram;
    reg         cycle4, cmd_f7f7;  // use this to toggle bram WRITE every 4th GbE word during cmd=f7f7

    reg         crc_en, crc_rst;
    wire [31:0] crc_out;
    reg  [15:0] byte_count;
    reg l_lock125, ck125_locklost;
    reg [7:0] time_r,time_r_i;


    assign f_sclk   = 1'b1;  // to Finisar GbE Transceiver
    // JG, end new declarations ^^^^^^


    assign bc0 = 1'b0;
    wire [15:0] geminfo[31:0];

    assign geminfo[0][15:0]=16'd0;
    assign geminfo[1][15:0]=16'd7;
    assign geminfo[2][15:0]=16'd13;
    assign geminfo[3][15:0]=16'd18;
    assign geminfo[4][15:0]=16'd22;
    assign geminfo[5][15:0]=16'd27;
    assign geminfo[6][15:0]=16'd32;
    assign geminfo[7][15:0]=16'd37;
    assign geminfo[8][15:0]=16'd42;
    assign geminfo[9][15:0]=16'd47;
    assign geminfo[10][15:0]=16'd52;
    assign geminfo[11][15:0]=16'd57;
    assign geminfo[12][15:0]=16'd62;
    assign geminfo[13][15:0]=16'd67;
    assign geminfo[14][15:0]=16'd72;
    assign geminfo[15][15:0]=16'd77;
    assign geminfo[16][15:0]=16'd82;
    assign geminfo[17][15:0]=16'd87;
    assign geminfo[18][15:0]=16'd92;
    assign geminfo[19][15:0]=16'd97;
    assign geminfo[20][15:0]=16'd102;
    assign geminfo[21][15:0]=16'd107;
    assign geminfo[22][15:0]=16'd112;
    assign geminfo[23][15:0]=16'd116;
    assign geminfo[24][15:0]=16'd121;
    assign geminfo[25][15:0]=16'd0;
    assign geminfo[26][15:0]=16'd0;
    assign geminfo[27][15:0]=16'd0;
    assign geminfo[28][15:0]=16'd0;
    assign geminfo[29][15:0]=16'd0;
    assign geminfo[30][15:0]=16'd0;
    assign geminfo[31][15:0]=16'd0;

    reg [15:0] geminfo_r;
    reg [4:0] gempad_r;
    reg [1:0] q;

    assign test_in[11:7] = alct_rx[11:7];
    assign test_in[13] = alct_rx[13];
    assign test_in[12] = alct_rx[19];
    assign test_in[0] = sda0;
    assign test_in[1] = tmb_sn;
    assign test_in[2] = t_crit;
    assign test_in[3] = jtag_fpga3;
    assign test_in[4] = prom_d3;
    assign test_in[5] = prom_d7;
    assign test_in[6] = alct_rx[23];

    // clct_status: temp for testing,  _ccb_tx[8:0] was 9'h0aa, now toggle with push-button
    assign _ccb_tx[8] = pb;
    assign _ccb_tx[7] = !pb;
    assign _ccb_tx[6] = pb;
    assign _ccb_tx[5] = !pb;
    assign _ccb_tx[4] = pb;
    assign _ccb_tx[3] = !pb;
    assign _ccb_tx[2] = pb;
    assign _ccb_tx[1] = !pb;
    assign _ccb_tx[0] = pb;


    assign vme_reply[0] = 1'b1;   // OE_b, low true
    assign vme_reply[1] = 1'b1;   // DIR
    assign vme_reply[2] = 1'b0;   // DTACK, inverted on TMB board
    assign vme_reply[3] = ~vme_cmd10; // IACKOUT = IACKIN, inverted on TMB board?
    assign vme_reply[4] = 1'b0;   // BERR, inverted on TMB board
    assign vme_reply[5] = 1'b0;   // IRQ, inverted on TMB board
    assign vme_reply[6] = 1'b0;   // vme_ready, High enables 0-5 above?
    //   assign _gtl_oe = 1'b0;   // JRG: always set LOW (SEU danger, short to GND --PU) Now float for Mezz 2012 compatibility
    assign gtl_loop = 1'b0;  // JRG: always set LOW (SEU danger, make OPEN --PD)   **Now INPUT for Mezz 2012!**
    assign dmb_loop = 1'b1;  // JRG: set HIGH for SPECIAL TMB ONLY! LOW for normal CMS operation (SEU danger, make OPEN --PD)
    assign rpc_loop = 1'b1;  // JRG: set HIGH for Produtcion Test, LOW for normal CMS operation (SEU safe --PD)
    assign ccb_status_oe = 1'b1;  // JRG:  set HIGH for Produtcion Test and for normal CMS operation (SEU danger, make OPEN --PU)
    assign _dmb_oe = 1'b0;
    assign _hard_reset_tmb_fpga = 1'b1;

    assign low=0;
    assign zero=0;
    assign zero2=2'b0;
    assign zero32=0;
    assign one=1'b1;

    assign fcs  = 1'b1;  // drive high for Mezz 2012 compatibility, useful on the bench
    assign t12_rst  = 1'b1;  // low-true signal for Snap12 Transmitter
    assign t12_sclk = 1'b1;  // to Snap12 Transmitter
    assign r12_sclk = 1'b1;  // to Snap12 Receiver
    assign rst_qpll = 1'b1;  // reset is low-true, but in ExtControl mode (sw6 On) it becomes fSelect[5]
    //                                                 and autoRestart (sw5) becomes fSelect[4]
    //        note that sw1-4 are fSelect[0:3] but they only function in ExtControl mode (sw6 On),
    //        and fSelect sets the VCXO center frequency (because automatic calibration is Off)
    //   wire qpll_lock; // probably random in ExtControl mode (sw6 On)

    //   reg [11:0]  l1a_count; // count L1accepts
    wire  ccb_cken; // global clock signal to use for ccb_rx[0] "clock"
    reg [11:0]  ccbrxzero_count; // count toggles on ccb_rx0
    reg [12:0]  pulse_count; // count triggers, 10 low-true sources ANDed together
    reg [11:0]  pulses_fired;
    reg [11:0]  in_pulse_r;
    wire [11:0] in_pulse;
    wire      rst_errcnt;
    reg       trigger, rst_errcnt_r;

    // - pulse counter (pulse is CE); send pulses & read back count via status LEDs (11 ccb_rx)
    //      > 1 reset signal (L1Reset=ccb_reserved4, to clear) and triggered by 11 different pulses:
    //          crate BC0, L1A, tmb_soft_reset=tmb_reserved1, clct/alct_external_trigger, dmb_cfeb_calibrate[2:0],
    // 	    adb_pulse sync/async, alct_hard_reset,
    //         - verify that all single CMD & DATA bits work reliably (avoid CMD/DATA = 0C,0D,0E,0F; 10,11,12,13; 40,41,42,43)
    //         - need to check LEDs at least one time too, to verify status bus works

    // unless noted otherwise, these pulses are only 25ns long and count just one time:
    assign rst_errcnt   = !_ccb_rx[29]; // TMB_SoftRst (tmb_res1), CCB base+6c or 6a
    assign in_pulse[0]  = !_ccb_rx[11]; // crate BC0, CCB base+52
    assign in_pulse[1]  = !_ccb_rx[12]; // L1A, CCB base+54
    assign in_pulse[2]  = !_ccb_rx[29]; // TMB_SoftRst (tmb_res1), CCB base+6c or 6a
    assign in_pulse[3]  = !_ccb_rx[32]; // clct_ext_trig, CCB base+86
    assign in_pulse[4]  = !_ccb_rx[33]; // alct_ext_trig, CCB base+88
    assign in_pulse[5]  = !_ccb_rx[39]; // dmb_cfeb_calib0, CCB base+8a
    assign in_pulse[6]  = !_ccb_rx[40]; // dmb_cfeb_calib1, CCB base+8c
    assign in_pulse[7]  = !_ccb_rx[41]; // dmb_cfeb_calib2, CCB base+8e
    assign in_pulse[8]  = !_ccb_rx[27]; // alct_hard_reset_ccb, CCB base+66:  500 ns long pulse
    assign in_pulse[9]  = !_ccb_rx[30]; // alct_adb_pulse_sync, CCB base+82:  500 ns long pulse
    assign in_pulse[10] = !_ccb_rx[31]; // alct_adb_pulse_async, CCB base+84: long pulse
    assign in_pulse[11] = !_ccb_rx[28]; // tmb_reserved0, Not Really a Pulse!  I will think of a better way to test this.
    //  -- right now we access this with CCB base+2a (CSRB6, write a 1 then a 0 to bit2): we get a random count each time

    assign _ccb_tx[26:22] = _ccb_rx[47:43];  // returns DMB_Reserved_Out[4:0] from CCB back to the CCB on TMB_Reserved_In[4:0]
    // CCB can Write DMB_Reserved_Out[4:0] (to all TMBs & DMBs) on base+2a (CSRB6, bits 14:10).  ccb_rx[47-43]
    // CCB can Read TMB_Reserved_In[4:0] from TMB on base+34 (CSRB11, bits 7:3).  ccb_tx[26-22]
    //   --> For this test, TMB will return the value we set on DMB_Reserved_Out, back to the CCB via TMB_Reserved_In.

    //  For these I am not sure how best to test them, still thinking...
    // ccb_rx0 should be a clock... count it to see it toggle, and send some bits via CCB.
    // CCB can Write TMB_Reserved_Out[2:0] (to all TMBs) on base+2a (CSRB6, bits 9:7).  ccb_rx[38-36]
    // CCB can Write TMB_Reserved0 (to all TMBs) on base+2a (CSRB6, bit2).   ccb_rx28
    // CCB can Write CCB_Reserved[3:2] (to all TMBs) on base+2a (CSRB6, bits 1:0).  ccb_rx[25-24]
    //   -- ccb_reserved(1:0) are for QPLL & TTC status... just try to read them back via CCB.  ccb_rx[23-22]
    // TMB_L1A_Release/Request can generate L1As at the CCB... try this and count the L1As via CCB.  ccb_tx[21-20]

    wire [2:0]  tmb_res_out;
    wire [5:0]  ccb_unused;
    wire [7:0]  ccb_data;
    wire [7:0]  ccb_cmd;
    wire        ccb_cmdstrb, ccb_datstrb;
    wire        _alct_adb_pulse_async, _alct_adb_pulse_sync;
    reg [7:0]  ccb_data_r;
    reg [7:0]  ccb_cmd_r, last_cmd;
    reg        ccb_cmdstrb_r, ccb_datstrb_r;
    reg        alct_cfg_out, tmb_cfg_out, results_hold, late_load_done;
    reg [4:0]  get_bit_ptr;
    reg [19:0] results_r;
    reg [1:0]  ccb_rsv_r;
    reg [1:0]  tmb_l1a_relreq;
    assign ccb_data[7:0] = ~_ccb_rx[21:14];
    assign ccb_datstrb = !_ccb_rx[13];
    assign ccb_cmd[7:0] = { (~_ccb_rx[7:2]), (!_ccb_rx[8]), (!_ccb_rx[9])};
    assign ccb_cmdstrb = !_ccb_rx[10];
    assign tmb_res_out[2:0] = ~_ccb_rx[38:36];
    assign ccb_unused[4:0] = ~_ccb_rx[28:24];
    assign ccb_unused[5]   = ~_ccb_rx[42];

    // These tx bits are outputs for TMB_L1A_Release/Request. Create pulses using CCB_Reserved[3:2] from CCB:
    //   when TMB L1A Release & Request go out, CCB should send an L1A, check!  May need to enable that on CCB.
    assign _ccb_tx[21:20] = tmb_l1a_relreq[1:0];  // must be 25ns pulses from TMB to CCB
    // CCB can Write CCB_Reserved[3:2] (to all TMBs) on base+2a (CSRB6, bits 1:0).  ccb_rx[25-24]

    // take these to TMB-FP LEDs for observation:
    assign _alct_adb_pulse_async = !_ccb_rx[31];
    assign _alct_adb_pulse_sync = !_ccb_rx[30];
    assign ccb_ttcrx_rdy = !_ccb_rx[22];
    assign ccb_qpll_lck = !_ccb_rx[23];
    //   assign mpc_in0 = !_ccb_rx[34];
    //   assign mpc_in1 = !_ccb_rx[35];

    reg [38:0] init_dmbloop[26:0];
    reg en_loopbacks, en_loopbacks_r, en_loopbacks_rr, lhc_tog_err;
    reg lhc_tog, en_lhcloop, en_lhcloop_r, lhcloop_tog, lhcloop_tog_r;
    reg [26:0] in_dmbloop, lout_dmbloop, llout_dmbloop;
    wire [26:0] rawin_dmbloop, out_dmbloop;
    wire [27:0] good_dmbloop, err_dmbloop;
    wire [15:0] count_dmbloop[26:0];    // Not Used: 27 16-bit wide elements, error counts for DMB Loop signals
    wire [15:0] count_ratloop[46:0];    // Not Used: 47 16-bit wide elements, error counts for RAT Loop signals
    wire [15:0] count_slowloop[4:0];    // Not Used: 5 16-bit wide elements, error counts for SLOW Loop signals
    reg [11:0]  dmbloop_errcnt, dmbloop1_stat, dmbloop2_stat, dmbloop3_stat;
    reg [31:0]  loop_count;
    reg 	       en_cabletests, en_fibertests, en_cabletests_r, en_fibertests_r;
    reg [38:0]  init_ratloop[46:0];
    reg [38:0]  init_slowloop[4:0];
    reg [4:0]   in_slowloop, lout_slowloop, llout_slowloop;
    reg [46:0]  in_ratloop, lout_ratloop, loutpos_ratloop, llout_ratloop;
    wire [4:0]  out_slowloop, good_slowloop, err_slowloop;
    wire [46:0] rawin_ratloop, out_ratloop, good_ratloop, err_ratloop;
    reg [11:0]  ratloop_errcnt, ratloop1_stat, ratloop2_stat, ratloop3_stat, ratloop4_stat;
    reg [26:0]  slowloop_count;
    reg [11:0]  slowloop_errcnt, hzloop_count;
    reg [7:0]   slowloop_err, slowloop_stat;
    reg [3:0]   selusr;
    integer     i;

    //   DMB Loop:  27 pairs + one clock.    lout_dmbloop goes out, then in_dmbloop comes back from DMB-Loopback
    // dmbfifo_step1ck -> dmb_rx0   This is CCB clock, 40 MHz. But NOT a CLK pin!  Div2 via Flop and send to bufg?
    //    Note: step4 selects STEP mode for 3:0 = cfeb, rpc, dmb, alct.  step4 Low makes all free-running clocks.
    // 0 dmb_tx33 -> dmb_rx1
    // 1 dmb_tx47 -> dmb_rx2
    // 2 dmb_tx48 -> dmb_rx3
    // 3 dmb_tx45 -> dmb_rx4
    // 4 dmb_tx46 -> dmb_rx5
    // 5 dmb_tx0  -> dmb_tx12
    // 6 dmb_tx1  -> dmb_tx13
    // 7 dmb_tx2  -> dmb_tx14
    // 8 dmb_tx3  -> dmb_tx15
    // 9 dmb_tx4  -> dmb_tx8
    //10 dmb_tx5  -> dmb_tx9
    //11 dmb_tx6  -> dmb_tx10
    //12 dmb_tx7  -> dmb_tx11
    //13 dmb_tx16 -> dmb_tx28
    //14 dmb_tx17 -> dmb_tx29
    //15 dmb_tx18 -> dmb_tx30
    //16 dmb_tx19 -> dmb_tx31
    //17 dmb_tx20 -> dmb_tx24
    //18 dmb_tx21 -> dmb_tx25
    //19 dmb_tx22 -> dmb_tx26
    //20 dmb_tx23 -> dmb_tx27
    //21 dmb_tx32 -> dmb_tx42
    //22 dmb_tx34 -> dmb_tx38
    //23 dmb_tx35 -> dmb_tx39
    //24 dmb_tx36 -> dmb_tx40
    //25 dmb_tx37 -> dmb_tx41
    //26 dmb_tx44 -> dmb_tx43
    assign dmbfifo_step1ck      = dmb_rx       [0];
    assign dmb_o2tx        [33] = lout_dmbloop [0];
    assign rawin_dmbloop   [0]  = dmb_rx       [1];
    assign dmb_o3tx        [47] = lout_dmbloop [1];
    assign rawin_dmbloop   [1]  = dmb_rx       [2];
    assign dmb_o3tx        [48] = lout_dmbloop [2];
    assign rawin_dmbloop   [2]  = dmb_rx       [3];
    assign dmb_o3tx        [45] = lout_dmbloop [3];
    assign rawin_dmbloop   [3]  = dmb_rx       [4];
    assign dmb_o3tx        [46] = lout_dmbloop [4];
    assign rawin_dmbloop   [4]  = dmb_rx       [5];

    assign dmb_tx[7:0] = lout_dmbloop[12:5];
    assign rawin_dmbloop[12:5] = {dmb_i1tx[11:8],dmb_i1tx[15:12]};
    assign dmb_o1tx[23:16] = lout_dmbloop[20:13];
    assign rawin_dmbloop[20:13] = {dmb_i2tx[27:24],dmb_i2tx[31:28]};
    assign dmb_o2tx[32] = lout_dmbloop[21];
    assign dmb_o2tx[37:34] = lout_dmbloop[25:22];
    assign dmb_o3tx[44] = lout_dmbloop[26];
    assign rawin_dmbloop[26:21] = {dmb_i3tx[43],dmb_i3tx[41:38],dmb_i3tx[42]};
    //  RAT_Loopback signals:
    assign rpc_tx[3:0] = lout_ratloop[3:0]; // [3] is actually alct_tx29
    assign smb_clk = lout_ratloop[4];
    assign alct_txa[17:5] = lout_ratloop[17:5];
    assign _hard_reset_alct_fpga = lout_ratloop[18]; // this goes to alct_tx18
    assign alct_txb[23:19] = lout_ratloop[23:19];
    assign alct_loop = lout_ratloop[24];
    assign alct_txoe = lout_ratloop[25];
    assign alct_clock_en = lout_ratloop[26];
    assign step[0] = lout_ratloop[27];  // requires step[4] to be set or this goes nowhere!
    assign alct_rxoe = lout_ratloop[28];
    assign smb_data = lout_ratloop[29];
    assign step[2] = lout_ratloop[30];  // requires step[4] to be set or this goes nowhere!
    assign rpc_orx[25:10]  = lout_ratloop[46:31];

    assign rawin_ratloop[0] = rpc_i2rx[30]; // rpc_sync="rpc_tx"0
    assign rawin_ratloop[1] = rpc_i2rx[29]; // ERROR! rpc_posneg="rpc_tx"1
    assign rawin_ratloop[2] = rpc_i2rx[31]; // rpc_loop_tm="rpc_tx"2
    assign rawin_ratloop[3] = alct_rx[4];   // rpc_free0="rpc_tx"3=alct_tx29   bubble
    assign rawin_ratloop[4] = rpc_i2rx[27]; // smb_clk
    assign rawin_ratloop[5] = alct_rx[25];  // alct_txa5-17
    assign rawin_ratloop[6] = alct_rx[26];  //  bubble
    assign rawin_ratloop[7] = alct_rx[24];  //  2*bubble
    assign rawin_ratloop[8] = alct_rx[21];
    assign rawin_ratloop[9] = alct_rx[22];  // ERROR alct_tx9   2*bubble
    assign rawin_ratloop[10] = alct_rx[23]; // ERROR alct_tx10
    assign rawin_ratloop[11] = alct_rx[20]; //  2*bubble
    assign rawin_ratloop[12] = alct_rx[18]; // ERROR alct_tx12   bubble
    assign rawin_ratloop[13] = alct_rx[19]; // ERROR! alct_tx13  bubble
    assign rawin_ratloop[14] = alct_rx[17];
    assign rawin_ratloop[15] = alct_rx[14]; //  2*bubble
    assign rawin_ratloop[16] = alct_rx[15]; // ERROR alct_tx16
    assign rawin_ratloop[17] = alct_rx[2];  //  bubble
    assign rawin_ratloop[18] = alct_rx[1];  // typo-error, fixed  // alct_tx18 = hard_reset_alct_fpga    bubble
    assign rawin_ratloop[19] = alct_rx[16]; // ERROR   // alct_txb19-23   2*bubble
    assign rawin_ratloop[20] = alct_rx[12]; // ERROR alct_tx20  bubble
    assign rawin_ratloop[21] = alct_rx[9];  // ERROR alct_tx21  bubble
    assign rawin_ratloop[22] = alct_rx[10]; //  bubble
    assign rawin_ratloop[23] = alct_rx[8];  //  2*bubble
    assign rawin_ratloop[24] = alct_rx[5];  // Fixed error?  // alct_loop
    assign rawin_ratloop[25] = alct_rx[7];  // alct_txoe
    assign rawin_ratloop[26] = alct_rx[11]; // alct_clock_en
    assign rawin_ratloop[27] = alct_rx[13]; // step0
    assign rawin_ratloop[28] = alct_rx[6];  // alct_rxoe   bubble
    assign rawin_ratloop[29] = alct_rx[3];  // ERROR   // smb_data   bubble
    assign rawin_ratloop[30] = rpc_i2rx[28];// step2   bubble
    assign rawin_ratloop[31] = rpc_i1rx[7]; // rpc_rx10-25
    assign rawin_ratloop[32] = rpc_i1rx[8];
    assign rawin_ratloop[33] = rpc_i1rx[9];
    assign rawin_ratloop[34] = rpc_i1rx[6];
    assign rawin_ratloop[35] = rpc_i1rx[3];
    assign rawin_ratloop[36] = rpc_i1rx[4];
    assign rawin_ratloop[37] = rpc_i1rx[5];
    assign rawin_ratloop[38] = rpc_i1rx[0];
    assign rawin_ratloop[39] = rpc_i1rx[1];
    assign rawin_ratloop[40] = rpc_i1rx[2];
    assign rawin_ratloop[41] = alct_rx[28]; // 20   bubble
    assign rawin_ratloop[42] = rpc_i2rx[32];
    assign rawin_ratloop[43] = rpc_i2rx[33];
    assign rawin_ratloop[44] = rpc_i2rx[34];
    assign rawin_ratloop[45] = alct_rx[27];
    assign rawin_ratloop[46] = rpc_i2rx[26];

    assign good_dmbloop[27] = !(|err_dmbloop);

    //   assign step[4] = en_loopbacks; // ~sw[8];  // step4 Low makes free clocks from TMB; Hi allows logic signals for clocks
    assign step[4] = (en_loopbacks & (~en_fibertests)); // ~sw[8];  // step4 Low makes free clocks from TMB; Hi allows logic signals for clocks
    assign step[3] = 1'b0;   // this is cfeb step signal
    //   assign step[2] = 1'b0;   // this is rpc step signal
    //   assign step[1] = lhc_clk;  // this is dmb step signal... now uses ODDR below.
    //   assign step[0] = 1'b0;   // this is alct step signal
    assign sel_usr[3:0] = selusr[3:0];  // if en_fibertests selusr <= 4'b1101
    assign jtag_usr[3] = lout_slowloop[2]; // vstat[2] = slowloop2 is SLOW!!  Only ~3 HZ max from power-sense chip
    assign jtag_usr[1] = lout_slowloop[0];
    assign jtag_usr[2] = lout_slowloop[1];
    always @(*)
    begin
        if (!en_fibertests) selusr = 4'b1101;    // rpc_jtag active, 3 bits only, includes ~1 Hz Vstat2 test
        else selusr = {2'b00,lout_slowloop[4],lout_slowloop[3]}; // alct_jtag active, 5 bits under test
    end

    ODDR #(.DDR_CLK_EDGE("OPPOSITE_EDGE"), .INIT(1'b0), .SRTYPE("ASYNC")) DMB_FIFO_CLK (.Q(step[1]), .C(lhc_clk), .CE(1'b1), .D1(1'b1), .D2(1'b0), .R(1'b0), .S(1'b0));  // make step[1] an image of lhc_clk, as it goes out and loops back as dmbfifo_step1ck



    initial begin
        gempad_r        = 0;
        geminfo_r       = 0;
        q               = 0;
        l_lock40        = 0;
        ck160_locklost  = 0;
        time_40_r       = 8'h00;
        time_40_rr      = 8'h00;
        free_tc_r       = 1'b0;
        slow_tc_r       = 1'b0;
        loop_count      = 0;
        en_loopbacks    = 1'b0;
        en_loopbacks_r  = 0;
        en_loopbacks_rr = 0;
        en_cabletests_r = 0;
        en_cabletests   = 1'b0;
        en_fibertests_r = 0;
        en_fibertests   = 1'b0;
        llout_dmbloop   = 0;
        lout_dmbloop    = 0;
        in_dmbloop     <= 0;
        dmbloop_errcnt  = 0;
        dmbloop1_stat   = 0;
        dmbloop2_stat   = 0;
        dmbloop3_stat   = 0;
        lhc_tog         = 0;
        en_lhcloop      = 0;
        en_lhcloop_r    = 0;
        lhcloop_tog     = 0;
        lhcloop_tog_r   = 0;
        lout_ratloop    = 0;
        loutpos_ratloop = 0;
        llout_ratloop   = 0;
        in_ratloop      = 0;
        ratloop_errcnt  = 0;
        ratloop1_stat   = 0;
        ratloop2_stat   = 0;
        ratloop3_stat   = 0;
        ratloop4_stat   = 0;
        slowloop_count  = 0;
        slowloop_err    = 0;
        slowloop_stat   = 0;
        lout_slowloop   = 0;
        llout_slowloop  = 0;
        in_slowloop     = 0;
        slowloop_errcnt = 0;
        selusr          = 4'b1101;
        for (i= 0; i<27; i=i+1) begin
            init_dmbloop[i]  = 39'd15 + 11401017*i;
        end
        for (i=0; i<47; i=i+1) begin
            init_ratloop[i] = 39'd68 + 1301017*i;
        end
        for (i=0; i<5; i=i+1) begin
            init_slowloop[i] = 39'd89 + 11901017*i;
        end
        shft_seq      = 47'h000000000001;
        pb_pulse      = 0;
        triad_word    = 0;
        triad_word_r  = 0;
        triad_word_l1 = 0;
        triad_word_l2 = 0;
        triad_word_l3 = 0;
        triad_word_l4 = 0;
        triad_word_l5 = 0;
        triad_word_l6 = 0;
        triad_counter = 0;
        clct_pattern  = 0;
        ferr_f[7:0]   = 0;
        err_wait      = 0;
        rnd_word      = 0;
        debounced_bit = 0;
        hold_bit      = 0;
        bc0_led       = 0;
        bc0_r         = 1;
        bc0_rr        = 1;
        bc0_r3        = 1;
        // JG, some new items:
        l_rxdv          = 0;
        l_kchar         = 0;
        kchar_r         = 0;
        tx_resetdone_r  = 0;
        rx_resetdone_r  = 0;
        rx_resetdone_r2 = 0;
        rx_resetdone_r3 = 0;
        comma_align     = 0;
        gbe_kout        = 2'h1;
        gbe_txdat       = 16'h50bc;
        tx_out_r        = 16'h50bc;
        tx_kout_r       = 2'h1;
        sync_state      = 2'h0;
        data_state      = 2'h0;
        mac_seek        = 0;
        mac_sync        = 0;
        mac_ack         = 0;
        l_lock40        = 0;
        ck160_locklost  = 0;
        l_lock125       = 0;
        ck125_locklost  = 0;
        time_r          = 8'h00;
        pkt_lim         = 16'd64;
        gbe_rxcount     = 16'h0000;
        good_rx_cmd     = 1'b0;
        rx_timeout      = 1'b0;
        pkt_id          = 8'h0000;
    end // initial begin


    //----------------------------------------------------------------------------------------------------------------------
    // Clock Generation
    //----------------------------------------------------------------------------------------------------------------------


    // lhc clock from tmb baseboard
    BUFG lhcck(.I(tmb_clock0), .O(lhc_ck)); // only goes to mmcm now for 4-phase generation

    // Differential LHC clock from QPLL
    IBUFGDS #(.DIFF_TERM("TRUE"),.IOSTANDARD("LVDS_25"))  qpll40(.I(lhc_ckp) , .IB(lhc_ckn) , .O(qpll_ck40));
    IBUFGDS #(.DIFF_TERM("FALSE"),.IOSTANDARD("LVDS_25"))  clock125(.I(ck125p) , .IB(ck125n) , .O(ck125));

    // Differential 160 mhz
    IBUFDS_GTXE1  clock160(.I(ck160p) , .IB(ck160n) , .O(ck160), .ODIV2(), .CEB(zero));

    IBUFDS_GTXE1  clock_gbe(.I(ck_gbep) , .IB(ck_gben) , .O(ckgbe), .ODIV2(), .CEB(1'b0));
    BUFG gbe_refclock(.I(ckgbe), .O(gbe_refck));

    bufg_div8clk clk1p6(lhc_clk,!lhc_locked,slwclk,stopped,locked); // slwclk is now 5 MHz again (was 1.6 MHz with div25 and lhc_clk, then 5 MHz using ck125)

    BUFR ccbrx0_clock(.I(_ccb_rx[0]), .O(ccb_cken));


    // MMCM for 4-phase LHC clock
    MMCM_ADV
    #(.BANDWIDTH            ("OPTIMIZED"),
    .CLKOUT4_CASCADE      ("FALSE"),
    .CLOCK_HOLD           ("FALSE"),
    .COMPENSATION         ("ZHOLD"),
    .STARTUP_WAIT         ("FALSE"),
    .DIVCLK_DIVIDE        (1),
    .CLKFBOUT_MULT_F      (25.000),
    .CLKFBOUT_PHASE       (0.000),
    .CLKFBOUT_USE_FINE_PS ("FALSE"),
    .CLKOUT0_DIVIDE_F     (25.000),
    .CLKOUT0_PHASE        (0.000),
    .CLKOUT0_DUTY_CYCLE   (0.500),
    .CLKOUT0_USE_FINE_PS  ("FALSE"),
    .CLKOUT1_DIVIDE       (25),
    .CLKOUT1_PHASE        (90.000),
    .CLKOUT1_DUTY_CYCLE   (0.500),
    .CLKOUT1_USE_FINE_PS  ("FALSE"),
    .CLKOUT2_DIVIDE       (25),
    .CLKOUT2_PHASE        (180.000),
    .CLKOUT2_DUTY_CYCLE   (0.500),
    .CLKOUT2_USE_FINE_PS  ("FALSE"),
    .CLKOUT3_DIVIDE       (25),
    .CLKOUT3_PHASE        (270.000),
    .CLKOUT3_DUTY_CYCLE   (0.500),
    .CLKOUT3_USE_FINE_PS  ("FALSE"),
    .CLKIN1_PERIOD        (25.0),
    .REF_JITTER1          (0.010))
    mmcm_lhc4phase
    // Output clocks
    (.CLKFBOUT            (lhcckfbout),
    .CLKFBOUTB           (),
    .CLKOUT0             (lhc_ck0),
    .CLKOUT0B            (),
    .CLKOUT1             (lhc_ck90),
    .CLKOUT1B            (),
    .CLKOUT2             (lhc_ck180),
    .CLKOUT2B            (),
    .CLKOUT3             (lhc_ck270),
    .CLKOUT3B            (),
    .CLKOUT4             (),
    .CLKOUT5             (),
    .CLKOUT6             (),
    // Input clock control
    .CLKFBIN             (lhcckfbout_buf),
    .CLKIN1              (qpll_ck40),  // qpll_ck40=tmb_clock05p, lhc_ck=tmb_clock0
    .CLKIN2              (1'b0),
    // Tied to always select the primary input clock
    .CLKINSEL            (1'b1),
    // Ports for dynamic reconfiguration
    .DADDR               (7'h0),
    .DCLK                (1'b0),
    .DEN                 (1'b0),
    .DI                  (16'h0),
    .DO                  (),
        .DRDY                (),
        .DWE                 (1'b0),
        // Ports for dynamic phase shift
        .PSCLK               (1'b0),
        .PSEN                (1'b0),
        .PSINCDEC            (1'b0),
        .PSDONE              (),
        // Other control and status signals
        .LOCKED              (lhc_locked),
        .CLKINSTOPPED        (),
        .CLKFBSTOPPED        (),
        .PWRDWN              (1'b0),
    .RST                 (1'b0));
    // Output buffering
    //-----------------------------------
    BUFG lhcclkf_buf     ( .O (lhcckfbout_buf), .I (lhcckfbout));
    BUFG lhcclkout0_buf  ( .O (lhc_clk),        .I (lhc_ck0));
    BUFG lhcclkout90_buf ( .O (lhc_clk90),      .I (lhc_ck90));

/* old Tao settings:
assign gtx_ready = lock40 & ck160_locked & synced_snapr;
assign reset = !_ccb_rx[1] | ext_rst | (!sw[7] & !pb);  // JGhere, this ccb_rx signal can screw up a bench test...
assign gtx_reset = reset | !gtx_ready;
*/

assign gtx_ready = tx_resetdone_r & rx_resetdone_r & txoutclk_mmcm_lk & lock40 & ck160_locked;
assign reset     = (!sw[7] & !pb);
assign gtx_reset = reset | !gtx_ready;
assign rxdv      = ~(|rxer | rx_bufstat[2]) & gbe_fok; // idle is K28.5,D16.2                  = BC,50 in time order


// always @(posedge _ccb_rx[0] or posedge reset) begin
always @(posedge ccb_cken or posedge reset) begin
    if (reset) ccbrxzero_count <= 0;
    else       ccbrxzero_count <= ccbrxzero_count + 1'b1; // count toggles on ccb_rx0
end // always @ (posedge _ccb_rx[0] or posedge reset)

reg  tx_sel;
always @(posedge tx_clk or posedge gtx_reset) begin
    if (gtx_reset) tx_sel <= 1'b1;
    else           tx_sel <= ~tx_sel;
end


wire [8:0] rd_addr;
reg [15:0] rd_ptr=0;
reg [7:0]  cmd_code_r=0, cmd_code_rr=0, nbx_r=0, ibx=0;
reg 	 dump_enable=0,  dump_loop_i=0, event_enable=0, send_event_r=0;
reg 	 dump_enable_r=0, dump_enable_rr=0, dump_loop_r=0, event_enable_r=0, dump_done=0, event_done=0;
assign send_event = dump_enable_rr || (event_enable_r & !event_done);  // use this OR (f3f3 && (bk_adr == v)) to Read Enable the BRAMs

always @(posedge rdclk) begin
    //   always @(posedge ck40) begin  // try rdclk here instead, should be OK.
    fiber_reset    <= gtx_reset;
    send_event_r   <= send_event;
    dump_enable_r  <= dump_enable;
    dump_loop_r    <= dump_loop & dump_enable_r; // dump_loop is set in gbe cmd section
    event_enable_r <= event_enable;
    nbx_r          <= nbx;
    cmd_code_r     <= cmd_code[7:0];
    cmd_code_rr    <= cmd_code_r;

    if      (gtx_reset)      dump_done <= 1'b0;
    else if (dump_enable_rr) dump_done <= (&rd_ptr[8:1]);  // end dump when rd_ptr is 1FE or 1FF
    else                     dump_done <= 1'b0;

    dump_enable_rr <= (!dump_done | dump_loop_r) & dump_enable & dump_enable_r;

    if (gtx_reset)           event_done <= 1'b0;
    else if (event_enable_r) event_done <= (ibx==nbx_r);  // event_done is used in gbe cmd section
    else                     event_done <= 1'b0;

    if ( gtx_reset || ((cmd_code_r == 8'hf0)&&(cmd_code_rr == 8'hf0)) || ((cmd_code_r == 8'hf7)&&(cmd_code_rr == 8'hf7)) ) begin
        rd_ptr       <= 0;
        ibx          <= 0;
        send_event_r <= 0;
    end
    else if (dump_enable_rr | event_enable_r) begin
        if (!event_done)
            rd_ptr <= rd_ptr + 1'b1; // JGhere, use rd_ptr as BRAM RdAddr unless f3f3 is set?  use sel_rdclk for it!
        if (dump_enable_r)
            ibx <= 0;
        else if (ibx<nbx_r)
            ibx <= ibx + 1'b1;
    end
    else ibx <= 0;
end

wire [47:0] gem_datagen;
// when button is not held, send auto-gen data from data packer. else send from ethernet input
wire gem_data_mux = (hold_bit) ? gem_datagen[47:0] : data_oram[0][47:0];

reg [47:0] fiber_out[7:0];
always @(negedge ck40) begin  // 80 MHz derived from GTX_TxPLL
    fiber_out[0][47:0] <= (send_event) ? gem_data_mux       : 48'h000000000000;
    fiber_out[1][47:0] <= (send_event) ? data_oram[1][47:0] : 48'h000000000000;
    fiber_out[2][47:0] <= (send_event) ? data_oram[2][47:0] : 48'h000000000000;
    fiber_out[3][47:0] <= (send_event) ? data_oram[3][47:0] : 48'h000000000000;
    fiber_out[4][47:0] <= (send_event) ? data_oram[4][47:0] : 48'h000000000000;
    fiber_out[5][47:0] <= (send_event) ? data_oram[5][47:0] : 48'h000000000000;
    fiber_out[6][47:0] <= (send_event) ? data_oram[6][47:0] : 48'h000000000000;
    fiber_out[7][47:0] <= (send_event) ? data_oram[7][47:0] : 48'h000000000000;
end // always

// JGhere, probably a lot of lines in the next section can be deleted:
always @(posedge slwclk or posedge reset) // everything that uses 1.6 MHz clock with simple Reset (from lhc_clk)
begin
    if (reset) begin
        free_count      <= 0;
        time_count      <= 0;
        free_tc         <= 0;
        slow_tc         <= 0;
        shft_seq        <= 47'h000000000001;
        rnd_word        <= 0;
        debounced_bit   <= 0;
        hold_bit        <= 0;
        err_wait        <= 0;
        pb_pulse        <= 0;
        slowloop_count  <= 0;
        slowloop_err    <= 0;
        slowloop_stat   <= 0;
        lout_slowloop   <= 0;
        llout_slowloop  <= 0;
        in_slowloop     <= 0;
        slowloop_errcnt <= 0;
        gempad_r        <= 0;  // JGnew
    end
    else begin
        if (debounced_bit) gempad_r[4:0] <= gempad_in[5:1];

        free_count <= free_count + 1'b1;
        // slow loop checking logic:
        lout_slowloop[4] <= out_slowloop[4];
        lout_slowloop[3] <= out_slowloop[3];
        lout_slowloop[1] <= out_slowloop[1];
        lout_slowloop[0] <= out_slowloop[0];
        if (!en_fibertests) begin  // oe_rpc jtag active, usually only for short tests
            lout_slowloop[2] <= free_count[19]; // vstat[2] = slowloop2 is SLOW!!  Only ~3 HZ max from power-sense chip
            in_slowloop[0] <= rpc_i2rx[36];
            in_slowloop[1] <= rpc_smbrx;
            in_slowloop[2] <= vstat[2]; // SLOW!!  Only ~3 HZ max from power-sense chip
            in_slowloop[3] <= 1'b1; // ok, but only a real test for alct_jtag case!
            in_slowloop[4] <= 1'b1; //  only a real test for alct_jtag case
            if (en_loopbacks_r) begin
                slowloop_err[6] <= err_slowloop[1];
                slowloop_err[5] <= err_slowloop[0];
                slowloop_err[4] <= 0;
                slowloop_err[3] <= 0;
                slowloop_err[2] <= 0;
                slowloop_err[1] <= 0;
                slowloop_err[0] <= 0;
                if (free_count[18:0] == 19'h50000) begin
                    slowloop_err[7] <= (in_slowloop[2]^free_count[19]);
                    hzloop_count <= hzloop_count + 1'b1;
                end
            end
            else slowloop_err <= 0;
        end
        else begin     //  oe_alct jtag active, expect to run this for "long term" tests
            lout_slowloop[2] <= out_slowloop[2]; // gp_io4, no speed problem here...
            in_slowloop[0] <= rpc_i2rx[37];
            in_slowloop[1] <= rpc_i2rx[35];  // alct_tx1=tms_alct  bubble
            in_slowloop[2] <= gp_io4;  // alct_rx29
            in_slowloop[3] <= rpc_dsn; // alct_tx3=sel0_alct -> alct_rx30, real test   bubble
            in_slowloop[4] <= jtag_usr0_tdo; // alct_rx0, real test
            if (en_loopbacks_r) begin
                slowloop_err[7] <= 0;
                slowloop_err[6] <= 0;
                slowloop_err[5] <= 0;
                slowloop_err[4] <= err_slowloop[4];
                slowloop_err[3] <= err_slowloop[3];
                slowloop_err[2] <= err_slowloop[2];
                slowloop_err[1] <= err_slowloop[1];
                slowloop_err[0] <= err_slowloop[0];
            end
            else slowloop_err <= 0;
        end
        if (en_loopbacks_r) slowloop_count <= slowloop_count + 1'b1;
        slowloop_stat <= slowloop_stat | slowloop_err;
        if ( |slowloop_err ) slowloop_errcnt <= ((slowloop_errcnt[11:0] + 1'b1) | (slowloop_errcnt[11]<<11));

        if (free_count == 23'h7fffff) slow_tc <= 1;  // ~.6 Hz or 1.664 sec cycle, for GbE looping
        else slow_tc <= 0;
        // JGhere, probably delete a lot (most) of the previous section, above here ^^^^

        if (free_count[15:0] == 16'hffff) free_tc <= 1;  // ~24 Hz or .04 sec cycle, for debounce
        else free_tc <= 0;

        if (free_tc) shft_seq[46:0] <= {shft_seq[45:0],shft_seq[46]}; // shift a bit over time for "random" word
        if (!pb_pulse) begin
            pb_pulse <= sw[7] & !pb; // guaranteed true at least one full "free_tc" cycle
            err_wait <= 0;
            debounced_bit <= 0;
            hold_bit <= 0;
            rnd_word <= 0;
        end
        else begin
            if (free_tc & !err_wait) begin
                if (!sw[8]) rnd_word <= shft_seq; // use this for one "random" word when NOT PRBS testing.  one SLWCLK
                debounced_bit  <= 1;  // use this for a debounced test control pulse.  lasts one SLWCLK period.
                hold_bit  <= 1;  // use this for a debounced test control pulse.  lasts until button release.
                err_wait <= 1;  // use this to sync with Xmit clocks & load rnd_word in their domains?
            end
            else if (free_tc & err_wait) begin
                pb_pulse <= sw[7] & !pb;
            end
            else begin
                rnd_word <= 0;
                debounced_bit <= 0;
            end
        end
        if (gtx_ready && time_count < 8'hfe) time_count <= time_count + 1'b1; // use to delay startup; ends.
    end
end


// JGhere, probably delete all of the next ~450 lines:
always @(posedge dmbfifo_step1ck or posedge reset) // used by one bit on DmbLoopback
begin
    if (reset) begin
        en_lhcloop <= 0;
        lhcloop_tog <= 0;
    end
    else begin
        en_lhcloop <= en_loopbacks;
        if (en_lhcloop) lhcloop_tog <= !lhcloop_tog;
    end
end

always @(negedge lhc_clk90 or posedge reset) // everything that uses neg_edge lhc_clk90.
begin
    if (reset) begin
        lout_ratloop <= 0;
    end
    else begin
        lout_ratloop <= out_ratloop;
    end
end


always @(negedge lhc_clk or posedge bc0) // everything that uses lhc_clk w/simple Reset
begin
    if (bc0) begin // bc0 now High True!
        bc0_r <= 1;
        bc0_rr <= 1;
        bc0_r3 <= 1;
    end
    else begin
        bc0_r <= bc0;
        bc0_rr <= bc0_r;
        bc0_r3 <= bc0_rr;
        if (!bc0_rr & bc0_r3) bc0_led <= ~bc0_led;  // this never gets a reset...
    end
end

always @(posedge lhc_clk or posedge reset) // everything that uses lhc_clk w/simple Reset
begin
    if (reset) begin
        time_40_r <= 8'h00;
        time_40_rr <= 8'h00;
        lhcloop_tog_r <= 0;
        en_lhcloop_r <= 0;
        l_lock40 <= 0;    // monitor clk160 from TXPLL this way
        ck160_locklost <= 0;
        qpll_lock_lost <= 0;
        pulse_count <= 0;
        in_pulse_r <= 0;
        pulses_fired <= 0;
        ccb_cmdstrb_r <= 0;
        ccb_datstrb_r <= 0;
        ccb_cmd_r <= 8'h00;
        ccb_data_r <= 8'h00;
        get_bit_ptr <= 0;
        late_load_done <= 1'b0;
        results_r <= 0;
        results_hold <= 1'b0;
        tmb_cfg_out <= 1'b0;
        alct_cfg_out <= 1'b0;
        last_cmd <= 8'h00;
        ccb_rsv_r <= 0;
        tmb_l1a_relreq <= 0;
        //	   l1a_count <= 0;
        en_loopbacks <= 0;
        en_cabletests <= 1'b0; // this does nothing so far
        en_fibertests <= 1'b0; // this does nothing so far
        en_cabletests_r <= 1'b0;
        en_fibertests_r <= 1'b0;
        // were always in lhc_clk domain:
        en_loopbacks_r <= 0;
        en_loopbacks_rr <= 0;
        lhc_tog <= 0;
        lhc_tog_err <= 0;
        lout_dmbloop <= 0;
        llout_dmbloop <= 0;
        in_dmbloop <= 0;
        dmbloop_errcnt <= 0;
        dmbloop1_stat <= 0;
        dmbloop2_stat <= 0;
        dmbloop3_stat <= 0;
        loop_count <= 0;
        loutpos_ratloop <= 0;
        llout_ratloop <= 0;
        in_ratloop <= 0;
        ratloop_errcnt <= 0;
        ratloop1_stat <= 0;
        ratloop2_stat <= 0;
        ratloop3_stat <= 0;
        ratloop4_stat <= 0;
    end
    else begin

        if (time_40_rr[7] & lock40) l_lock40 <= 1;
        if (l_lock40 & (!lock40)) ck160_locklost <= 1;
        time_40_r <= time_count;
        time_40_rr <= time_40_r;  // transfer slow slwclk time counter to lhc_clk domain
        lhcloop_tog_r <= lhcloop_tog; //
        qpll_lock_lost <= !qpll_lock | qpll_lock_lost;
        in_pulse_r <= in_pulse;
        pulses_fired <= (pulses_fired|in_pulse_r);
        //	   ccb_cmdstrb_r <= (ccb_cmdstrb & !ccb_cmdstrb_r);
        //	   if(ccb_cmdstrb & !ccb_cmdstrb_r) ccb_cmd_r <= ccb_cmd;
        //	   else ccb_cmd_r <= 8'h00;
        if(ccb_cmdstrb & !ccb_cmdstrb_r) begin
            ccb_cmd_r <= ccb_cmd;
            ccb_cmdstrb_r <= 1'b1;
        end
        else ccb_cmdstrb_r <= 1'b0;

        ccb_datstrb_r <= (ccb_datstrb & !ccb_datstrb_r);
        if(ccb_datstrb & !ccb_datstrb_r) ccb_data_r <= ccb_data;
        //	   else ccb_data_r <= 8'h00;

        ccb_rsv_r <= ~_ccb_rx[25:24]; // this is ccb_reserved[3:2] controlled by CSRB6
        //	   tmb_l1a_relreq <= ~ccb_rsv_r & ~_ccb_rx[25:24]; // when TMB L1A Release & Request go out, CCB should send an L1A, check!
        tmb_l1a_relreq <= _ccb_rx[25:24]; // removed pulse logic; probably CCB won't send an L1A, check? UN-inverted this in v2p2

        en_cabletests_r <= en_cabletests;
        en_fibertests_r <= en_fibertests;
        en_lhcloop_r <=  en_lhcloop;

        // things that were always in lhc_clk domain...
        en_loopbacks_rr <= en_loopbacks_r;
        en_loopbacks_r <= en_loopbacks;
        llout_dmbloop <= lout_dmbloop;
        lout_dmbloop <= out_dmbloop;
        in_dmbloop <= rawin_dmbloop;
        if (locked & en_loopbacks_r) loop_count <= loop_count + 1'b1;
        if (en_loopbacks) lhc_tog <= ~lhc_tog;
        if (en_loopbacks && en_loopbacks_r && en_lhcloop_r) lhc_tog_err <= (lhcloop_tog ^ lhc_tog);
        else lhc_tog_err <= 0;
        dmbloop1_stat <= dmbloop1_stat | err_dmbloop[11:0];
        dmbloop2_stat <= dmbloop2_stat | err_dmbloop[23:12];
        dmbloop3_stat <= dmbloop3_stat | err_dmbloop[27:24];
        if ( |err_dmbloop ) dmbloop_errcnt <= ((dmbloop_errcnt[11:0] + 1'b1) | (dmbloop_errcnt[11]<<11));
        //    ^^^^^^^^^^^^^^^^
        llout_ratloop <= loutpos_ratloop;
        loutpos_ratloop <= out_ratloop;
        in_ratloop <= rawin_ratloop;
        ratloop4_stat[10:0] <= ratloop4_stat[10:0] | err_ratloop[46:36];
        ratloop3_stat[11:0] <= ratloop3_stat[11:0] | err_ratloop[35:24];
        ratloop2_stat[11:0] <= ratloop2_stat[11:0] | err_ratloop[23:12];
        ratloop1_stat[11:0] <= ratloop1_stat[11:0] | err_ratloop[11:0];
        if (en_fibertests) begin  // tests are disabled for step 0 & 2 (ratloop 27 & 30) so ignore
            if ( (|err_ratloop[26:0]) || ( |err_ratloop[29:28]) || ( |err_ratloop[46:31]) ) ratloop_errcnt <= ((ratloop_errcnt[11:0] + 1'b1) | (ratloop_errcnt[11]<<11));
        end
        else begin  // step tests are ON when fibertests are Off.
            if ( |err_ratloop[46:0] ) ratloop_errcnt <= ((ratloop_errcnt[11:0] + 1'b1) | (ratloop_errcnt[11]<<11));
        end



        // this is for our "CCB Pulsed Signal" checks: the "fired" register and a pulse counter based on in_pulse and "trigger"
        trigger <= ( ( in_pulse_r == 12'h001)||( in_pulse_r == 12'h002)||( in_pulse_r == 12'h004)||( in_pulse_r == 12'h008)||( in_pulse_r == 12'h010)||( in_pulse_r == 12'h020)||( in_pulse_r == 12'h040)||( in_pulse_r == 12'h080)||( in_pulse_r == 12'h100)||( in_pulse_r == 12'h200)||( in_pulse_r == 12'h800)||( in_pulse_r == 12'h400) );
        if (trigger) pulse_count <= pulse_count + 1'b1;
        // JG, check for Result Register instructions for CCB tests (CC plus CD, CE, CF or C0):
        if (ccb_cmdstrb_r && ccb_cmd_r[7:0]==8'hCC) begin  // check all 8 bits here for "CC"
            if (results_hold == 1'b0) begin   // first pass for CC cmd
                tmb_cfg_out <= last_cmd[0]; // put out bit0 on the first CC cmd
                // lock results immediately on CMD CC for some commands:    add  qpll_lock_lost etc in Data Bus check CE
                if (last_cmd[7:0]==8'hCD) results_r <= {pulse_count[11:0],last_cmd[7:0]};    // count of CCB pulsed signals
                else if(last_cmd[7:0]==8'hCE) results_r <= {ck160_locklost,!lock40,qpll_lock_lost,!qpll_lock ,ccb_data_r[7:0],last_cmd[7:0]}; // CCB data bus content
                else if(last_cmd[7:0]==8'hCF) results_r <= {pulses_fired[11:0],last_cmd[7:0]};   // observed CCB test pulses
                else if(last_cmd[7:0]==8'hC0) results_r <= {ccbrxzero_count[11:0],last_cmd[7:0]};   // check ccb_rx0 clocking
                else  results_r <= {!_ccb_rx[42],~_ccb_rx[50:48],tmb_res_out[2:0],!_ccb_rx[28],~_ccb_rx[25:22],last_cmd[7:0]};   // default is send last cmd along with a bunch of CCB signals to check
            end

            else begin    // CC cmd: second pass and beyond
                tmb_cfg_out <= results_r[get_bit_ptr]; // put out bit[i] on later CC cmds
                // lock test results as late as possible for some commands, at end of last_cmd register transmission:
                if (!late_load_done && get_bit_ptr == 5'h07) begin  // the final bit of "last_cmd" field before sending "results"
                    late_load_done <= 1'b1;  // prevents re-update of Results during an over-looped series of CC cmds
                    if(last_cmd[7:0]==8'hD0) results_r <= {dmbloop_errcnt[11:0],last_cmd[7:0]}; // total errors in 27 DMB loops, bit 11 = rollover
                    if(last_cmd[7:0]==8'hD1) results_r <= {dmbloop1_stat[11:0],last_cmd[7:0]};  // DMB loop1 signals with error
                    if(last_cmd[7:0]==8'hD2) results_r <= {dmbloop2_stat[11:0],last_cmd[7:0]};  // DMB loop2 signals with error
                    if(last_cmd[7:0]==8'hD3) results_r <= {dmbloop3_stat[11:0],last_cmd[7:0]};  // DMB loop3 signals with error
                    //                                   ^^^^ 28 bits here, only last 3:0 active for DMB loop tests
                    //		 if(last_cmd[7:0]==8'hD4) results_r <= {dmbloop4_stat[11:0],last_cmd[7:0]};  // DMB unused signals
                    //		 if(last_cmd[7:0]==8'hD5) results_r <= {dmbloop5_stat[11:0],last_cmd[7:0]};  // DMB unused signals

                    if(last_cmd[7:0]==8'hD8) results_r <= {ratloop_errcnt[11:0],last_cmd[7:0]}; // total errors in 47 fast RPC loops, bit 11=rollover
                    if(last_cmd[7:0]==8'hD9) results_r <= {ratloop1_stat[11:0],last_cmd[7:0]};  // RPC loop1 signals with error
                    if(last_cmd[7:0]==8'hDA) results_r <= {ratloop2_stat[11:0],last_cmd[7:0]};  // RPC loop2 signals with error
                    if(last_cmd[7:0]==8'hDB) results_r <= {ratloop3_stat[11:0],last_cmd[7:0]};  // RPC loop3 signals with error
                    if(last_cmd[7:0]==8'hDC) results_r <= {ratloop4_stat[11:0],last_cmd[7:0]};  // RPC loop4 signals with error
                    //                                   ^^^^ 47 bits here, only last 10:0 active, these are fast RPC loop tests
                    if(last_cmd[7:0]==8'hDD) results_r <= {~vstat[3],1'b0,~vstat[1:0],slowloop_stat[7:0],last_cmd[7:0]};  // RPC slowloop signals with error
                    //                                   ^^^^ 8 bits here, only last 7:0 active, these are the SLOW RPC loop tests
                    if(last_cmd[7:0]==8'hDE) results_r <= {hzloop_count[11:0],last_cmd[7:0]}; // counts VERY slow 1.5 Hz Loopback cycles
                    if(last_cmd[7:0]==8'hDF) results_r <= {slowloop_errcnt[11:0],last_cmd[7:0]}; // total errors in 8 slow RPC loops, bit 11=rollover

                    //		 if(last_cmd[7:0]==8'hE0) results_r <= {cable_stat[11:0],last_cmd[7:0]};   // all skewclear cables status
                    //		 if(last_cmd[7:0]==8'hE1) results_r <= {error1count[11:0],last_cmd[7:0]};  // for cable 1
                    //		 if(last_cmd[7:0]==8'hE2) results_r <= {error2count[11:0],last_cmd[7:0]};  // for cable 2
                    //		 if(last_cmd[7:0]==8'hE3) results_r <= {error3count[11:0],last_cmd[7:0]};  // for cable 3
                    //		 if(last_cmd[7:0]==8'hE4) results_r <= {error4count[11:0],last_cmd[7:0]};  // for cable 4
                    //		 if(last_cmd[7:0]==8'hE5) results_r <= {error5count[11:0],last_cmd[7:0]};  // for cable 5
                    //		 if(last_cmd[7:0]==8'hF0) results_r <= {fiber_stat[11:0],last_cmd[7:0]};  // all fibers link status
                    //		 if(last_cmd[7:0]==8'hF1) results_r <= {error_f1count[11:0],last_cmd[7:0]};  // for fiber 1
                    //		 if(last_cmd[7:0]==8'hF2) results_r <= {error_f2count[11:0],last_cmd[7:0]};  // for fiber 2
                    //		 if(last_cmd[7:0]==8'hF3) results_r <= {error_f3count[11:0],last_cmd[7:0]};  // for fiber 3
                    //		 if(last_cmd[7:0]==8'hF4) results_r <= {error_f4count[11:0],last_cmd[7:0]};  // for fiber 4
                    //		 if(last_cmd[7:0]==8'hF5) results_r <= {error_f5count[11:0],last_cmd[7:0]};  // for fiber 5
                    //		 if(last_cmd[7:0]==8'hF6) results_r <= {error_f6count[11:0],last_cmd[7:0]};  // for fiber 6
                    //		 if(last_cmd[7:0]==8'hF7) results_r <= {error_f7count[11:0],last_cmd[7:0]};  // for fiber 7
                    //		 if(last_cmd[7:0]==8'hFC) results_r <= {cable_count[11:0],last_cmd[7:0]};   // count of cable test cycles
                    if(last_cmd[7:0]==8'hFD) results_r <= {loop_count[31:20],last_cmd[7:0]}; // #1.05M counts @40MHz Loopback test cycles
                    if(last_cmd[7:0]==8'hFE) results_r <= {slowloop_count[26:15],last_cmd[7:0]}; // #32K counts @1.6 MHz Loopback cycles
                    //		 if(last_cmd[7:0]==8'hFF) results_r <= {fiber_count[11:0],last_cmd[7:0]};   // count of fiber test cycles
                end

            end

            // end of each CC cmd:
            if (get_bit_ptr == 5'd19) alct_cfg_out <= 1'b1;
            else alct_cfg_out <= 1'b0;

            results_hold <= 1'b1;
            get_bit_ptr <= get_bit_ptr + 1'b1;
        end

        else if (ccb_cmdstrb_r) begin   // handle CMDs that actually DO something... not CC, clear all CC registers
            get_bit_ptr <= 5'h00;
            results_hold <= 1'b0;
            late_load_done <= 1'b0;  // allows update of Results just once during a series of CC cmds
            tmb_cfg_out <= ccb_cmd_r[0];
            alct_cfg_out <= 1'b0;

            last_cmd <= ccb_cmd_r;
            // here we must decode what the CMDs actually DO... e.g. Controls for TMB and the slave boards
            if (ccb_cmd_r[7:2]==6'h07) begin  // this is the CCB "Stop Trig" CMD, use it to freeze tests
                if (ccb_cmd_r[1:0]==2'h1) en_loopbacks <= 1'b0;  // stop DMB & RPC loopback tests
                else if (ccb_cmd_r[1:0]==2'h2) en_cabletests <= 1'b0;  // stop skewclear pattern tests
                else if (ccb_cmd_r[1:0]==2'h3) en_fibertests <= 1'b0;  // stop fiber pattern tests
                else begin  // disable all tests by default on 2'h0?  Give datastrb the same function?
                    en_loopbacks <= 1'b0;
                    en_cabletests <= 1'b0;
                    en_fibertests <= 1'b0;
                end
            end

            //   before "Start" of tests, might need to send L1 Reset... let software control that!
            if (ccb_cmd_r[7:2]==6'h06) begin  // this is the CCB "Start Trig" CMD, use it to start tests
                if (ccb_cmd_r[1:0]==2'h1) en_loopbacks <= 1'b1;  // start DMB & RPC loopback tests
                else if (ccb_cmd_r[1:0]==2'h2) en_cabletests <= 1'b1;  // start skewclear pattern tests
                else if (ccb_cmd_r[1:0]==2'h3) en_fibertests <= 1'b1;  // start fiber pattern tests
                else begin  // enable all tests by default on 2'h0?  Give datastrb the same function?
                    en_loopbacks <= 1'b1;
                    en_cabletests <= 1'b1;
                    en_fibertests <= 1'b1;
                end
            end
        end // else if (ccb_cmdstrb_r)

    end // else: !if(reset)
end // always @ (posedge lhc_clk or posedge reset)


assign _ccb_tx[19] = alct_cfg_out;  // JG, don't invert
assign _ccb_tx[18] = tmb_cfg_out;   // JG, don't invert
assign _ccb_tx[17:9] = pulse_count[8:0];  // 9 bit alct_status, to CCB Front Panel (our LED plug)

// JGhere, Begin new code insertion for GbE and BRAMs:
wire        clkfbout;
MMCM_ADV #(
    .BANDWIDTH            ("OPTIMIZED"),
    .CLKOUT4_CASCADE      ("FALSE"),
    .CLOCK_HOLD           ("FALSE"),
    .COMPENSATION         ("ZHOLD"),
    .STARTUP_WAIT         ("FALSE"),
    .DIVCLK_DIVIDE        (4),
    .CLKFBOUT_MULT_F      (36.000),  // was 43.000, changed so FVCO <= 1200 (now 1125 MHz)
    .CLKFBOUT_PHASE       (0.000),
    .CLKFBOUT_USE_FINE_PS ("FALSE"),
    .CLKOUT0_DIVIDE_F     (18.000),  // was 21.500, for 62.5 MHz output, changed to maintain div2
    .CLKOUT0_PHASE        (0.000),
    .CLKOUT0_DUTY_CYCLE   (0.500),
    .CLKOUT0_USE_FINE_PS  ("FALSE"),
    .CLKOUT1_DIVIDE       (12),      // was 15 for 90 MHz, now 12 for 93.75 MHz
    .CLKOUT1_PHASE        (0.000),
    .CLKOUT1_DUTY_CYCLE   (0.500),
    .CLKOUT1_USE_FINE_PS  ("FALSE"),
    .CLKOUT2_DIVIDE       (13),      // was 16 for 84 MHz, now 13 for 86.538 MHz
    .CLKOUT2_PHASE        (0.000),
    .CLKOUT2_DUTY_CYCLE   (0.500),
    .CLKOUT2_USE_FINE_PS  ("FALSE"),
    .CLKOUT3_DIVIDE       (75),      // was 89 for 15.125 MHz, now 75 for 15.0 MHz
    .CLKOUT3_PHASE        (0.000),
    .CLKOUT3_DUTY_CYCLE   (0.500),
    .CLKOUT3_USE_FINE_PS  ("FALSE"),
    .CLKIN1_PERIOD        (8.0),
    .REF_JITTER1          (0.010)
) txusrclk_mmcm (
// Output clocks
    .CLKFBOUT            (clkfbout),
    .CLKFBOUTB           (),
    .CLKOUT0             (gbe_txclk2_buf),
    .CLKOUT0B            (),
    .CLKOUT1             (ck94),   // JG: ck90 not used anymore, but changed to ck94 anyway
    .CLKOUT1B            (),
    .CLKOUT2             (ck87),   // JG: ck84 not used anymore, but changed to ck87 anyway
    .CLKOUT2B            (),
    .CLKOUT3             (ck15),   // JG: ck15125 not used anymore, but changed to ck15 anyway
    .CLKOUT3B            (),
    .CLKOUT4             (),
    .CLKOUT5             (),
    .CLKOUT6             (),
// Input clock control
    .CLKFBIN             (clkfbout),
    .CLKIN1              (ck125),  // use ck125 or gbe_tx_outclk...or gbe_refck?
    .CLKIN2              (1'b0),
// Tied to always select the primary input clock
    .CLKINSEL            (1'b1),
// Ports for dynamic reconfiguration
    .DADDR               (7'h0),
    .DCLK                (1'b0),
    .DEN                 (1'b0),
    .DI                  (16'h0),
    .DO                  (),
    .DRDY                (),
    .DWE                 (1'b0),
// Ports for dynamic phase shift
    .PSCLK               (1'b0),
    .PSEN                (1'b0),
    .PSINCDEC            (1'b0),
    .PSDONE              (),
// Other control and status signals
    .LOCKED              (txoutclk_mmcm_lk),
    .CLKINSTOPPED        (),
    .CLKFBSTOPPED        (),
    .PWRDWN              (1'b0),
    .RST                 (!rxpll_lk)
);

BUFG    clkf_buf  (.O (gbe_txclk2), .I (gbe_txclk2_buf));

BUFGMUX rdclk_buf (.O (rdclk), .I0 (ck40buf),.I1 (gbe_txclk2_buf), .S(sel_rdclk));

assign rd_addr = (sel_rdclk)? tx_adr[10:2] : rd_ptr[8:0];


//----------------------------------------------------------------------------------------------------------------------
// Block Ram Generation for Pattern Injection
//----------------------------------------------------------------------------------------------------------------------
wire bram_rd_en[BRAM_LIM:0];

genvar ibram;
generate
for (ibram=12'h000; ibram<=BRAM_LIM; ibram=ibram+1'b1) begin:bramgen

    assign bram_rd_en[ibram] = send_event || ((cmd_code == 16'hf3f3) & (bk_adr==ibram) & gtx_ready);

    RAMB36E1 #(
        .DOA_REG        (0),         // Optional output register ( 0 or 1)
        .DOB_REG        (0),         // Optional output register ( 0 or 1)
        .EN_ECC_READ    ("TRUE"),
        .EN_ECC_WRITE   ("TRUE"),
        .RAM_MODE       ("SDP"),     // "SDP" or "TDP"

      //.WRITE_WIDTH_A  (0),  // 0, 1, 2, 4, 9, 18, 36
        .READ_WIDTH_A   (72), // 0, 1, 2, 4, 9, 18, 36, or 72, 64??

        .WRITE_WIDTH_B  (72), // 0, 1, 2, 4, 9, 18, 36, or 72, 64??
      //.READ_WIDTH_B   (0),  // 0, 1, 2, 4, 9, 18, or 36

        //  --WriteMode: value on output upon a write ( "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE")--
        .WRITE_MODE_A   ("READ_FIRST"),
        .WRITE_MODE_B   ("READ_FIRST")
        )
        block_ram_ecc (
            .ADDRARDADDR    ({1'b1, rd_addr[8:0], 6'h3f}),      // 16 bit RDADDR, but only 14:6 are used w/ECC.  1/3f?

            .DIADI          (data_iram[31:0]),                  // DI low 32-bit
            .DIBDI          (data_iram[63:32]),                 // DI high 32-bit

            .DOADO          (data_oram[ibram][31:0]),               // DO low 32-bit
            .DOBDO          (data_oram[ibram][63:32]),              // DO high 32-bit

            .WEA            (4'h0),                             // WEA, NA for SDP
            .ENARDEN        (bram_rd_en[ibram]),                // RDEN
            .REGCEAREGCE    (1'b0),                             // REGCE, NA if DO_REG=0
            .RSTRAMARSTRAM  (1'b0),
            .RSTRAMB        (1'b0),                             // NA if SDP
            .RSTREGARSTREG  (1'b0),                             // NA if SDP or DO_REG=0
            .RSTREGB        (1'b0),                             // NA if SDP or DO_REG=0
            .CLKARDCLK      (rdclk),                            // RDCLK
            .ADDRBWRADDR    ({1'b1, rx_adr_r[10:2], 6'h3f}),    // 16 bit WRADDR, but only 14:6 are used w/ECC.  1/3f?
            .WEBWE          (8'hFF),                            // WE?  WEBWE ( 8 bits)
            .ENBWREN        (cycle4 & (bk_adr==ibram) & gtx_ready), // WREN  Alfke: "WE off" is not sufficient to protect
            .REGCEB         (1'b0),                             // REGCEB        Init data, So require stable clocks before EN is set,
            .CLKBWRCLK      (gbe_txclk2),                       // WRCLK    and ensure safe setup time for ADR when EN is Enabled!
            .INJECTSBITERR  (1'b0),
            .SBITERR        (sbiterr_oram[ibram]),
            .INJECTDBITERR  (1'b0),
            .DBITERR        (dbiterr_oram[ibram])

            // not valid if DO_REG=0:  REGCEAREGCE, REGCEB, RSTREGARSTREG, RSTREGB
            // not used for SDP:  CASCADE*, REGCEB, RSTRAMB, RSTREGARSTREG, RSTREGB, WEA
            //.RST  (reset) // err.mon: SBITERR DBITERR
        );
end
endgenerate

mac_crc  mac_crc32(gbe_txdat, crc_en, crc_out, crc_rst, gbe_txclk2);

always @(posedge ck40 or posedge reset) // everything that uses ck40 w/simple Reset
begin
    if (reset) begin
        l_lock125 <= 0;    // use ck40(160) to monitor ck125
        ck125_locklost <= 0;
    end
    else begin
        if (time_40_rr[7] & locked) l_lock125 <= 1;
        if (l_lock125 & (!locked)) ck125_locklost <= 1;
    end
end

always @(posedge gbe_txclk2) // everything that uses GbE USR clock, no Reset
begin
    tx_out_r  <= tx_out;
    tx_kout_r <= tx_kout;
    time_r_i  <= time_count;
end

always @(posedge gbe_txclk2 or posedge reset) // everything using GbE USR clock w/simple Reset
begin
    if (reset) begin
        time_r <= 8'h00;
        tx_resetdone_r <= 0;
        rx_resetdone_r <= 0;
    end
    else begin
        tx_resetdone_r <= tx_resetdone;
        rx_resetdone_r <= rx_resetdone;
        time_r <= time_r_i;  // transfer slow clk time counter to GbE USR clock domain
    end
end



//----------------------------------------------------------------------------------------------------------------------
// GbE Command Parsing
//----------------------------------------------------------------------------------------------------------------------

wire dump_enable_next = (!dump_enable & debounced_bit) | (cmd_code == 16'hFDFD);
reg   event_done_r = 0, dump_done_r = 0, dump_loop = 0;
reg [7:0]  nbx_i=0, nbx=0;
always @(posedge gbe_txclk2 or posedge gtx_reset) // everything using GbE USR clock w/GTX_Reset
begin
    if (gtx_reset) begin
        comma_align     <= 0;
        counter_send    <= 0;
        l_rxdv          <= 0;
        l_kchar         <= 0;
        ll_kchar        <= 0;
        l_gbe_rxdat     <= 0;
        ll_gbe_rxdat    <= 0;
        kchar_r         <= 0;
        ovfl_packet     <= 0;
        counter         <= 16'h0000;
        ireg            <= 0;
        tx_adr          <= 0;
        rx_adr          <= 0;
        rx_adr_r        <= 0;
        cycle4          <= 0;
        cmd_f7f7        <= 0;
        pkt_id          <= 8'h0000;
        gbe_rxcount     <= 16'h0000;
        cmd_code        <= 16'h0000;
        bk_adr          <= 12'h000;
        gbe_txdat       <= 16'h50bc;  // <-- idle 28.5/16.2
        gbe_kout        <= 2'h1;
        sync_state      <= 2'h0;
        data_state      <= 2'h0;
        mac_seek        <= 0;
        mac_sync        <= 0;
        mac_ack         <= 0;
        rx_resetdone_r2 <= 0;
        rx_resetdone_r3 <= 0;
        crc_rst         <= 1;
        crc_en          <= 0;
        pkt_send        <= 0; // 1st data word @counter == 12 --> 11 extra words, so subtract 22 bytes.
        good_rx_cmd     <= 1'b0;
        rx_timeout      <= 1'b0;
        data_bram       <= 16'hd1d0;
        data_bram_r     <= 16'he4e2;
        sel_rdclk       <= 1'b0;
        event_done_r    <= 0;
        dump_done_r     <= 0;
        dump_loop_i     <= 0;
        dump_loop       <= 0;
        nbx_i           <= 0;
        nbx             <= 0;
    end

    else begin // Not Reset case
        event_done_r    <= event_done;
        dump_done_r     <= dump_done;
        rx_resetdone_r2 <= rx_resetdone_r;
        rx_resetdone_r3 <= rx_resetdone_r2;

        if (rx_resetdone_r3) comma_align <= 1; // maybe use time_r > 8'h10 ? // a latch ?? srs?

        crc_rst   <= (data_state == 2'h2) ? 1'b1 : 1'b0;  // needs one cycle during state 3
        dump_loop <= (dump_done_r)        ? 1'b0 : dump_loop_i; // sets FPGA for continuous loop of sending all patterns
        nbx       <= (event_done_r)       ? 1'b0 : nbx_i; // tells how many BX to send for this event (8 bits)

        if      (dump_enable_next)        dump_enable <= 1'b1;  // was !dump_done_r ...before that was 1'b1;.  dump_done is driven by ck40
        else if (cmd_code[15:12] == 4'hF) dump_enable <= 1'b0;  // stop dump on any non-FDFD command: F0F0, F3F3, F7F7, FEFE...
        else if (dump_enable)             dump_enable <= !dump_done_r;  // dump_done is driven by ck40   ... was 1'b1;
        else                              dump_enable <= dump_enable;

        if      (cmd_code == 16'hFEFE)    event_enable <= !event_done_r;  // event_done is driven by ck40
        else if (cmd_code[15:12] == 4'hF) event_enable <= 1'b0;           // stop dump on any non-FEFE command: F0F0, F3F3, F7F7, FDFD...
        else                              event_enable <= event_enable;

        if (cmd_code == 16'hF3F3) sel_rdclk <= 1'b1;
        else                      sel_rdclk <= 1'b0;  // JGhere, maybe this will change the clock before the Read is finished, so delay it?

        // load counter2 to READ registers and send it out
        if      (cmd_code == 16'hf3f3) pkt_lim <= 16'd2061;   // 16'd61 special test size 100 bytes.  later will be 4 KB
        else if (cmd_code == 16'hf7f7) pkt_lim <= 16'd2048; // just over 4 KB for good Rx/Tx BRAM loading overlap
        else                           pkt_lim <= 16'd36; // our usual 50-byte size is default; F3F3 is the special 4 KB count

        byte_count <= 16'hffff&((pkt_lim*2) - 16'd22); // Need BYTES; subtract preamble bytes etc.

        /* BRAM->PC */
        // select what goes where for the readout BRAM bus
        if ((bk_adr < 12'h000) || (bk_adr > BRAM_LIM))
            data_bram <= 16'h0a0f; // limited range of bk_adr space
        else if (cmd_code == 16'hf3f3) begin
            case (tx_adr[1:0])
                2'b00: data_bram <= data_oram[bk_adr][63:48]; // correct for latency of BRAM readout response
                2'b01: data_bram <= data_oram[bk_adr][15:0];
                2'b10: data_bram <= data_oram[bk_adr][31:16];
                2'b11: data_bram <= data_oram[bk_adr][47:32];
            endcase // case (tx_adr[1:0])
        end
        else
            data_bram <= 16'hdfd8;

        data_bram_r  <= data_bram;

        if (data_state[1]) begin  // end of sent data packet
            //	      cmd_code             <= 16'h0000;
            if (|cmd_code)
                cmd_code <= 16'h0000;
            bk_adr       <= 12'h000;
            pkt_send     <= 1'b0;
            counter      <= 0;
            if (data_state == 2'h2)
                pkt_id <= pkt_id + 1'b1;
        end
        else if (rx_timeout) begin
            bk_adr   <= 12'h000;
            counter  <= 0;
            pkt_send <= 1'b0;
        end
        else if (!sync_state[0] && pkt_send) begin
            counter <= counter + 1'b1;  // verified that sync[0] ends OFF
        end
        else if (!sync_state[0] && time_r[7] && (cmd_code > 16'h0000)) pkt_send <= 1'b1;


        /* GbE Tx to PC */
        if ((counter > 0) && (counter <= pkt_lim)) begin  // put data on gbe_txdat after preamble, MACaddr's & Length
            counter_send <= 1;
            if (counter == 16'd1) begin
                gbe_txdat <= 16'h55fb;
                gbe_kout  <= 2'h1;
                tx_adr    <= 0; /* tx_adr == BRAM readout bus */
            end
            else if (counter < 16'd5) begin
                gbe_txdat <= {counter[2],15'h5555};
                gbe_kout  <= 2'h0;
            end
            else if (counter < 16'd7) begin
                crc_en    <= 1;
                gbe_txdat <= 16'hffff; // MAC32 No Longer inverts gbe_txdat input w/counter == 5 or 6; just send ones.
                /*  JRG,  for historical reference...
                    en_levelshift <= (cmd_code != 16'hf2f2);
                    en_csr <= (cmd_code != 16'hf5f5);
                    */
            end
            else if (counter < 16'd11) begin
                gbe_txdat <= 16'h0000;
            end
            else if (counter < 16'd12) begin  // only one cycle for this step!  counter == 11
                gbe_txdat <= {byte_count[7:0],byte_count[15:8]};
                tx_adr    <= tx_adr + 1'b1; // shifting 2-deep tx_bram pipeline
                ireg      <= 0;
            end
            else if (counter < 16'd13) begin  // only one cycle for this step!  counter == 12
                gbe_txdat <= cmd_code;    //  <-- first word returns the cmd_code
                tx_adr    <= tx_adr + 1'b1; // shifting 2-deep tx_bram pipeline
            end

            // JGhere: this is the gbe command decode.  A lot of it is not needed by the CSC_GEM test stand!
            else begin // if (counter >= 16'd13).  first data word @counter == 13 --> 12 counts ahead!
                tx_adr <= tx_adr + 1'b1;
                ireg   <= ireg + cmd_code[1] + (~counter[0]&cmd_code[0]); // 32b if CMD Odd, 16b if CMD Even
                //  F5 => 0 + (cnt[0]&1), alternates ireg increments for 32-bit register readout.
                //  F6 => 1 + (cnt[0]&0), continuous increments for 16-bit register readout.

                // JGhere: next 3 lines are dummies, can be deleted...  not needed by the CSC_GEM test stand!
                if(cmd_code[1]^cmd_code[0]) begin  // generalized for f1f1 or f2f2 ( == !f3f3 && !f4f4) or f5 or f6...f9, fa, fd, fe
                    gbe_txdat <= 16'ha1a1;
                end
                else if (cmd_code[2]) begin  // Function4 is active, or f7 or fc, ff.  send cmd_code first.
                    if (counter < 16'd14)
                        gbe_txdat <= {4'hd,bk_adr[11:0]};    //  <-- 2nd word returns the bk_adr
                    else
                        gbe_txdat <= {pkt_id[4:0],rx_adr[10:0]};  // send the sequential packet ID for each packet, plus 11 bit sub-address
                end
                else if (~|cmd_code[3:0]) begin  // only command f0 can make this true
                    gbe_txdat <= prev_cmd[counter[1:0]];    //  returns preveious few cmd_codes, repeats to end of packet
                end

                else if (bk_adr <= BRAM_LIM) begin  // this is data_bram range.  for f3f3, f8f8, fbfb...
                    if (counter < 16'd14)
                        gbe_txdat <= {4'hd,bk_adr[11:0]};    //  <-- 2nd word returns the bk_adr
                    else
                        gbe_txdat <= data_bram_r[15:0];  // added a pipe register for data_bram, allows better timing?
                end
                else begin
                    gbe_txdat <= 16'h50bc;  // <-- idle 28.5/16.2
                    gbe_kout <= 2'h1;
                end
            end // if (counter >= 16'd13)
        end // if (counter > 0 && counter <= GBE_LIM)
        //jg note: the MAC_ACK etc still don't work (v11p5)...do we care?
        else if (!time_r[7] || sync_state[0]) begin  // send Sync "pairs" for a while after Reset
            if (time_r[6] && time_r[1]) begin
                mac_seek <= (ll_kchar && !l_kchar && ll_gbe_rxdat == 16'h42bc);
                if (mac_seek && l_kchar && l_gbe_rxdat == 16'hb5bc)
                    mac_sync <= 1'b1;
                if (mac_seek && mac_sync && !ll_kchar && ll_gbe_rxdat == 16'h4000)
                    mac_ack <= 1'b1;
            end
            else mac_seek <= 0;

            case (sync_state)
                2'b00: gbe_txdat <= 16'h42bc;
                2'b10: gbe_txdat <= 16'hb5bc;
                default: gbe_txdat <= {1'b0,mac_sync,14'h0000}; // <-- we want this to be the final sync state.
            endcase
            gbe_kout  <= ((~sync_state) & 2'b01);
            sync_state <= sync_state + 1'b1;
        end
        else  begin // otherwise send Idles, allows time for Rx Checking
            counter_send <= 0;
            crc_en <= 0;
            gbe_txdat <= 16'h50bc;  // <-- idle 28.5/16.2
            gbe_kout <= 2'h1;
            tx_adr <= 0;
            if (counter_send) data_state <= 2'h1;
            else if (data_state > 2'h0) data_state <= data_state + 2'h1;
        end

        l_rxdv       <= rxdv;  // note, rxdv may stay True for over .5 sec after Reset!  Why...?
        ll_kchar     <= l_kchar;
        l_kchar      <= rxer[0]|rxer[1];
        kchar_r      <= rxer[1:0];
        ll_gbe_rxdat <= l_gbe_rxdat;
        l_gbe_rxdat  <= gbe_rxdat;

        /* GbE Rx from PC */
        if (time_r[7] && rxdv) begin // get command from first 1-2 data bytes of packet
            if (gbe_rxcount == 16'd4407) begin  // tuned to handle weird .5 sec rxdv case after Reset.
                ovfl_packet <= ovfl_packet + 1'b1; // tracks when a rx packet times-out, over jumbo GbE limit
                rx_timeout <= 1'b1;  // only set for pretty big jumbo packet, over 8810 bytes
            end
            else gbe_rxcount <= gbe_rxcount + 1'b1;

            if (cmd_code != 16'hf7f7) rx_adr_r[10:0] <= 11'h000;
            else if (cycle4) rx_adr_r <= rx_adr;

            if (rx_timeout) begin
                rx_adr <= 0;
                tx_adr <= 0;
            end

            // JGhere, Break down GbE packet Rx from the PC:
            if ( rx_timeout | event_done_r | dump_done_r ) begin
                cmd_code    <= 16'h0000;
                dump_loop_i <= 0;
                nbx_i       <= 0;
                cycle4      <= 0;
                cmd_f7f7    <= 0;
            end
            else if ( (gbe_rxcount == 16'h0003) && (cmd_code == 16'h0000)  ) begin
                if ( (gbe_rxdat&16'hf0f0)==16'hf0f0 && (gbe_rxdat&16'h0f0f)!=16'h0f0f ) begin
                    cmd_code     <= gbe_rxdat; //   ^^^ make sure cmd code looks valid ^^^
                    prev_cmd[0]  <= gbe_rxdat;
                    prev_cmd[1]  <= prev_cmd[0];
                    prev_cmd[2]  <= prev_cmd[1];
                    prev_cmd[3]  <= prev_cmd[2];
                    good_rx_cmd  <= 1'b1;
                    rx_adr[10:0] <= 11'h000;
                    cycle4       <= 0;
                    cmd_f7f7     <= 0;
                end
            end
            else if ( |cmd_code && (gbe_rxcount == 16'h0004)) begin
                if (cmd_code == 16'hFDFD) dump_loop_i <= (gbe_rxdat[3:0]==4'hC);
                else dump_loop_i <= 0;
                if (cmd_code == 16'hFEFE) nbx_i <= (gbe_rxdat[7:0]);
                else nbx_i <= 0;
                if ( (cmd_code == 16'hf3f3 || cmd_code == 16'hf7f7) && (gbe_rxdat[11:8] == 4'h0) ) bk_adr <= gbe_rxdat[11:0];
                else bk_adr <= 0;
                data_iram[15:0] <= 16'hdddd;
            end
            /* PC->BRAM*/
            else if (gbe_rxcount > 16'h0004 && (cmd_code == 16'hf7f7) && (bk_adr <= BRAM_LIM) ) begin
                if (gbe_rxcount == 16'h0005)
                    cmd_f7f7 <= (cmd_code == 16'hf7f7);

                cycle4 <= (cmd_f7f7 & (rx_adr[1:0] == 2'h3)); // cannot begin before rx_count=6

                if (rx_adr != 11'h7ff)
                    rx_adr <= rx_adr + 1'b1;
                else
                    cmd_f7f7 <= 0;

                if      (rx_adr[1:0] == 2'h0) data_iram[15:0]  <= gbe_rxdat[15:0];
                else if (rx_adr[1:0] == 2'h1) data_iram[31:16] <= gbe_rxdat[15:0];
                else if (rx_adr[1:0] == 2'h2) data_iram[47:32] <= gbe_rxdat[15:0];
                else                          data_iram[63:48] <= gbe_rxdat[15:0];
            end
            else begin
                good_rx_cmd <= 1'b0;
                cycle4 <= 0;
            end // else: !if(gbe_rxcount == 3 or 4, or cmd=f7f7)
        end  // if (time_r[7] & rxdv & correct switch state)

        else  begin  // if (rxdv == 0) or time_r[7]==0
            gbe_rxcount <= 16'h0000;
            rx_timeout  <= 1'b0;
            cycle4      <= 0;
            cmd_f7f7    <= 0;
        end
    end // else: !if(gtx_reset)
end

always @(*) begin
    case (data_state)
        2'h1:    begin  // <-- last "counter_send" is in time with data_state 1
                     tx_out[15:0] = crc_out[15:0];  // send out CRC "MSB-first"; it's already inverted and backwards
                     tx_kout = 2'h0;
                 end
        2'h2:    begin
                     tx_out[15:0] = crc_out[31:16];
                     tx_kout = 2'h0;
                 end
        2'h3:    begin
                     tx_out[15:0] = 16'hf7fd; // eof code
                     tx_kout = 2'h3;
                 end
        default: begin    // tx_out control returns to gbe_txdat w/Idles
                     tx_out[15:0] = gbe_txdat[15:0];
                     tx_kout = gbe_kout;
                 end
    endcase
end  // always @ (*)


GBE_T20R20 gbe_gtx (
    gbe_refck,         // GTX0_DOUBLE_RESET_CLK_IN,

// Loopback and Powerdown Ports ----------------------
    low[2:0],          // GTX0_LOOPBACK_IN,
    low[1:0],          // GTX0_RXPOWERDOWN_IN,

// Receive Ports - 8b10b Decoder ----------------------
    rx_comma,          // GTX0_RXCHARISCOMMA_OUT,  ***1
    rxer[1:0],         // GTX0_RXCHARISK_OUT,  ***2
    rxer[3:2],         // GTX0_RXDISPERR_OUT,  ***3
    rxer[5:4],         // GTX0_RXNOTINTABLE_OUT,  ***4
    rx_disp,           // GTX0_RXRUNDISP_OUT,  ***5

// Receive Ports - Clock Correction Ports -----------------
    ignore[2:0],       // GTX0_RXCLKCORCNT_OUT,

// Receive Ports - Comma Detection and Alignment --------------
    comma_align,       // GTX0_RXENMCOMMAALIGN_IN,
    comma_align,       // GTX0_RXENPCOMMAALIGN_IN,

// Receive Ports - RX Data Path interface -----------------
    gbe_rxdat,         // GTX0_RXDATA_OUT,  compare to count_out
    !txoutclk_mmcm_lk, // GTX0_RXRESET_IN, // hold reset until rxPLL and tx_usrclk are locked
    gbe_txclk2,        // GTX0_RXUSRCLK2_IN,

// Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR ------
    ignore[3],         // GTX0_RXELECIDLE_OUT,
    gbe_rxn,           // GTX0_RXN_IN,
    gbe_rxp,           // GTX0_RXP_IN,

// Receive Ports - RX Elastic Buffer and Phase Alignment Ports -------
    reset,             // GTX0_RXBUFRESET_IN,  // don't need this one
    rx_bufstat,        // GTX0_RXBUFSTATUS_OUT,  --ignore these in RxDV?

// Receive Ports - RX Loss-of-sync State Machine --------------
    rx_lostsync,       // GTX0_RXLOSSOFSYNC_OUT,  *** 6    Use this where???

// Receive Ports - RX PLL Ports ----------------------
    reset,             // GTX0_GTXRXRESET_IN,  // should use this one
    ckgbe,             // GTX0_MGTREFCLKRX_IN,
    zero,              // GTX0_PLLRXRESET_IN, // should not use this one
    rxpll_lk,          // GTX0_RXPLLLKDET_OUT,
    rx_resetdone,      // GTX0_RXRESETDONE_OUT,

// Transmit Ports - 8b10b Encoder Control Ports --------------
    zero2,             // GTX0_TXCHARDISPMODE_IN,  ***7
    zero2,             // GTX0_TXCHARDISPVAL_IN,  ***8
    tx_kout_r,         // GTX0_TXCHARISK_IN,  ***9

// Transmit Ports - GTX Ports -----------------------
    low[12:0],         // GTX0_GTXTEST_IN,

// Transmit Ports - TX Data Path interface -----------------
    tx_out_r,          // GTX0_TXDATA_IN,
    gbe_tx_outclk,     // GTX0_TXOUTCLK_OUT,
    !rxpll_lk,         // GTX0_TXRESET_IN, // hold reset until rxPLL is locked!!!
    gbe_txclk2,        // GTX0_TXUSRCLK2_IN,

// Transmit Ports - TX Driver and OOB signaling --------------
    gbe_txn,           // GTX0_TXN_OUT,
    gbe_txp,           // GTX0_TXP_OUT,

// Transmit Ports - TX PLL Ports ----------------------
    reset,             // GTX0_GTXTXRESET_IN, // should use this one
    tx_resetdone       // GTX0_TXRESETDONE_OUT
);


/*  switch modes
sw7 & sw8:   send err_count at 1.6 Hz; PB forces an error  (use w/Tx tester, OK at TAMU)
!sw7 & sw8:  send err_count at 1.6 Hz; PB is Reset  (good to use at OSU & w/tester at TAMU)

sw7 & !sw8:  send comp_dat when non-zero; PB forces a triad pattern  (for Tx tester at TAMU)
!sw7 & !sw8: send comp_dat when non-zero; PB is Reset  (good to use at OSU & w/tester at TAMU)
*/

//----------------------------------------------------------------------------------------------------------------------
// GEM Data Emulation
//----------------------------------------------------------------------------------------------------------------------

reg [191:0] partition0 = 0;
reg [191:0] partition1 = 0;
reg [191:0] partition2 = 0;
reg [191:0] partition3 = 0;
reg [191:0] partition4 = 0;
reg [191:0] partition5 = 0;
reg [191:0] partition6 = 0;
reg [191:0] partition7 = 0;

//genvar ibit;
//
//reg [3:0] rowcnter;
//reg [7:0] keycnter;

reg [3:0] gem_hit_emu=4'b0;
parameter gem_hit_s0 = 4'b0;
parameter gem_hit_s1 = 4'b1;
parameter gem_hit_s2 = 4'b1;
parameter gem_hit_s3 = 4'b1;

always @(posedge lhc_ck) begin
case (gem_hit_emu)
    gem_hit_s0:  begin
        partition0 <= 192'h000111222333444555666777888999AAABBBCCCDDDEEEFFF;
        partition1 <= 192'h111222333444555666777888999AAABBBCCCDDDEEEFFF000;
        partition2 <= 192'h222333444555666777888999AAABBBCCCDDDEEEFFF000111;
        partition3 <= 192'h333444555666777888999AAABBBCCCDDDEEEFFF000111222;
        partition4 <= 192'h444555666777888999AAABBBCCCDDDEEEFFF000111222333;
        partition5 <= 192'h555666777888999AAABBBCCCDDDEEEFFF000111222333444;
        partition6 <= 192'h666777888999AAABBBCCCDDDEEEFFF000111222333444555;
        partition7 <= 192'h777888999AAABBBCCCDDDEEEFFF000111222333444555666;
        gem_hit_emu <= gem_hit_s1;
    end
    gem_hit_s1: begin
        partition0 <= 192'hFFFEEEDDDCCCBBBAAA999888777666555444333222111000;
        partition1 <= 192'hEEEDDDCCCBBBAAA999888777666555444333222111000FFF;
        partition2 <= 192'hDDDCCCBBBAAA999888777666555444333222111000FFFEEE;
        partition3 <= 192'hCCCBBBAAA999888777666555444333222111000FFFEEEDDD;
        partition4 <= 192'hBBBAAA999888777666555444333222111000FFFEEEDDDCCC;
        partition5 <= 192'hAAA999888777666555444333222111000FFFEEEDDDCCCBBB;
        partition6 <= 192'h999888777666555444333222111000FFFEEEDDDCCCBBBAAA;
        partition7 <= 192'h888777666555444333222111000FFFEEEDDDCCCBBBAAA999;
        gem_hit_emu <= gem_hit_s2;
    end
    gem_hit_s2: begin
        partition0 <= 192'hAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA;
        partition1 <= 192'h555555555555555555555555555555555555555555555555;
        partition2 <= 192'hAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA;
        partition3 <= 192'h555555555555555555555555555555555555555555555555;
        partition4 <= 192'hAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA;
        partition5 <= 192'h555555555555555555555555555555555555555555555555;
        partition6 <= 192'hAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA;
        partition7 <= 192'h555555555555555555555555555555555555555555555555;
        gem_hit_emu <= gem_hit_s3;
    end
    gem_hit_s3: begin
        partition0 <= 192'h555555555555555555555555555555555555555555555555;
        partition1 <= 192'hAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA;
        partition2 <= 192'h555555555555555555555555555555555555555555555555;
        partition3 <= 192'hAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA;
        partition4 <= 192'h555555555555555555555555555555555555555555555555;
        partition5 <= 192'hAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA;
        partition6 <= 192'h555555555555555555555555555555555555555555555555;
        partition7 <= 192'hAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA;
        gem_hit_emu <= gem_hit_s0;
    end
endcase
end

wire [13:0] hit0;
wire [13:0] hit1;
wire [13:0] hit2;

csc_hit_packer u_csc_hit_packer (
    .clock      (lhc_ck),
    .reset      (1'b0),

    .partition0 (partition0),
    .partition1 (partition1),
    .partition2 (partition2),
    .partition3 (partition3),
    .partition4 (partition4),
    .partition5 (partition5),
    .partition6 (partition6),
    .partition7 (partition7),

    .hit0       (hit0),
    .hit1       (hit1),
    .hit2       (hit2),

    .bsy1 (bsy1),
    .bsy2 (bsy2),

    .cluster_sep     (4'b1),
    .cluster_sep_adj (4'b0),
    .mask_adj        (1'b1),

    .data_out        (gem_datagen)
);

wire sump;
assign sump = (|hit0[13:0]) | (|hit1[13:0]) | (|hit2[13:0]) | (|bsy1) | (|bsy2);

//----------------------------------------------------------------------------------------------------------------------
// LED Assignments
//----------------------------------------------------------------------------------------------------------------------

always @(*)
begin // JG, v1p16: swap LED 3<>4, notes and all
    led_low[0] = !_ccb_rx[22] | sump;     // ccb_ttcrx_rdy, always OFF
    led_low[1] = !_ccb_rx[23];            // ccb_qpll_lck, OFF but often blinks ON (very short)
    led_low[2] =  qpll_lock;              // always ON!   was !_ccb_rx[31] == _alct_adb_pulse_async
    led_low[3] =  qpll_lock_lost;         // always OFF?  was _ccb_rx[35] == mpc_in1, always ON
    led_low[4] =  lhc_clk;
    led_low[5] =  ck160_locked;           // always ON!                                              // Tx GTX PLL Ref lock
    led_low[6] = !ck160_locklost;         // --> ON!  comes from mmcm driven by tx_pll_ck160 in FPGA
    led_low[7] =  lhc_clk;                // just reset, includes ccb_rx[1]==L1reset

    led_hi[8]  = !(hold_bit | gtx_reset); // M1: synced with PB & errors at crate Rx
    led_hi[9]  =  qpll_lock_lost;         // 0
    led_hi[10] = !cmd_code[7];            // !gtx_reset; // 0
    led_hi[11] = !rd_ptr[8];              // ~6.25 usec  // gtx_ready;             // 1
    led_hi[12] = !dump_enable_rr;         // 12.5 usec   // !ck160_locklost;       // 0
    led_hi[13] = !dump_enable_r;          // 12.5 usec   // (locked & lhc_locked); // 1
    led_hi[14] = !dump_enable;            // 12.5 usec
    led_hi[15] = !dump_done;              // 50ns

    test_led[9]   = (sw[8]==1'b1 && sw[7]==1'b1) ? lhc_ck : bc0;
    test_led[10]  = bc0_r3;

end

//----------------------------------------------------------------------------------------------------------------------
// snap12 mmcm clock manager
//----------------------------------------------------------------------------------------------------------------------

SRL16E #(.INIT(16'h7FFF)) SRL16TXPLL(.Q(txpll_rst), .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b1), .CE (1'b1), .CLK(qpll_ck40), .D(1'b0));
SRL16E #(.INIT(16'h0FFF)) SRL16MMCM(.Q(ck160_rst), .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b1), .CE (ck160_locked), .CLK(qpll_ck40), .D(1'b0));

//----------------------------------------------------------------------------------------------------------------------
// snap12 mmcm clock manager
//----------------------------------------------------------------------------------------------------------------------

//     mmcm from 80 MHz:               In,    out80,  out160,   out40,   reset,       locked
//   bufg_x2div2 snap_mmcm (tx_clk_out, tx_clk, snap_clk2, ck40, !ck160_locked, lock40); // from Tx GTX PLL out clk
//     mmcm from 80 MHz:                 In,    out80,  out160,   out40,   reset,       locked,   out40-no-bufg
bufg_x2div2plus snap_mmcm (tx_clk_out, tx_clk, snap_clk2, ck40, !ck160_locked, lock40, ck40buf); // from Tx GTX PLL out clk

//----------------------------------------------------------------------------------------------------------------------
// GEM + CFEB Fiber Outputs
//----------------------------------------------------------------------------------------------------------------------

// JGhere, fiber Tx "from DCFEB" section:
//   was Snap12 module, not cfeb-tmb test code
tmb_fiber_out  gem0out   (
    .RST                 (reset),                 // Manual only
    .TRG_SIGDET          (),                      // from IPAD to IBUF.  N/A?
    .TRG_RX_N            (),                      // empty
    .TRG_RX_P            (),                      // empty
    .TRG_TDIS            (),                      // OBUF output, for what?  N/A?
    .TRG_TX_N            (txn[0]),                // pick a fiber
    .TRG_TX_P            (txp[0]),                // pick a fiber
    .G6C                 (fiber_out[0][7:0]),     // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
    .G5C                 (fiber_out[0][15:8]),    // if ENA_TEST_PAT then it's prbs so these don't matter...
    .G4C                 (fiber_out[0][23:16]),   // but good for testing low-rate non-zero triad data:
    .G3C                 (fiber_out[0][31:24]),
    .G2C                 (fiber_out[0][39:32]),
    .G1C                 (fiber_out[0][47:40]),
    .TRG_TX_REFCLK       (ck160),                 // QPLL 160 from MGT clk
    .TRG_TXUSRCLK        (snap_clk2),             // get 160 from TXOUTCLK (times 2)
    .TRG_CLK80           (tx_clk),                // get 80 from TXOUTCLK
    .TRG_GTXTXRST        (txpll_rst),             // maybe Manual "reset" only
    .TRG_TX_PLLRST       (txpll_rst),             // Tie LOW.
    .TRG_RST             (fiber_reset),           // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
    .ENA_TEST_PAT        (sw[8]),                 // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
    .INJ_ERR             (ferr_f[0] & sw[8]),     // use my switch/PB combo logic for this, high-true? Pulse high once.
    .TRG_SD              (),                      // from IBUF, useless output. N/A
    .TRG_TXOUTCLK        (),                      // 80 MHz; This has to go to MCM to generate 160/80
    .TRG_TX_PLL_LOCK     (),                      // inverse holds the MCM in Reset                                                                                                                                                                       // Tx GTX PLL Ref lock
    .TRG_TXRESETDONE     (),                      // N/A
    .TX_SYNC_DONE        (),                      // not used in DCFEB tests
    .STRT_LTNCY          (tx_begin[0]),           // after every Reset, to TP for debug only  -- !sw7 ?
    .LTNCY_TRIG          (tx_fc[0]),              // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
    .MON_TX_SEL          (),                      // N/A
    .MON_TRG_TX_ISK      (),                      // N/A returns 4 bits
    .MON_TRG_TX_DATA     ()                       // N/A returns 32 bits
);

tmb_fiber_out  dcfeb1out (
    .RST                 ( reset),               // Manual only
    .TRG_SIGDET          ( ),                    // from IPAD to IBUF.  N/A?
    .TRG_RX_N            ( ),                    // empty
    .TRG_RX_P            ( ),                    // empty
    .TRG_TDIS            ( ),                    // OBUF output, for what?  N/A?
    .TRG_TX_N            ( txn[1]),              // pick a fiber
    .TRG_TX_P            ( txp[1]),              // pick a fiber
    .G1C                 ( fiber_out[1][7:0]),   // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
    .G2C                 ( fiber_out[1][15:8]),  // if ENA_TEST_PAT then it's prbs so these don't matter...
    .G3C                 ( fiber_out[1][23:16]), // but good for testing low-rate non-zero triad data:
    .G4C                 ( fiber_out[1][31:24]),
    .G5C                 ( fiber_out[1][39:32]),
    .G6C                 ( fiber_out[1][47:40]),
    .TRG_TX_REFCLK       ( ck160),               // QPLL 160 from MGT clk
    .TRG_TXUSRCLK        ( snap_clk2),           // get 160 from TXOUTCLK (times 2)
    .TRG_CLK80           ( tx_clk),              // get 80 from TXOUTCLK
    .TRG_GTXTXRST        ( txpll_rst),           // maybe Manual "reset" only
    .TRG_TX_PLLRST       ( txpll_rst),           // Tie LOW.
    .TRG_RST             ( fiber_reset),         // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
    .ENA_TEST_PAT        ( sw[8]),               // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
    .INJ_ERR             ( ferr_f[1] & sw[8]),   // use my switch/PB combo logic for this, high-true? Pulse high once.
    .TRG_SD              ( ),                    // from IBUF, useless output. N/A
    .TRG_TXOUTCLK        ( tx_clk_out),          // 80 MHz; This has to go to MCM to generate 160/80
    .TRG_TX_PLL_LOCK     ( ck160_locked),        // inverse holds the MCM in Reset                                                                                                                                                                       // Tx GTX PLL Ref lock
    .TRG_TXRESETDONE     ( ),                    // N/A
    .TX_SYNC_DONE        ( synced_snapt),        // not used in DCFEB tests
    .STRT_LTNCY          ( tx_begin[1]),         // after every Reset, to TP for debug only  -- !sw7 ?
    .LTNCY_TRIG          ( tx_fc[1]),            // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
    .MON_TX_SEL          ( ),                    // N/A
    .MON_TRG_TX_ISK      ( ),                    // N/A returns 4 bits
    .MON_TRG_TX_DATA     ( )                     // N/A returns 32 bits
);

tmb_fiber_out  dcfeb2out (
    .RST                 ( reset),               // Manual only
    .TRG_SIGDET          ( ),                    // from IPAD to IBUF.  N/A?
    .TRG_RX_N            ( ),                    // empty
    .TRG_RX_P            ( ),                    // empty
    .TRG_TDIS            ( ),                    // OBUF output, for what?  N/A?
    .TRG_TX_N            ( txn[2]),              // pick a fiber
    .TRG_TX_P            ( txp[2]),              // pick a fiber
    .G1C                 ( fiber_out[2][7:0]),   // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
    .G2C                 ( fiber_out[2][15:8]),  // if ENA_TEST_PAT then it's prbs so these don't matter...
    .G3C                 ( fiber_out[2][23:16]), // but good for testing low-rate non-zero triad data:
    .G4C                 ( fiber_out[2][31:24]),
    .G5C                 ( fiber_out[2][39:32]),
    .G6C                 ( fiber_out[2][47:40]),
    .TRG_TX_REFCLK       ( ck160),               // QPLL 160 from MGT clk
    .TRG_TXUSRCLK        ( snap_clk2),           // get 160 from TXOUTCLK (times 2)
    .TRG_CLK80           ( tx_clk),              // get 80 from TXOUTCLK
    .TRG_GTXTXRST        ( 0),                   // maybe Manual "reset" only
    .TRG_TX_PLLRST       ( 0),                   // Tie LOW.
    .TRG_RST             ( fiber_reset),         // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
    .ENA_TEST_PAT        ( sw[8]),               // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
    .INJ_ERR             ( ferr_f[2] & sw[8]),   // use my switch/PB combo logic for this, high-true? Pulse high once.
    .TRG_SD              ( ),                    // from IBUF, useless output. N/A
    .TRG_TXOUTCLK        ( ),                    // 80 MHz; This has to go to MCM to generate 160/80
    .TRG_TX_PLL_LOCK     ( ),                    // inverse holds the MCM in Reset                                                                                                                                                                       // Tx GTX PLL Ref lock
    .TRG_TXRESETDONE     ( ),                    // N/A
    .TX_SYNC_DONE        ( ),                    // not used in DCFEB tests
    .STRT_LTNCY          ( tx_begin[2]),         // after every Reset, to TP for debug only  -- !sw7 ?
    .LTNCY_TRIG          ( tx_fc[2]),            // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
    .MON_TX_SEL          ( ),                    // N/A
    .MON_TRG_TX_ISK      ( ),                    // N/A returns 4 bits
    .MON_TRG_TX_DATA     ( )                     // N/A returns 32 bits
);

tmb_fiber_out  dcfeb3out (
    .RST                 ( reset),               // Manual only
    .TRG_SIGDET          ( ),                    // from IPAD to IBUF.  N/A?
    .TRG_RX_N            ( ),                    // empty
    .TRG_RX_P            ( ),                    // empty
    .TRG_TDIS            ( ),                    // OBUF output, for what?  N/A?
    .TRG_TX_N            ( txn[3]),              // pick a fiber
    .TRG_TX_P            ( txp[3]),              // pick a fiber
    .G1C                 ( fiber_out[3][7:0]),   // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
    .G2C                 ( fiber_out[3][15:8]),  // if ENA_TEST_PAT then it's prbs so these don't matter...
    .G3C                 ( fiber_out[3][23:16]), // but good for testing low-rate non-zero triad data:
    .G4C                 ( fiber_out[3][31:24]),
    .G5C                 ( fiber_out[3][39:32]),
    .G6C                 ( fiber_out[3][47:40]),
    .TRG_TX_REFCLK       ( ck160),               // QPLL 160 from MGT clk
    .TRG_TXUSRCLK        ( snap_clk2),           // get 160 from TXOUTCLK (times 2)
    .TRG_CLK80           ( tx_clk),              // get 80 from TXOUTCLK
    .TRG_GTXTXRST        ( 0),                   // maybe Manual "reset" only
    .TRG_TX_PLLRST       ( 0),                   // Tie LOW.
    .TRG_RST             ( fiber_reset),         // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
    .ENA_TEST_PAT        ( sw[8]),               // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
    .INJ_ERR             ( ferr_f[3] & sw[8]),   // use my switch/PB combo logic for this, high-true? Pulse high once.
    .TRG_SD              ( ),                    // from IBUF, useless output. N/A
    .TRG_TXOUTCLK        ( ),                    // 80 MHz; This has to go to MCM to generate 160/80
    .TRG_TX_PLL_LOCK     ( ),                    // inverse holds the MCM in Reset                                                                                                                                                                       // Tx GTX PLL Ref lock
    .TRG_TXRESETDONE     ( ),                    // N/A
    .TX_SYNC_DONE        ( ),                    // not used in DCFEB tests
    .STRT_LTNCY          ( tx_begin[3]),         // after every Reset, to TP for debug only  -- !sw7 ?
    .LTNCY_TRIG          ( tx_fc[3]),            // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
    .MON_TX_SEL          ( ),                    // N/A
    .MON_TRG_TX_ISK      ( ),                    // N/A returns 4 bits
    .MON_TRG_TX_DATA     ( )                     // N/A returns 32 bits
);

tmb_fiber_out  dcfeb4out (
    .RST                 ( reset),               // Manual only
    .TRG_SIGDET          ( ),                    // from IPAD to IBUF.  N/A?
    .TRG_RX_N            ( ),                    // empty
    .TRG_RX_P            ( ),                    // empty
    .TRG_TDIS            ( ),                    // OBUF output, for what?  N/A?
    .TRG_TX_N            ( txn[4]),              // pick a fiber
    .TRG_TX_P            ( txp[4]),              // pick a fiber
    .G1C                 ( fiber_out[4][7:0]),   // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
    .G2C                 ( fiber_out[4][15:8]),  // if ENA_TEST_PAT then it's prbs so these don't matter...
    .G3C                 ( fiber_out[4][23:16]), // but good for testing low-rate non-zero triad data:
    .G4C                 ( fiber_out[4][31:24]),
    .G5C                 ( fiber_out[4][39:32]),
    .G6C                 ( fiber_out[4][47:40]),
    .TRG_TX_REFCLK       ( ck160),               // QPLL 160 from MGT clk
    .TRG_TXUSRCLK        ( snap_clk2),           // get 160 from TXOUTCLK (times 2)
    .TRG_CLK80           ( tx_clk),              // get 80 from TXOUTCLK
    .TRG_GTXTXRST        ( 0),                   // maybe Manual "reset" only
    .TRG_TX_PLLRST       ( 0),                   // Tie LOW.
    .TRG_RST             ( fiber_reset),         // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
    .ENA_TEST_PAT        ( sw[8]),               // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
    .INJ_ERR             ( ferr_f[4] & sw[8]),   // use my switch/PB combo logic for this, high-true? Pulse high once.
    .TRG_SD              ( ),                    // from IBUF, useless output. N/A
    .TRG_TXOUTCLK        ( ),                    // 80 MHz; This has to go to MCM to generate 160/80
    .TRG_TX_PLL_LOCK     ( ),                    // inverse holds the MCM in Reset                                                                                                                                                                       // Tx GTX PLL Ref lock
    .TRG_TXRESETDONE     ( ),                    // N/A
    .TX_SYNC_DONE        ( ),                    // not used in DCFEB tests
    .STRT_LTNCY          ( tx_begin[4]),         // after every Reset, to TP for debug only  -- !sw7 ?
    .LTNCY_TRIG          ( tx_fc[4]),            // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP--sw8,7
    .MON_TX_SEL          ( ),                    // N/A
    .MON_TRG_TX_ISK      ( ),                    // N/A returns 4 bits
    .MON_TRG_TX_DATA     ( )                     // N/A returns 32 bits
);

tmb_fiber_out  dcfeb5out  (
    .RST                  (reset),               // Manual only
    .TRG_SIGDET           (),                    // from IPAD to IBUF.  N/A?
    .TRG_RX_N             (),                    // empty
    .TRG_RX_P             (),                    // empty
    .TRG_TDIS             (),                    // OBUF output, for what?  N/A?
    .TRG_TX_N             (txn[5]),              // pick a fiber
    .TRG_TX_P             (txp[5]),              // pick a fiber
    .G1C                  (fiber_out[5][7:0]),   // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
    .G2C                  (fiber_out[5][15:8]),  // if ENA_TEST_PAT then it's prbs so these don't matter...
    .G3C                  (fiber_out[5][23:16]), // but good for testing low-rate non-zero triad data:
    .G4C                  (fiber_out[5][31:24]),
    .G5C                  (fiber_out[5][39:32]),
    .G6C                  (fiber_out[5][47:40]),
    .TRG_TX_REFCLK        (ck160),               // QPLL 160 from MGT clk
    .TRG_TXUSRCLK         (snap_clk2),           // get 160 from TXOUTCLK (times 2)
    .TRG_CLK80            (tx_clk),              // get 80 from TXOUTCLK
    .TRG_GTXTXRST         (0),                   // maybe Manual "reset" only
    .TRG_TX_PLLRST        (0),                   // Tie LOW.
    .TRG_RST              (fiber_reset),         // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
    .ENA_TEST_PAT         (sw[8]),               // HIGH for PRBS! (Low will send data from GxC registers) Use This Later, send low-rate pattern.
    .INJ_ERR              (ferr_f[5] & sw[8]),   // use my switch/PB combo logic for this, high-true? Pulse high once.
    .TRG_SD               (),                    // from IBUF, useless output. N/A
    .TRG_TXOUTCLK         (),                    // 80 MHz; This has to go to MCM to generate 160/80
    .TRG_TX_PLL_LOCK      (),                    // inverse holds the MCM in Reset                                                                                                                                                                       // Tx GTX PLL Ref lock
    .TRG_TXRESETDONE      (),                    // N/A
    .TX_SYNC_DONE         (),                    // not used in DCFEB tests
    .STRT_LTNCY           (tx_begin[5]),         // after every Reset, to TP for debug only  -- !sw7 ?
    .LTNCY_TRIG           (tx_fc[5]),            // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
    .MON_TX_SEL           (),                    // N/A
    .MON_TRG_TX_ISK       (),                    // N/A returns 4 bits
    .MON_TRG_TX_DATA      ()                     // N/A returns 32 bits
);

tmb_fiber_out  dcfeb6out  (
    .RST                  ( reset),               // Manual only
    .TRG_SIGDET           ( ),                    // from IPAD to IBUF.  N/A?
    .TRG_RX_N             ( ),                    // empty
    .TRG_RX_P             ( ),                    // empty
    .TRG_TDIS             ( ),                    // OBUF output, for what?  N/A?
    .TRG_TX_N             ( txn[6]),              // pick a fiber
    .TRG_TX_P             ( txp[6]),              // pick a fiber
    .G1C                  ( fiber_out[6][7:0]),   // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
    .G2C                  ( fiber_out[6][15:8]),  // if ENA_TEST_PAT then it's prbs so these don't matter...
    .G3C                  ( fiber_out[6][23:16]), // but good for testing low-rate non-zero triad data:
    .G4C                  ( fiber_out[6][31:24]),
    .G5C                  ( fiber_out[6][39:32]),
    .G6C                  ( fiber_out[6][47:40]),
    .TRG_TX_REFCLK        ( ck160),               // QPLL 160 from MGT clk
    .TRG_TXUSRCLK         ( snap_clk2),           // get 160 from TXOUTCLK (times 2)
    .TRG_CLK80            ( tx_clk),              // get 80 from TXOUTCLK
    .TRG_GTXTXRST         ( 0),                   // maybe Manual "reset" only
    .TRG_TX_PLLRST        ( 0),                   // Tie LOW.
    .TRG_RST              ( fiber_reset),         // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
    .ENA_TEST_PAT         ( sw[8]),               // HIGH for PRBS! (Low will send data from GxC registers) Use This Later, send low-rate pattern.
    .INJ_ERR              ( ferr_f[6] & sw[8]),   // use my switch/PB combo logic for this, high-true? Pulse high once.
    .TRG_SD               ( ),                    // from IBUF, useless output. N/A
    .TRG_TXOUTCLK         ( ),                    // 80 MHz; This has to go to MCM to generate 160/80
    .TRG_TX_PLL_LOCK      ( ),                    // inverse holds the MCM in Reset                                                                                                                                            // Tx GTX PLL Ref lock
    .TRG_TXRESETDONE      ( ),                    // N/A
    .TX_SYNC_DONE         ( ),                    // not used in DCFEB tests
    .STRT_LTNCY           ( tx_begin[6]),         // after every Reset, to TP for debug only  -- !sw7 ?
    .LTNCY_TRIG           ( tx_fc[6]),            // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
    .MON_TX_SEL           ( ),                    // N/A
    .MON_TRG_TX_ISK       ( ),                    // N/A returns 4 bits
    .MON_TRG_TX_DATA      ( )                     // N/A returns 32 bits
);

tmb_fiber_out  dcfeb7out (
    .RST                 ( reset),               // Manual only
    .TRG_SIGDET          ( ),                    // from IPAD to IBUF.  N/A?
    .TRG_RX_N            ( ),                    // empty
    .TRG_RX_P            ( ),                    // empty
    .TRG_TDIS            ( ),                    // OBUF output, for what?  N/A?
    .TRG_TX_N            ( txn[7]),              // pick a fiber
    .TRG_TX_P            ( txp[7]),              // pick a fiber
    .G1C                 ( fiber_out[7][7:0]),   // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
    .G2C                 ( fiber_out[7][15:8]),  // if ENA_TEST_PAT then it's prbs so these don't matter...
    .G3C                 ( fiber_out[7][23:16]), // but good for testing low-rate non-zero triad data:
    .G4C                 ( fiber_out[7][31:24]),
    .G5C                 ( fiber_out[7][39:32]),
    .G6C                 ( fiber_out[7][47:40]),
    .TRG_TX_REFCLK       ( ck160),               // QPLL 160 from MGT clk
    .TRG_TXUSRCLK        ( snap_clk2),           // get 160 from TXOUTCLK                                                                                                                                                                           ( times 2)
    .TRG_CLK80           ( tx_clk),              // get 80 from TXOUTCLK
    .TRG_GTXTXRST        ( 0),                   // maybe Manual "reset" only
    .TRG_TX_PLLRST       ( 0),                   // Tie LOW.
    .TRG_RST             ( fiber_reset),         // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
    .ENA_TEST_PAT        ( sw[8]),               // HIGH for PRBS!                                                                                                                                                                                  ( Low will send data from GxC registers)  Use This Later, send low-rate pattern.
    .INJ_ERR             ( ferr_f[7] & sw[8]),   // use my switch/PB combo logic for this, high-true? Pulse high once.
    .TRG_SD              ( ),                    // from IBUF, useless output. N/A
    .TRG_TXOUTCLK        ( ),                    // 80 MHz; This has to go to MCM to generate 160/80
    .TRG_TX_PLL_LOCK     ( ),                    // inverse holds the MCM in Reset                                                                                                                                           // Tx GTX PLL Ref lock
    .TRG_TXRESETDONE     ( ),                    // N/A
    .TX_SYNC_DONE        ( ),                    // not used in DCFEB tests
    .STRT_LTNCY          ( tx_begin[7]),         // after every Reset, to TP for debug only  -- !sw7 ?
    .LTNCY_TRIG          ( tx_fc[7]),            // bring out to TP.  Signals when TX sends "FC"                                                                                                                                                    ( once every 128 BX).  Send raw to TP  --sw8,7
    .MON_TX_SEL          ( ),                    // N/A
    .MON_TRG_TX_ISK      ( ),                    // N/A returns 4 bits
    .MON_TRG_TX_DATA     ( )                     // N/A returns 32 bits
);

//----------------------------------------------------------------------------------------------------------------------
endmodule
//----------------------------------------------------------------------------------------------------------------------
