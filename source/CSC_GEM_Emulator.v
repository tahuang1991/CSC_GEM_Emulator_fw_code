`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// *** JRG, firmware for Fiber Tx Bench Board, used for GEM-CSC pattern tests **
//  Only for use on 2010 prototype boards; compile it for the XC6VLX195T FPGA
//
// Create Date:    1:22 1/15/13
// Design Name: 
// Module Name:    CSC_GEM_emulator
//
// started from dcfeb_test code v3.19.  Versioning info below.
//  1.1: just get PRBS test working, with err_count sent to GbE at ~1 Hz. sw8: Force err on PB & sw7. why groups of 4?
//        -- use MTP fiber 12
//  1.2: try to add triad word inject and readout via GbE. !sw8: load data on PB & sw7. -- bug in gbe tx_dat logic
//  1.3: fixed bug, works well.  Maybe too many triad words get injected... tuned LED assignments
//  1.4: constrain inject triad to single clk80 period, hijack ferr_r logic.   d001f004?
//        -- I don't see any cause for groups of 4 forced errors...
//  1.5: bring rx_fc to test_led10.  also qpll_lock, rx_sync_done.  changed these 4 TPs are set same (5,7,9,10).
//  1.6: send sw[8:7] to FP LEDs, also QPLL_Lock, rst/pb...etc.
//  1.7: add NullVMEop logic, mod UCF.  debug triad transmit function...use to test triad RECEIVE function!
//  1.8: edit UCF, lots of backplane probing logic for ccb_rx1,12,22-35 to LEDs and ccb_tx0-19 to CCB.
//        -- remove Translator Loopback test & set on-board TMB bidir bus control signals to good state:
//         /gtl_oe=0, gtl_loop=0, dmb_loop=0, rpc_loop=0, ccb_status_oe=1, /dmb_oe=0
//  1.9: adjust some output assign values, make fp-LED7 OR of !pb + !L1reset
//  1.10: add control for both hard_reset_fpga signals, invert some fp-LED signals,
//        add trigger counter for 10 CCB pulses and CCB L1Reset function
//  1.11: add 16-bit-serialized readout of results register via TMB_CFG_DONE, with ALCT_CFG_DONE as handshake
//        -- activated with CCB_CMD 33h (load CCB VME "CMD bus" register with CCh)
//        -- any non-33 CMD will clear the results register
//        -- currently sends only the results for pulse_count
//  1.12: increase Results register to 20 bits, decode & return all 8 bits of the CMD code
//        -- establish a few new active functions: readback data_reg & pulses_fired, include ALCTadbs & HardRst, also TMBres0
//        -- defined a set of futire functions that will be needed
//  1.13: make some fixes in pulses_fired and results_r
//  1.14: make some fixes in cmd CC results & get_bit_ptr.  Added logic to check most CCB signals to TMB using CMDs CC & C0.
//        -- also trying 5-bit tmb_reserved_in data bus to the CCB & added a test for TMB_L1Arelease/request lines
//  1.15: fix serial results_reg "end" signal, change tmb_l1a_relreq logic, tweak ccb_cmd_r control logic
//  1.16: add Lock monitor logic for QPLL and ck160, put ccb_rx[0] into a BUFR for improved performance, but did not help!
//  2.0:  TMB mainboard is now loopback-safe (2.5 V) so enable DMB-Loop & RPC-Loop.  Enable Loops for CCB test signals.
//  2.1:  Add all DMB-Loop logic & load readback registers for software checks.  NO UCF entries for ccb_rx[50:48], yikes!
//  2.2:  Added UCF entries for ccb_rx[50:48], UN-inverted tmb_l1a_relreq inputs from ccb_rx[25:24].
//        -- Add TrigStop/Start functions for test control, but defaults to ON at Reset, Start not normally needed?
//  2.3:  Add fixes for dmbfifo_step1ck tests.  No clock, looks frozen; try Step1 next....
//  2.4:  Add STEP control, use ODDR to drive step1 as mirror of lhc_ck to dmbfifo_ck via DMB_Loopback
//  2.5:  Put all 40 MHz functions into lhc_ck domain, add 7 ccb_cmd diagnostic signals & qpll_lock to test_leds
//  2.6:  Use tmb_clock0 now for CCB bus communications, so Step4 can be used in other tests
//        -- LONG TERM: FIX the clocking to the QPLL, better to use tmb_clock1 instead of tmb_clock05p (FPGA SEU can screw QPLL)
//        -- fixed all DMB Loop logic to run on slwclk, bring loop[3] out&in to LEDs
//  2.7:  add another EN delay inside PRBS39 & initialize ALL registers in there; add llout_dmbloop for testLEDs;
//        add good_dmbloop monitoring too
//        -- force bit err_dmbloop[27] to zero, but add debug for its lhc_tog_err in sw[7] testLEDs
//  2.8:  take dmbloop back to 40 MHz; fix trig_stop/start 07/06 encoding; activate step4 with en_loopbacks (no sw[8] now)
//  2.9:  en_loopbacks defails to 0 now; add a lhcloop_ck-lhc_ck sync step for en_lhcloop before toggle error check
//  2.10: bring err_dmbloop[3] to sw7 Low test_LED; put LHC_CK into PRBS39!   Note: STEP4 KILLS QPLL (TMB_clock0) @TMB!!
// 
//  3.0:  add RAT Loop signals, new UCF too: 47 easy out-N-back + 8 more complicated involving RPCtx/JTAG control
//        -- 55 outs: RPCtx(0-7), RPCrx(10-25), ALCTtx(0-29,31).  5 + 2 JTAG lines are SLOW (1.6 MHz), and 1 is SLOWER (~1 Hz)
//        -- 55 ins (inc. JTAG MUXes on TMB):  RPCrx(0-9,26-38), ALCTrx(0-30), 1 Hz Vstat2.  Make 47 FAST results + 8 SLOW results
//        -- Results include RATstat1-4(47), SLOWstat(8), RATerrCnt, SLOWerrCnt, SLOWcount & HzCount for # of respective trials
//        -- JTAG "5" test enabled with (en_loopbacks & en_fibertests)
//        -- JTAG "3" test (inc. 1 Hz Vstat2) enabled with (en_loopbacks & !en_fibertests)
//        -- step4 enabled with (en_loopbacks & !en_fibertests): allows tests for DmbFIFOclk & RATloop(27,30)  (step 1,0,2 resp.)
//        -- Reset & Powerup default is  en_loopbacks = en_fibertests = en_cabletests = 0
//  3.1:  take care of "step" test cases (step 1,0,2).  These are dmb_loop27, rat_loop27 and rat_loop30.
//  3.2:  fix a startup bug in dmbloop[27], fix bug in ratloop[24], bring 4 RATloops to test_leds (1,13,18,19)
//        -- note: en_loopbacks should not be set before en_fibertests
//  3.3:  bring out RAWIN signals to check ratloop timing relative to lhc_ck and "louts"
//        -- first check EPIC if "in_" & "llout_" are not put into IO block registers.  -> YES, in IOBs.
//  3.4:  send LOUTs for ratloops 0.5 lhc_ck cycle sooner (old LOUTs called LOUTPOS now).  Fix typo on ratloop18.
//        -- *temp* just for testLED selection, made en_cabletests equivalent to sw[7] (send cmd = 1a in software)
//  3.5:  CHANGED to SPEED GRADE -1!  Modified just one MMCM for this.  Good! in_ratloop1 stuck LOW on Mezz#1, io_397 open at translator.
//        -- add MMCM for LHC_CK to get phase90, apply negedge 90 (= 270?) for ratloop LOUT registers
//  3.6-3.7:  special tests to debug Mezz#1 rawin_ratloop[1] (stuck low, bad ball joint @translator chip)
//  3.8:  enable ForceError on fiber for Tx output as well as on Rx input.  lhc_clk is DEAD on the bench!
//  3.9:  use qpll_ck40 to drive lhc_clk (not lhc_ck), now try to force Tx errors.  delete GTX LOC constraint from Ben's code
//  3.10err: ...send random data for 7-8 fibers, only rx for #1.  try removing comp_rx BUFGs
//  3.10: ...remove all GBE logic & unused logic & extra clocks!
//  3.11: fix error generator debounce, bring it to LED with slwclk too -- always errors from fiber, why?
//  3.12: make some fixes for forced error generation (shft_seq, pb_pulse, rnd_word) -- always errors from fiber, why?
//  3.13: initalize several registers properly related to ferr_z & triad_word
//  3.14: change settings inside MGT MMCM module
//  3.15: add logic to Reset MMCMs in sequence, add hold_bit for debounced !pb signal
//  3.16: took away most logic using ck125, added SRL16Es to Reset MMCMs at startup, changed LOCK logics, changed testLEDs.
//        -- add FCS=1 and float gtl_oe for Mezz 2012 compatibility, good for bench test use.
//  3.17: changed to bufg_x2div2 MMCM, watch the lock40 signal.  -- Looks good! ( <- only edit after last compile )
//  3.18: changed "test_led(1-4)" to inputs from connector, added "bc0" as input #1 to reset PRBS for fiber TX(1-7)
//       -- bc0 also toggles a register going to an LED.  Changed all LEDs & test_leds.
//  3.19: now handles high-true bc0(test_led1), ext_rst(test_led2) and ForceError(test_led3) inputs (was Low True)
//       -- assigned these to appropriate functions like reset & ferr_i 
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
module cscgem_emulator(
    input 	      ck125n, ck125p,
    input 	      ck160n, ck160p,
    input 	      lhc_ckn, lhc_ckp,
    input  [8:7]      sw,
    input 	      tmb_clock0, pb, vme_cmd10,
    input 	      qpll_lock,
    input  [8:1]      gempad_in,  // new inputs from CFEB Emulator board from "test LED" pins on connector
    input  [3:0]      vstat, // +1.5V TMB, Vcore RPC (driven via loopback), +3.3V TMB, +5V TMB
    input [28:1]      alct_rx,
    input 	      jtag_usr0_tdo, gp_io4, rpc_dsn, rpc_smbrx,
    input 	      prom_d3, prom_d7, jtag_fpga3, sda0, tmb_sn, t_crit,
    input [50:0]      _ccb_rx,  // add 42-47
    input [5:0]       dmb_rx,
    input [15:8]      dmb_i1tx,  // yes, input for loopback tests, only use on MODIFIED TMBs!
    input [31:24]     dmb_i2tx,  // yes, input for loopback tests, only use on MODIFIED TMBs!
    input [43:38]     dmb_i3tx,  // yes, input for loopback tests, only use on MODIFIED TMBs!
    input [9:0]       rpc_i1rx,
    input [37:26]     rpc_i2rx,
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
    output [26:0]     _ccb_tx,  // add 20-26
    output            _hard_reset_tmb_fpga, _hard_reset_alct_fpga,
    output 	      rst_qpll, fcs,
    output [3:0]      rpc_tx, // [3] is alct_tx29 on TMB!
    output [6:0]      vme_reply,
    output [4:0]      step, // step4 enables STEP mode for 3:0 = cfeb, rpc, dmb, alct.  step4 Low makes all free-running clocks.
    output reg [7:0]  led_low,
    output reg [15:8] led_hi,
    output reg [10:9] test_led,
    input 	      t12_fault, r12_fok,
    input  [7:1]      rxn, rxp,
    output [7:1]      txn, txp,
    output 	      t12_rst, t12_sclk, r12_sclk
   )/* synthesis syn_useioff = 1 */;


// snap12 GTX signals
   wire        synced_snapt, synced_snapr, all_tx_ready;
   wire        snap_clk2, ck160_locked, ck160_rst;
   wire        snap_wait;
//   wire [7:1]  check_ok_snapr, check_bad_snapr;
//   wire [7:1]  rxdv_diff, rxcomma_diff, rxdv_snapr, rxcomma_snapr;  // 
   wire [7:1]  tx_begin, tx_fc;
//   wire [7:1]  lgood_snapr, lbad_snapr, llost_snapr;
//   reg  [7:1]  rxdvr_snapr, rxcommar_snapr, check_okr_snapr, check_badr_snapr;
   reg  [7:1]  frand, ferr_f;
   reg [15:0]  err_count;
   reg  [7:0]  time_r_snap;
   reg  [7:0]  time_snap;
   wire  stopped, locked, lock40, dmbfifo_step1ck;

   parameter SEEDBASE = 64'h8731c6ef4a5b0d29;
   parameter SEEDSTEP = 16'hc01d;
   reg [5:0]  ireg;  // not used
   reg [22:0] free_count;
   reg 	      free_tc, free_tc_r;
   reg 	      slow_tc, slow_tc_r;
   reg [7:0]  time_count;
   reg [7:0]  time_40i;
   reg [7:0]  time_40r;
   wire  gtx_ready;
   wire [13:0] test_in;  // not used?
   reg 	       l_lock40, ck160_locklost, qpll_lock_lost;
   reg 	       bc0_r, bc0_rr, bc0_r3, bc0_led;
   wire        reset, gtx_reset;
   wire        ext_rst, force_err, bc0, stat0;  // stat[3:0] =  cfebemul_in[2,3,1,4] on Emul board!
   wire ck125, ck160, lhc_ck, lhc_clk, qpll_ck40, slwclk;   // ext 125 usr, QPLL160, ccb_ck40, QPLL40, ccb_ck40/25=1.6MHz
   wire lhc_ck0, lhc_ck90, lhc_ck180, lhc_ck270, lhcckfbout, lhcckfbout_buf, lhc_clk90;
   wire zero, one;
   wire [1:0] zero2;
   wire [31:0] zero32;
   
   wire [12:0] low;
   wire [3:0]  ignore;  // outputs from GTX we don't care about

// Add-ons for Ben dCFEB testing:
   wire    tx_clk_out, tx_clk;
   wire    rx_clk, rx_strt, rx_valid, rx_match, rx_fc;
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
   reg 	       ferr_i, ferr_r, ferr_done;

   //assign bc0 = cfebemul_in[1];    // clct_stat1
   //assign ext_rst = cfebemul_in[2];  // clct_stat3
   //assign force_err = cfebemul_in[3];  // clct_stat2
   //assign stat0 = cfebemul_in[4];  // ??
	assign bc0 = 1'b0;
	
	wire [15:0] geminfo[31:0];
	
	assign geminfo[0][15:0]=16'd0;
	//assign geminfo[1]=16'd6;
	assign geminfo[1][15:0]=16'd7;
	//assign geminfo[1]=16'd8;
	//assign geminfo[1]=16'd9;
	//assign geminfo[1]=16'd10;
	//assign geminfo[2]=16'd11;
	//assign geminfo[2]=16'd12;
	assign geminfo[2][15:0]=16'd13;
	//assign geminfo[2]=16'd14;
	//assign geminfo[2]=16'd15;
	//assign geminfo[3]=16'd16;
	//assign geminfo[3]=16'd17;
	assign geminfo[3][15:0]=16'd18;
	//assign geminfo[3]=16'd19;
	//assign geminfo[4]=16'd20;
	//assign geminfo[4]=16'd21;
	assign geminfo[4][15:0]=16'd22;
	//assign geminfo[4]=16'd23;
	//assign geminfo[4]=16'd24;
	//assign geminfo[5]=16'd25;
	//assign geminfo[5]=16'd26;
	assign geminfo[5][15:0]=16'd27;
	//assign geminfo[5]=16'd28;
	//assign geminfo[5]=16'd29;
	//assign geminfo[6]=16'd30;
	//assign geminfo[6]=16'd31;
	assign geminfo[6][15:0]=16'd32;
	//assign geminfo[6]=16'd33;
	//assign geminfo[6]=16'd34;
	//assign geminfo[7]=16'd35;
	//assign geminfo[7]=16'd36;
	assign geminfo[7][15:0]=16'd37;
	//assign geminfo[7]=16'd38;
	//assign geminfo[7]=16'd39;
	//assign geminfo[8]=16'd40;
	//assign geminfo[8]=16'd41;
	assign geminfo[8][15:0]=16'd42;
	//assign geminfo[8]=16'd43;
	//assign geminfo[8]=16'd44;
	//assign geminfo[9]=16'd45;
	//assign geminfo[9]=16'd46;
	assign geminfo[9][15:0]=16'd47;
	//assign geminfo[9]=16'd48;
	//assign geminfo[9]=16'd49;
	//assign geminfo[10]=16'd50;
	//assign geminfo[10]=16'd51;
	assign geminfo[10][15:0]=16'd52;
	//assign geminfo[10]=16'd53;
	//assign geminfo[10]=16'd54;
	//assign geminfo[11]=16'd55;
	//assign geminfo[11]=16'd56;
	assign geminfo[11][15:0]=16'd57;
	//assign geminfo[11]=16'd58;
	//assign geminfo[11]=16'd59;
	//assign geminfo[12]=16'd60;
	//assign geminfo[12]=16'd61;
	assign geminfo[12][15:0]=16'd62;
	//assign geminfo[12]=16'd63;
	//assign geminfo[12]=16'd64;
	//assign geminfo[13]=16'd65;
	//assign geminfo[13]=16'd66;
	assign geminfo[13][15:0]=16'd67;
	//assign geminfo[13]=16'd68;
	//assign geminfo[13]=16'd69;
	//assign geminfo[14]=16'd70;
	//assign geminfo[14]=16'd71;
	assign geminfo[14][15:0]=16'd72;
	//assign geminfo[14]=16'd73;
	//assign geminfo[14]=16'd74;
	//assign geminfo[15]=16'd75;
	//assign geminfo[15]=16'd76;
	assign geminfo[15][15:0]=16'd77;
	//assign geminfo[15]=16'd78;
	//assign geminfo[15]=16'd79;
	//assign geminfo[16]=16'd80;
	//assign geminfo[16]=16'd81;
	assign geminfo[16][15:0]=16'd82;
	//assign geminfo[16]=16'd83;
	//assign geminfo[16]=16'd84;
	//assign geminfo[17]=16'd85;
	//assign geminfo[17]=16'd86;
	assign geminfo[17][15:0]=16'd87;
	//assign geminfo[17]=16'd88;
	//assign geminfo[17]=16'd89;
	//assign geminfo[18]=16'd90;
	//assign geminfo[18]=16'd91;
	assign geminfo[18][15:0]=16'd92;
	//assign geminfo[18]=16'd93;
	//assign geminfo[18]=16'd94;
	//assign geminfo[19]=16'd95;
	//assign geminfo[19]=16'd96;
	assign geminfo[19][15:0]=16'd97;
	//assign geminfo[19]=16'd98;
	//assign geminfo[19]=16'd99;
	//assign geminfo[20]=16'd100;
	//assign geminfo[20]=16'd101;
	assign geminfo[20][15:0]=16'd102;
	//assign geminfo[20]=16'd103;
	//assign geminfo[20]=16'd104;
	//assign geminfo[21]=16'd105;
	//assign geminfo[21]=16'd106;
	assign geminfo[21][15:0]=16'd107;
	//assign geminfo[21]=16'd108;
	//assign geminfo[21]=16'd109;
	//assign geminfo[22]=16'd110;
	//assign geminfo[22]=16'd111;
	assign geminfo[22][15:0]=16'd112;
	//assign geminfo[22]=16'd113;
	//assign geminfo[23]=16'd114;
	//assign geminfo[23]=16'd115;
	assign geminfo[23][15:0]=16'd116;
	//assign geminfo[23]=16'd117;
	//assign geminfo[23]=16'd118;
	//assign geminfo[24]=16'd119;
	//assign geminfo[24]=16'd120;
	assign geminfo[24][15:0]=16'd121;
	//assign geminfo[24]=16'd122;
	//assign geminfo[24]=16'd123;
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
   assign rst_errcnt  = !_ccb_rx[29]; // TMB_SoftRst (tmb_res1), CCB base+6c or 6a
   assign in_pulse[0] = !_ccb_rx[11]; // crate BC0, CCB base+52
   assign in_pulse[1] = !_ccb_rx[12]; // L1A, CCB base+54
   assign in_pulse[2] = !_ccb_rx[29]; // TMB_SoftRst (tmb_res1), CCB base+6c or 6a
   assign in_pulse[3] = !_ccb_rx[32]; // clct_ext_trig, CCB base+86
   assign in_pulse[4] = !_ccb_rx[33]; // alct_ext_trig, CCB base+88
   assign in_pulse[5] = !_ccb_rx[39]; // dmb_cfeb_calib0, CCB base+8a
   assign in_pulse[6] = !_ccb_rx[40]; // dmb_cfeb_calib1, CCB base+8c
   assign in_pulse[7] = !_ccb_rx[41]; // dmb_cfeb_calib2, CCB base+8e
   assign in_pulse[8] = !_ccb_rx[27]; // alct_hard_reset_ccb, CCB base+66:  500 ns long pulse
   assign in_pulse[9] = !_ccb_rx[30]; // alct_adb_pulse_sync, CCB base+82:  500 ns long pulse
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
   assign dmbfifo_step1ck = dmb_rx[0];
   assign dmb_o2tx[33] = lout_dmbloop[0];
   assign rawin_dmbloop[0] = dmb_rx[1];
   assign dmb_o3tx[47] = lout_dmbloop[1];
   assign rawin_dmbloop[1] = dmb_rx[2];
   assign dmb_o3tx[48] = lout_dmbloop[2];
   assign rawin_dmbloop[2] = dmb_rx[3];
   assign dmb_o3tx[45] = lout_dmbloop[3];
   assign rawin_dmbloop[3] = dmb_rx[4];
   assign dmb_o3tx[46] = lout_dmbloop[4];
   assign rawin_dmbloop[4] = dmb_rx[5];

   assign dmb_tx[7:0] = lout_dmbloop[12:5];
   assign rawin_dmbloop[12:5] = {dmb_i1tx[11:8],dmb_i1tx[15:12]};
   assign dmb_o1tx[23:16] = lout_dmbloop[20:13];
   assign rawin_dmbloop[20:13] = {dmb_i2tx[27:24],dmb_i2tx[31:28]};
   assign dmb_o2tx[32] = lout_dmbloop[21];
   assign dmb_o2tx[37:34] = lout_dmbloop[25:22];
   assign dmb_o3tx[44] = lout_dmbloop[26];
   assign rawin_dmbloop[26:21] = {dmb_i3tx[43],dmb_i3tx[41:38],dmb_i3tx[42]};
// JGhere, RAT_Loopback signals:
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

   assign err_dmbloop[27] = lhc_tog_err; // JGhere, Fixed?  uses lhc_clk 40, but could use prbs data?
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
      gempad_r = 0;
      geminfo_r = 0;
      q=0;
      l_lock40 = 0;
      ck160_locklost = 0;
      time_40i = 8'h00;
      time_40r = 8'h00;
      free_tc_r = 1'b0;
      slow_tc_r = 1'b0;
      loop_count = 0;
      en_loopbacks = 1'b0;
      en_loopbacks_r = 0;
      en_loopbacks_rr = 0;
      en_cabletests_r = 0;
      en_cabletests = 1'b0;
      en_fibertests_r = 0;
      en_fibertests = 1'b0;
      llout_dmbloop = 0;
      lout_dmbloop = 0;
      in_dmbloop <= 0;
      dmbloop_errcnt = 0;
      dmbloop1_stat = 0;
      dmbloop2_stat = 0;
      dmbloop3_stat = 0;
      lhc_tog = 0;
      en_lhcloop = 0;
      en_lhcloop_r = 0;
      lhcloop_tog = 0;
      lhcloop_tog_r = 0;
      lout_ratloop = 0;
      loutpos_ratloop = 0;
      llout_ratloop = 0;
      in_ratloop = 0;
      ratloop_errcnt = 0;
      ratloop1_stat = 0;
      ratloop2_stat = 0;
      ratloop3_stat = 0;
      ratloop4_stat = 0;
      slowloop_count = 0;
      slowloop_err = 0;
      slowloop_stat = 0;
      lout_slowloop = 0;
      llout_slowloop = 0;
      in_slowloop = 0;
      slowloop_errcnt = 0;
      selusr = 4'b1101;
      for (i = 0; i < 27; i = i + 1) begin
	 init_dmbloop[i] = 39'd15 + 11401017*i;
      end
      for (i = 0; i < 47; i = i + 1) begin
	 init_ratloop[i] = 39'd68 + 1301017*i;
      end
      for (i = 0; i < 5; i = i + 1) begin
	 init_slowloop[i] = 39'd89 + 11901017*i;
      end
      shft_seq = 47'h000000000001;
      pb_pulse = 0;
      ferr_i = 0;
      ferr_r = 0;
      ferr_done = 0;
      triad_word = 0;
      triad_word_r = 0;
      triad_word_l1 = 0;
      triad_word_l2 = 0;
      triad_word_l3 = 0;
      triad_word_l4 = 0;
      triad_word_l5 = 0;
      triad_word_l6 = 0;
      triad_counter = 0;
      frand[7:1] = 7'h01;
      ferr_f[7:1] = 0;
      err_wait = 0;
      rnd_word = 0;
      debounced_bit = 0;
      hold_bit = 0;
      bc0_led = 0;
      bc0_r = 1;
      bc0_rr = 1;
      bc0_r3 = 1;
   end // initial begin


    genvar u;
    generate
       for (u=0; u<27; u=u+1) begin:prbs39dmbgen
	  prbs39_test dmb_loops(init_dmbloop[u], en_loopbacks_r, in_dmbloop[u], out_dmbloop[u], good_dmbloop[u], err_dmbloop[u], count_dmbloop[u], err_wait&rnd_word[u], (!locked)|reset, lhc_clk);
       end
    endgenerate

    genvar x;
    generate
       for (x=0; x<47; x=x+1) begin:prbs39ratgen
	  prbs39_test rat_loops(init_ratloop[x], en_loopbacks_r, in_ratloop[x], out_ratloop[x], good_ratloop[x], err_ratloop[x], count_ratloop[x], err_wait&rnd_word[x], (!locked)|reset, lhc_clk); // slwclk?
       end
    endgenerate

    genvar w;
    generate
       for (w=0; w<5; w=w+1) begin:prbs39slowgen
	  prbs39_test slow_loops(init_slowloop[w], en_loopbacks_r, in_slowloop[w], out_slowloop[w], good_slowloop[w], err_slowloop[w], count_slowloop[w], err_wait&rnd_word[w], (!locked)|reset, slwclk); // slwclk?
       end
    endgenerate


   BUFG lhcck(.I(tmb_clock0), .O(lhc_ck)); // only goes to mmcm now for 4-phase generation
   IBUFGDS #(.DIFF_TERM("TRUE"),.IOSTANDARD("LVDS_25"))  qpll40(.I(lhc_ckp) , .IB(lhc_ckn) , .O(qpll_ck40));
   IBUFGDS #(.DIFF_TERM("FALSE"),.IOSTANDARD("LVDS_25"))  clock125(.I(ck125p) , .IB(ck125n) , .O(ck125));
   IBUFDS_GTXE1  clock160(.I(ck160p) , .IB(ck160n) , .O(ck160), .ODIV2(), .CEB(zero));

   bufg_div8clk clk1p6(lhc_clk,!lhc_locked,slwclk,stopped,locked); // slwclk is now 1.6 MHz (was 5 MHz using ck125)

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
  BUFG lhcclkf_buf
   (.O (lhcckfbout_buf),
    .I (lhcckfbout));
  BUFG lhcclkout0_buf
   (.O   (lhc_clk),
    .I   (lhc_ck0));
  BUFG lhcclkout90_buf
   (.O   (lhc_clk90),
    .I   (lhc_ck90));


   assign gtx_ready = lock40 & ck160_locked & synced_snapr;
   assign reset = !_ccb_rx[1] | ext_rst | (!sw[7] & !pb);  // JGhere, this ccb_rx signal can screw up a bench test...
   assign gtx_reset = reset | !gtx_ready;


//   always @(posedge _ccb_rx[0] or posedge reset) begin
   always @(posedge ccb_cken or posedge reset) begin
      if(reset) begin
	 ccbrxzero_count <= 0;
      end
      else begin
	 ccbrxzero_count <= ccbrxzero_count + 1'b1; // count toggles on ccb_rx0
      end
   end // always @ (posedge _ccb_rx[0] or posedge reset)



// JGnew, there are many changes in the next ~30 lines
   reg  tx_sel;
   always @(posedge tx_clk or posedge gtx_reset) begin
      if(gtx_reset) begin
	 tx_sel <= 1'b1;
      end
      else begin
	 tx_sel <= ~tx_sel;
      end
   end
// assign trg_tx_data = rst_tx ? 32'h50BC50BC : (tx_sel ? out_data[47:16] : {out_data[15:0],frm_sep});
// assign trg_tx_isk  = rst_tx ?  4'b0101 :     (tx_sel ?               4'b0000     :  4'b0001);

   always @(posedge lhc_clk or posedge hold_bit) begin  // 80 MHz derived from GTX_TxPLL
      if(hold_bit) begin
	 geminfo_r <= 0;
	 q <= 0;
	 ferr_i <= 0;
	 ferr_r <= 0;
	 ferr_done <= 0;
	 triad_word <= 0;
	 triad_word_r <= 0;
	 triad_word_l1 <= 0;
	 triad_word_l2 <= 0;
	 triad_word_l3 <= 0;
	 triad_word_l4 <= 0;
	 triad_word_l5 <= 0;
	 triad_word_l6 <= 0;
	 triad_counter <= 0;
	 frand[7:1] <= 7'h01;
	 ferr_f[7:1] <= 0;
	 geminfo_r <= 0;  // JGnew
      end
      else begin   // syncing forced single-clock effects (debounced, for bit error or triad word)
	 //geminfo_r[15:0] <= geminfo[gempad_r][15:0];
	 //triad_word_r <= triad_word;
	 //ferr_i <= debounced_bit | force_err;	 // normally zero, force_err is ~25ns pulse
	 //if (!tx_sel) q <= {q[0], debounced_bit};
	 //ferr_r <= ferr_i;
	 //frand[7:1] <= {frand[6:1],frand[7]};
	 //if (!sw[8] && (q==2'b10)) triad_word <= {8'hAD, 9'h000, geminfo_r[6:0]}; // JGnew
	 //else triad_word <= 24'h000000;
	 //if (!ferr_done & ferr_r) begin  // begin the Forced Error sequence, sync with snap tx_clk
	 //   ferr_f[7:1] <= frand[7:1];  // set for exactly one tx_clk cycle
	 //   //triad_word_r <= geminfo_r[15:0];  // loaded for exactly one tx_clk cycle / this was changed to make triad word to equal geminfo
	 //end
	 //else begin  // end the Forced Error sequence when PB is released (ferr_i goes low)
	 //   ferr_f[7:1] <= 0;
	 //   //triad_word_r <= 0;
	 //end	
 	 //ferr_done <= ferr_r;
	 
	 triad_counter <= triad_counter + 1;

	 if (triad_counter == 64'b1000) begin
		 triad_word_l1 <= 8'b1000000;
		 triad_word_l2 <= 8'b1000000;
		 triad_word_l3 <= 8'b1000000;
		 triad_word_l4 <= 8'b1000000;
		 triad_word_l5 <= 8'b1000000;
		 triad_word_l6 <= 8'b1000000;
	 end
	 else if (triad_counter == 64'b1001) begin
		 triad_word_l1 <= 8'b1000000;
		 triad_word_l2 <= 8'b1000000;
		 triad_word_l3 <= 8'b1000000;
		 triad_word_l4 <= 8'b1000000;
		 triad_word_l5 <= 8'b1000000;
		 triad_word_l6 <= 8'b1000000;
	 end
	 else if (triad_counter == 64'b1010) begin
		 triad_word_l1 <= 8'b0;
		 triad_word_l2 <= 8'b0;
		 triad_word_l3 <= 8'b0;
		 triad_word_l4 <= 8'b0;
		 triad_word_l5 <= 8'b0;
		 triad_word_l6 <= 8'b0;
	 end
	 else begin
		 triad_word_l1 <= 8'b0;
		 triad_word_l2 <= 8'b0;
		 triad_word_l3 <= 8'b0;
		 triad_word_l4 <= 8'b0;
		 triad_word_l5 <= 8'b0;
		 triad_word_l6 <= 8'b0;
	 end

      end

   end // always @ (posedge tx_clk or posedge reset)


// JGhere, lhc_clk is DEAD on the bench!   use qpll_ck40 instead
// slwclk is DEAD on the bench!  will be fixed by switch to qpll_ck40
   always @(posedge slwclk or posedge reset) // everything that uses 1.6 MHz clock with simple Reset (from lhc_clk)
     begin
	if (reset) begin
	   free_count <= 0;
	   time_count <= 0;
	   free_tc <= 0;
	   slow_tc <= 0;
	   shft_seq <= 47'h000000000001;
	   rnd_word <= 0;
	   debounced_bit <= 0;
	   hold_bit <= 0;
	   err_wait <= 0;
	   pb_pulse <= 0;
	   slowloop_count <= 0;
	   slowloop_err <= 0;
	   slowloop_stat <= 0;
	   lout_slowloop <= 0;
	   llout_slowloop <= 0;
	   in_slowloop <= 0;
	   slowloop_errcnt <= 0;
	   gempad_r <= 0;  // JGnew
	end
	else begin
	   if (debounced_bit) gempad_r[4:0] <= gempad_in[5:1];

	   free_count <= free_count + 1'b1;
// JGhere, add slow loop checking logic:
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
	   time_40i <= 8'h00;
	   time_40r <= 8'h00;
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

	   if (time_40r[7] & lock40) l_lock40 <= 1;
	   if (l_lock40 & (!lock40)) ck160_locklost <= 1;
	   time_40i <= time_count;
	   time_40r <= time_40i;  // transfer slow slwclk time counter to lhc_clk domain
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
   

   always @(posedge rx_clk) // everything that uses 160 MHz snap USR clock, w/o Reset
     begin
	time_r_snap <= time_count;
	time_snap <= time_r_snap;  // transfer slow slwclk time counter to snap USR clock domain
     end


// JGhere, begin fiber Rx data handling section *7:
   assign push_fifo = ( word[0] && save_triad );
   always @(posedge rx_clk or posedge reset or posedge snap_wait) // things that uses 160 MHz snap USR clock, w/Reset
     begin
	if (reset | snap_wait) begin
//	   err_count <= 0;  // use this to count PRBS errors?
	   comp_dat_r  <= 0;
	   comp_dat_2r <= 0;
	   comp_dat_3r <= 0;
	   comp_dat_4r <= 0;
	   comp_dat_5r <= 0;
	   save_triad  <= 0;
	   itriad  <= 0;
	   rst_errcnt_r <= 1'b1;
	end

	else begin // Not Reset case
// set itriad=9  if ( !sw[8] && itriad==0 && |comp_dat_r > 0 )  can we pick a few specific bits?  TRIAD FORMAT!
// push FIFO if (word[0] && itriad > 0 )
	   rst_errcnt_r <= rst_errcnt;
	   if (!sw[8] && !save_triad && |nz && word[0]) begin  // 6-input logic
	      itriad <= 9;
	      save_triad <= 1;
	   end
	   else if (push_fifo) begin
	      itriad <= itriad - 1'b1;
	      if(itriad == 4'h1)save_triad <= 0;
	   end
	   if (word[0]) begin
	      comp_dat_r   <= comp_dat;
	      comp_dat_2r   <= comp_dat_r;
	      comp_dat_3r   <= comp_dat_2r;
	      comp_dat_4r   <= comp_dat_3r;
	   end
	   if (rst_errcnt_r) err_count <= 0;
	   else if ( !snap_wait & time_snap[7] ) begin  // wait 3000 clocks after Reset
	      if ( !rx_match & rx_valid ) err_count <= err_count + 1'b1;
	   end
        end
     end



/*  switch modes
   sw7 & sw8:   send err_count at 1.6 Hz; PB forces an error  (use w/Tx tester, OK at TAMU)
   !sw7 & sw8:  send err_count at 1.6 Hz; PB is Reset  (good to use at OSU & w/tester at TAMU)

   sw7 & !sw8:  send comp_dat when non-zero; PB forces a triad pattern  (for Tx tester at TAMU)
   !sw7 & !sw8: send comp_dat when non-zero; PB is Reset  (good to use at OSU & w/tester at TAMU)
*/
   always @(*)
     begin   // JG, v1p16: swap LED 3<>4, notes and all
	led_low[0] = !_ccb_rx[22]; // ccb_ttcrx_rdy, always OFF
	led_low[1] = !_ccb_rx[23]; // ccb_qpll_lck, OFF but often blinks ON (very short)
	led_low[2] = qpll_lock;    // always ON!   was !_ccb_rx[31] == _alct_adb_pulse_async
	led_low[3] = qpll_lock_lost; // always OFF?  was _ccb_rx[35] == mpc_in1, always ON
	led_low[4] = lhc_clk;
//	led_low[5] = !_ccb_rx[12];   // L1accept
//	led_low[6] = trigger;      // OR of several trigger pulse inputs
	led_low[7] = (reset);    // just reset, includes ccb_rx[1]==L1reset
//	led_low[0] = sw[8];  // on    to TMB FP LEDs.  is this LED[7:0]? Prom D[7:0] should control on-TMB LEDs.
//	led_low[1] = sw[7];  // off
//	led_low[2] = !pb;     // on w/pb.   change to reset?  can we use CCB_Resync?
//	led_low[4] = synced_snapr; // on  // == RX_SYNC_DONE
	led_low[5] = ck160_locked; // always ON!  // Tx GTX PLL Ref lock
	led_low[6] = !ck160_locklost; // --> ON!  comes from mmcm driven by tx_pll_ck160 in FPGA
//	led_low[7] = qpll_lock;    // on
	if (!sw[7]) begin
	   if (sw[8] == 1) begin // sw8 ON, 7 OFF:  send err_count at 1.6 Hz; PB is Reset  (good to use at OSU)
	      led_hi[8] = synced_snapr;  // Mezz1: HI
	      led_hi[9] = !gtx_reset; // M1: synced with errors at crate Rx.  Low for 12.6 usec, irregular, very common.
 	      led_hi[10] = gtx_ready; // M1: synced with errors at crate Rx.  Low for 12.6 usec, irregular, very common.
	      led_hi[11] = !reset;    // M1: HI
	      led_hi[12] = lhc_clk;   // M1: Good!
	      led_hi[13] = (locked & lhc_locked); // M1: 
	      led_hi[14] = slwclk;
	      led_hi[15] = !ck160_locklost;   // M1: 
	      //test_led[6:5] = 0;//
/*
	      test_led[1] = tx_begin;  // M1: always HI; Lo @RST.  CHECK.  best for !sw7 case
	      test_led[2] = qpll_ck40;      // M1: Good.
	      test_led[3] = tx_fc;     // M1: 3.2usec between 12.5ns pulses.   == LTNCY_TRIG: 199ns to rx_fc
	      test_led[4] = synced_snapr; // M1: always HI.  == RX_SYNC_DONE
	      test_led[5] = rx_strt;   // M1: always LO; 200/225ns pulse after RST.  CHECK.
	      test_led[6] = !ck160_locklost; // M1: always HI.  this comes from mmcm driven by tx_pll_ck160
	      test_led[7] = rx_valid;  // M1: always HI; Lo @RST.  CHECK.
	      test_led[8] = qpll_lock; // M1: always HI.    was snap_wait
*/
	      //test_led[7] = reset;   // 
	      //test_led[8] = bc0_led; // 
 	      test_led[9] = bc0;     // 
	      test_led[10]  = bc0_r3; // 
//	      led_low = 0;
	   end
// tx_begin (!sw7), tx_fc, rx_strt, rx_valid, rx_match

// JGhere, add 7 signals to 0,0 case: ccb_cmdstrb & lhc_clk & ccb_cmd[0], tmb_cfg_out, alct_cfg_out, ccb_datstrb & ccb_data[0]?
//                                    
	   else begin            // Both OFF: send comp_dout when non-zero; PB is Reset  (good to use at OSU)
	      led_hi[12:8] = ~gempad_r[4:0];
	      led_hi[13] = locked;    // M1: HI
	      led_hi[14] = slwclk;
	      led_hi[15] = lock40 & (q!=2'b01);    // M1: synced with errors at crate Rx.  Low for 12.6 usec, irregular, very common.
//	      test_led[2] = lhc_clk;        // * used to check lhc_clk freq.
	      //test_led[5] = 0;  // rx_strt;   // CHECK.  *happens while Rawin=HI w/big negative glitch (starts ~4.2ns before clock, lasts ~6ns)
	      //test_led[6] = 0;  // 
	      //test_led[7] = reset;   // 
	      //test_led[8] = bc0_led; // 
 	      test_led[9] = bc0;     // 
	      test_led[10]  = bc0_r3; // 

/*	      test_led[8:1]   = rxdvr_snap[7:0];
	      test_led[9] = snap_wait; //
 */
//	      led_low = 8'b11111111;   // Prom D[7:0], go to TMB LEDs
	   end
	end
	else if (sw[8] == 0) begin  // *** sw7 ON from here on: send comp_dout when non-zero; PB forces a triad pattern
	   //	                               (for Tx tester at TAMU)
// Mezz LEDs are inverted...TMB FP LEDs are not...test_leds are not.
	   led_hi[14:8] = geminfo_r[6:0];
	   led_hi[15] = !(ck160_locked & (q!=2'b01)); // comment
//	   test_led[1] = in_ratloop[13];   // push_fifo;  // 2nd.  9 ~7ns Hi pulses @25ns spacing   used at OSU
//	   test_led[2] = rawin_ratloop[13]; // save_triad; // 1st.  Hi for 225ns, starting ~20ns before PushFIFO  used at OSU
//	   test_led[3] = err_ratloop[13];  // send_triad; // 3rd.  Hi ~702ns, starting ~158ns after SaveTriad.   used at OSU
//	   test_led[1] = push_fifo;  // 2nd.  9 ~7ns Hi pulses @25ns spacing   used at OSU
	     // ^^  Add WIRE.  Trigger all the time!
//	   test_led[2] = save_triad; // 1st.  Hi for 225ns, starting ~20ns before PushFIFO   used at OSU
//	   test_led[3] = send_triad; // 3rd.  Hi ~702ns, starting ~158ns after SaveTriad.   used at OSU
	     // ^^  CHECK.  32ns low pulse every 730ns!
//	   test_led[4] = lhc_clk; // clk 40 from Mezz QPLL, CLEAN clk, random phase from LHC Clock
	   //test_led[5] = 0;
//	   test_led[5] = rx_strt;   // change?  LOW   used at OSU
//	   test_led[6] = no_comp_dav;  // * LOW    used at OSU
	   //test_led[6] = 0;
	   //test_led[7] = reset;   // 
	   //test_led[8] = bc0_led; // 
 	   test_led[9] = bc0;     // 
	   test_led[10]  = bc0_r3; // 
	end    // if (sw[8] == 0)
	else begin  // Both ON: send err_count at 1.6 Hz; PB forces an error (use w/Tx tester at TAMU)
//WHF switching these bits
//	      led_hi[8] = !ck160_rst;   // M1: 
//	      led_hi[9] = !(hold_bit | gtx_reset); // M1: synced with PB & errors at crate Rx
// 	      led_hi[10] = time_40r[7]; // M1: 
//	      led_hi[11] = synced_snapt;// M1: 
//	      led_hi[12] = !qpll_lock_lost;// M1: HI
//	      led_hi[13] = (locked & lhc_locked);  // M1: 
//	      led_hi[14] = lhc_ck;
//	      led_hi[15] = lock40;    //  M1: 
///////////
              led_hi[8] = ck160_rst;   // M1: 
              led_hi[9] = (hold_bit | gtx_reset); // M1: synced with PB & errors at crate Rx
              led_hi[10] = !time_40r[7]; // M1: 
              led_hi[11] = !synced_snapt;// M1: 
              led_hi[12] = qpll_lock_lost;// M1: HI
              led_hi[13] = !(locked & lhc_locked);  // M1: 
              led_hi[14] = !lhc_ck;
              led_hi[15] = !lock40;    //  M1: 
///////////

	      //test_led[6:5] = 0;  //
/*
	      test_led[1] = tx_begin;  //  CHECK.  best for !sw7 case
	      test_led[3] = tx_fc;     // CHECK.
*/
	      //test_led[7] = tx_begin[2];  // 
	      //test_led[8] = tx_fc[2]; // 
 	      test_led[9] = bc0;      // 
	      test_led[10]  = bc0_r3; // 
//	      led_low = 8'b11111111;  // Prom D[7:0], go to TMB LEDs
	end
     end


   SRL16E #(.INIT(16'h7FFF)) SRL16TXPLL(.Q(txpll_rst), .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b1), .CE (1'b1), .CLK(qpll_ck40), .D(1'b0));

   SRL16E #(.INIT(16'h0FFF)) SRL16MMCM(.Q(ck160_rst), .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b1), .CE (ck160_locked), .CLK(qpll_ck40), .D(1'b0));

// JGhere, snap12 GTX clock section, only need one?  6 of the 7 fibers have dummies for TRG_TxOutCLK.
//     mmcm from 80 MHz:                    In,    out80,   out160,     reset,       locked-TOGGLES!
//     MGT_JGUSRCLK_SOURCE_MMCM snap_mmcm (tx_clk_out, tx_clk, snap_clk2, ck160_rst, lock40); // from Tx GTX PLL out clk
//     mmcm from 80 MHz:         In,    out80,  out160,   out40,   reset,       locked
   bufg_x2div2 snap_mmcm (tx_clk_out, tx_clk, snap_clk2, ck40, !ck160_locked, lock40); // from Tx GTX PLL out clk

// JGhere, fiber Tx "from DCFEB" section:
//   was Snap12 module, not cfeb-tmb test code
   tmb_fiber_out  dcfeb1out (
	.RST (reset),   // Manual only
	.TRG_SIGDET (), // from IPAD to IBUF.  N/A?
	.TRG_RX_N (),   // empty
	.TRG_RX_P (),   // empty
	.TRG_TDIS (),   // OBUF output, for what?  N/A?
	.TRG_TX_N (txn[1]),   // pick a fiber
	.TRG_TX_P (txp[1]),   // pick a fiber
	.G1C (8'b0),  // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
	.G2C (8'b0),  //   if ENA_TEST_PAT then it's prbs so these don't matter...
	.G3C (8'b0), //   but good for testing low-rate non-zero triad data:
	.G4C (8'b0),
	.G5C (8'b0),
	.G6C (8'b0),
	.TRG_TX_REFCLK (ck160),  // QPLL 160 from MGT clk
	.TRG_TXUSRCLK (snap_clk2),  // get 160 from TXOUTCLK (times 2)
	.TRG_CLK80 (tx_clk),     // get 80 from TXOUTCLK
	.TRG_GTXTXRST (txpll_rst),   // maybe Manual "reset" only
	.TRG_TX_PLLRST (txpll_rst),  //  Tie LOW.
	.TRG_RST (gtx_reset),       // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
	.ENA_TEST_PAT (sw[8]),   // HIGH for PRBS!  (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
	.INJ_ERR (ferr_f[1] & sw[8]),  // use my switch/PB combo logic for this, high-true? Pulse high once.
	.TRG_SD (),          // from IBUF, useless output. N/A
	.TRG_TXOUTCLK (tx_clk_out),   // 80 MHz; This has to go to MCM to generate 160/80
	.TRG_TX_PLL_LOCK (ck160_locked), // inverse holds the MCM in Reset  // Tx GTX PLL Ref lock
	.TRG_TXRESETDONE (), // N/A
	.TX_SYNC_DONE (synced_snapt),    // not used in DCFEB tests
	.STRT_LTNCY (tx_begin[1]),  // after every Reset, to TP for debug only  -- !sw7 ?
	.LTNCY_TRIG (tx_fc[1]),     // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
	.MON_TX_SEL (),      //  N/A
	.MON_TRG_TX_ISK (),  //  N/A returns 4 bits
	.MON_TRG_TX_DATA ()  //  N/A returns 32 bits
	);

   tmb_fiber_out  dcfeb2out (
	.RST (reset),   // Manual only
	.TRG_SIGDET (), // from IPAD to IBUF.  N/A?
	.TRG_RX_N (),   // empty
	.TRG_RX_P (),   // empty
	.TRG_TDIS (),   // OBUF output, for what?  N/A?
	.TRG_TX_N (txn[2]),   // pick a fiber
	.TRG_TX_P (txp[2]),   // pick a fiber
//	.G1C (8'b0),  // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
//	.G2C (8'b0),  //   if ENA_TEST_PAT then it's prbs so these don't matter...
//	.G3C (8'b0), //   but good for testing low-rate non-zero triad data:
//	.G4C (8'b0),
//	.G5C (8'b0),
//	.G6C (8'b0),
	.G1C (triad_word_l1[7:0]),  // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
	.G2C (triad_word_l2[7:0]),  //   if ENA_TEST_PAT then it's prbs so these don't matter...
	.G3C (triad_word_l3[7:0]), //   but good for testing low-rate non-zero triad data:
	.G4C (triad_word_l4[7:0]),
	.G5C (triad_word_l5[7:0]),
	.G6C (triad_word_l6[7:0]),
	.TRG_TX_REFCLK (ck160),  // QPLL 160 from MGT clk
	.TRG_TXUSRCLK (snap_clk2),  // get 160 from TXOUTCLK (times 2)
	.TRG_CLK80 (tx_clk),     // get 80 from TXOUTCLK
	.TRG_GTXTXRST (0),   // maybe Manual "reset" only
	.TRG_TX_PLLRST (0),  //  Tie LOW.
	.TRG_RST (gtx_reset),       // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
	.ENA_TEST_PAT (sw[8]),   // HIGH for PRBS!  (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
	.INJ_ERR (ferr_f[2] & sw[8]),  // use my switch/PB combo logic for this, high-true? Pulse high once.
	.TRG_SD (),          // from IBUF, useless output. N/A
	.TRG_TXOUTCLK (),   // 80 MHz; This has to go to MCM to generate 160/80
	.TRG_TX_PLL_LOCK (), // inverse holds the MCM in Reset  // Tx GTX PLL Ref lock
	.TRG_TXRESETDONE (), // N/A
	.TX_SYNC_DONE (),    // not used in DCFEB tests
	.STRT_LTNCY (tx_begin[2]),  // after every Reset, to TP for debug only  -- !sw7 ?
	.LTNCY_TRIG (tx_fc[2]),     // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
	.MON_TX_SEL (),      //  N/A
	.MON_TRG_TX_ISK (),  //  N/A returns 4 bits
	.MON_TRG_TX_DATA ()  //  N/A returns 32 bits
	);

   tmb_fiber_out  dcfeb3out (
	.RST (reset),   // Manual only
	.TRG_SIGDET (), // from IPAD to IBUF.  N/A?
	.TRG_RX_N (),   // empty
	.TRG_RX_P (),   // empty
	.TRG_TDIS (),   // OBUF output, for what?  N/A?
	.TRG_TX_N (txn[3]),   // pick a fiber
	.TRG_TX_P (txp[3]),   // pick a fiber
	.G1C (8'b0),  // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
	.G2C (8'b0),  //   if ENA_TEST_PAT then it's prbs so these don't matter...
	.G3C (8'b0), //   but good for testing low-rate non-zero triad data:
	.G4C (8'b0),
	.G5C (8'b0),
	.G6C (8'b0),
	.TRG_TX_REFCLK (ck160),  // QPLL 160 from MGT clk
	.TRG_TXUSRCLK (snap_clk2),  // get 160 from TXOUTCLK (times 2)
	.TRG_CLK80 (tx_clk),     // get 80 from TXOUTCLK
	.TRG_GTXTXRST (0),   // maybe Manual "reset" only
	.TRG_TX_PLLRST (0),  //  Tie LOW.
	.TRG_RST (gtx_reset),       // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
	.ENA_TEST_PAT (sw[8]),   // HIGH for PRBS!  (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
	.INJ_ERR (ferr_f[3] & sw[8]),  // use my switch/PB combo logic for this, high-true? Pulse high once.
	.TRG_SD (),          // from IBUF, useless output. N/A
	.TRG_TXOUTCLK (),   // 80 MHz; This has to go to MCM to generate 160/80
	.TRG_TX_PLL_LOCK (), // inverse holds the MCM in Reset  // Tx GTX PLL Ref lock
	.TRG_TXRESETDONE (), // N/A
	.TX_SYNC_DONE (),    // not used in DCFEB tests
	.STRT_LTNCY (tx_begin[3]),  // after every Reset, to TP for debug only  -- !sw7 ?
	.LTNCY_TRIG (tx_fc[3]),     // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
	.MON_TX_SEL (),      //  N/A
	.MON_TRG_TX_ISK (),  //  N/A returns 4 bits
	.MON_TRG_TX_DATA ()  //  N/A returns 32 bits
	);

   tmb_fiber_out  dcfeb4out (
	.RST (reset),   // Manual only
	.TRG_SIGDET (), // from IPAD to IBUF.  N/A?
	.TRG_RX_N (),   // empty
	.TRG_RX_P (),   // empty
	.TRG_TDIS (),   // OBUF output, for what?  N/A?
	.TRG_TX_N (txn[4]),   // pick a fiber
	.TRG_TX_P (txp[4]),   // pick a fiber
//	.G1C (triad_word_l1[7:0]),  // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
//	.G2C (triad_word_l2[7:0]),  //   if ENA_TEST_PAT then it's prbs so these don't matter...
//	.G3C (triad_word_l3[7:0]), //   but good for testing low-rate non-zero triad data:
//	.G4C (triad_word_l4[7:0]),
//	.G5C (triad_word_l5[7:0]),
//	.G6C (triad_word_l6[7:0]),
	.G1C (8'b0),  // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
	.G2C (8'b0),  //   if ENA_TEST_PAT then it's prbs so these don't matter...
	.G3C (8'b0), //   but good for testing low-rate non-zero triad data:
	.G4C (8'b0),
	.G5C (8'b0),
	.G6C (8'b0),
	.TRG_TX_REFCLK (ck160),  // QPLL 160 from MGT clk
	.TRG_TXUSRCLK (snap_clk2),  // get 160 from TXOUTCLK (times 2)
	.TRG_CLK80 (tx_clk),     // get 80 from TXOUTCLK
	.TRG_GTXTXRST (0),   // maybe Manual "reset" only
	.TRG_TX_PLLRST (0),  //  Tie LOW.
	.TRG_RST (gtx_reset),       // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
	.ENA_TEST_PAT (sw[8]),   // HIGH for PRBS!  (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
	.INJ_ERR (ferr_f[4] & sw[8]),  // use my switch/PB combo logic for this, high-true? Pulse high once.
	.TRG_SD (),          // from IBUF, useless output. N/A
	.TRG_TXOUTCLK (),   // 80 MHz; This has to go to MCM to generate 160/80
	.TRG_TX_PLL_LOCK (), // inverse holds the MCM in Reset  // Tx GTX PLL Ref lock
	.TRG_TXRESETDONE (), // N/A
	.TX_SYNC_DONE (),    // not used in DCFEB tests
	.STRT_LTNCY (tx_begin[4]),  // after every Reset, to TP for debug only  -- !sw7 ?
	.LTNCY_TRIG (tx_fc[4]),     // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
	.MON_TX_SEL (),      //  N/A
	.MON_TRG_TX_ISK (),  //  N/A returns 4 bits
	.MON_TRG_TX_DATA ()  //  N/A returns 32 bits
	);

   tmb_fiber_out  dcfeb5out (
	.RST (reset),   // Manual only
	.TRG_SIGDET (), // from IPAD to IBUF.  N/A?
	.TRG_RX_N (),   // empty
	.TRG_RX_P (),   // empty
	.TRG_TDIS (),   // OBUF output, for what?  N/A?
	.TRG_TX_N (txn[5]),   // pick a fiber
	.TRG_TX_P (txp[5]),   // pick a fiber
	.G1C (8'b0),  // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
	.G2C (8'b0),  //   if ENA_TEST_PAT then it's prbs so these don't matter...
	.G3C (8'b0), //   but good for testing low-rate non-zero triad data:
	.G4C (8'b0),
	.G5C (8'b0),
	.G6C (8'b0),
	.TRG_TX_REFCLK (ck160),  // QPLL 160 from MGT clk
	.TRG_TXUSRCLK (snap_clk2),  // get 160 from TXOUTCLK (times 2)
	.TRG_CLK80 (tx_clk),     // get 80 from TXOUTCLK
	.TRG_GTXTXRST (0),   // maybe Manual "reset" only
	.TRG_TX_PLLRST (0),  //  Tie LOW.
	.TRG_RST (gtx_reset),       // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
	.ENA_TEST_PAT (sw[8]),   // HIGH for PRBS!  (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
	.INJ_ERR (ferr_f[5] & sw[8]),  // use my switch/PB combo logic for this, high-true? Pulse high once.
	.TRG_SD (),          // from IBUF, useless output. N/A
	.TRG_TXOUTCLK (),   // 80 MHz; This has to go to MCM to generate 160/80
	.TRG_TX_PLL_LOCK (), // inverse holds the MCM in Reset  // Tx GTX PLL Ref lock
	.TRG_TXRESETDONE (), // N/A
	.TX_SYNC_DONE (),    // not used in DCFEB tests
	.STRT_LTNCY (tx_begin[5]),  // after every Reset, to TP for debug only  -- !sw7 ?
	.LTNCY_TRIG (tx_fc[5]),     // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
	.MON_TX_SEL (),      //  N/A
	.MON_TRG_TX_ISK (),  //  N/A returns 4 bits
	.MON_TRG_TX_DATA ()  //  N/A returns 32 bits
	);

   tmb_fiber_out  dcfeb6out (
	.RST (reset),   // Manual only
	.TRG_SIGDET (), // from IPAD to IBUF.  N/A?
	.TRG_RX_N (),   // empty
	.TRG_RX_P (),   // empty
	.TRG_TDIS (),   // OBUF output, for what?  N/A?
	.TRG_TX_N (txn[6]),   // pick a fiber
	.TRG_TX_P (txp[6]),   // pick a fiber
	.G1C (8'b0),  // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
	.G2C (8'b0),  //   if ENA_TEST_PAT then it's prbs so these don't matter...
	.G3C (8'b0), //   but good for testing low-rate non-zero triad data:
	.G4C (8'b0),
	.G5C (8'b0),
	.G6C (8'b0),
	.TRG_TX_REFCLK (ck160),  // QPLL 160 from MGT clk
	.TRG_TXUSRCLK (snap_clk2),  // get 160 from TXOUTCLK (times 2)
	.TRG_CLK80 (tx_clk),     // get 80 from TXOUTCLK
	.TRG_GTXTXRST (0),   // maybe Manual "reset" only
	.TRG_TX_PLLRST (0),  //  Tie LOW.
	.TRG_RST (gtx_reset),       // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
	.ENA_TEST_PAT (sw[8]),   // HIGH for PRBS!  (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
	.INJ_ERR (ferr_f[6] & sw[8]),  // use my switch/PB combo logic for this, high-true? Pulse high once.
	.TRG_SD (),          // from IBUF, useless output. N/A
	.TRG_TXOUTCLK (),   // 80 MHz; This has to go to MCM to generate 160/80
	.TRG_TX_PLL_LOCK (), // inverse holds the MCM in Reset  // Tx GTX PLL Ref lock
	.TRG_TXRESETDONE (), // N/A
	.TX_SYNC_DONE (),    // not used in DCFEB tests
	.STRT_LTNCY (tx_begin[6]),  // after every Reset, to TP for debug only  -- !sw7 ?
	.LTNCY_TRIG (tx_fc[6]),     // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
	.MON_TX_SEL (),      //  N/A
	.MON_TRG_TX_ISK (),  //  N/A returns 4 bits
	.MON_TRG_TX_DATA ()  //  N/A returns 32 bits
	);

   tmb_fiber_out  dcfeb7out (
	.RST (reset),   // Manual only
	.TRG_SIGDET (), // from IPAD to IBUF.  N/A?
	.TRG_RX_N (),   // empty
	.TRG_RX_P (),   // empty
	.TRG_TDIS (),   // OBUF output, for what?  N/A?
	.TRG_TX_N (txn[7]),   // pick a fiber
	.TRG_TX_P (txp[7]),   // pick a fiber
	.G1C (8'b0),  // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
	.G2C (8'b0),  //   if ENA_TEST_PAT then it's prbs so these don't matter...
	.G3C (8'b0), //   but good for testing low-rate non-zero triad data:
	.G4C (8'b0),
	.G5C (8'b0),
	.G6C (8'b0),
	.TRG_TX_REFCLK (ck160),  // QPLL 160 from MGT clk
	.TRG_TXUSRCLK (snap_clk2),  // get 160 from TXOUTCLK (times 2)
	.TRG_CLK80 (tx_clk),     // get 80 from TXOUTCLK
	.TRG_GTXTXRST (0),   // maybe Manual "reset" only
	.TRG_TX_PLLRST (0),  //  Tie LOW.
	.TRG_RST (gtx_reset),       // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
	.ENA_TEST_PAT (sw[8]),   // HIGH for PRBS!  (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
	.INJ_ERR (ferr_f[7] & sw[8]),  // use my switch/PB combo logic for this, high-true? Pulse high once.
	.TRG_SD (),          // from IBUF, useless output. N/A
	.TRG_TXOUTCLK (),   // 80 MHz; This has to go to MCM to generate 160/80
	.TRG_TX_PLL_LOCK (), // inverse holds the MCM in Reset  // Tx GTX PLL Ref lock
	.TRG_TXRESETDONE (), // N/A
	.TX_SYNC_DONE (),    // not used in DCFEB tests
	.STRT_LTNCY (tx_begin[7]),  // after every Reset, to TP for debug only  -- !sw7 ?
	.LTNCY_TRIG (tx_fc[7]),     // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
	.MON_TX_SEL (),      //  N/A
	.MON_TRG_TX_ISK (),  //  N/A returns 4 bits
	.MON_TRG_TX_DATA ()  //  N/A returns 32 bits
	);

// JGhere, begin fiber Rx input section:
   comp_fiber_in dcfeb_in (
	.CMP_RX_VIO_CNTRL (), // empty or Delete from code
	.CMP_RX_LA_CNTRL (),  // empty or Delete from code
	.RST (gtx_reset),     // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
        .CMP_SIGDET (),       //  N/A
        .CMP_RX_N (rxn[1]),   // pick a fiber
        .CMP_RX_P (rxp[1]),   // pick a fiber
	.CMP_TDIS (),      //  N/A
	.CMP_TX_N (),      // empty
	.CMP_TX_P (),      // empty
        .CMP_RX_REFCLK (ck160), // QPLL 160 via GTX Clock
        .CMP_SD (),        // from IBUF, useless output. N/A
	.CMP_RX_CLK160 (rx_clk), //*7 Rx recovered clock out.  Use for now as Logic Fabric clock
			           //   Needed to sync all 7 CFEBs with Fabric clock!
	.STRT_MTCH (rx_strt), //*7 gets set when the Start Pattern is present, N/A for Comp data.  To TP for debug.  --sw8,7
	.VALID (rx_valid),    //*7 send this output to TP (only valid after StartMtch has come by)
	.MATCH (rx_match),    //*7 send this output to TP  AND use for counting errors
			           // VALID="should match" when true, !MATCH is an error
        .RCV_DATA (comp_dat),  //*7 48 bit comp. data output, send to GbE if |48  > 0
		 	 //  keep 3 48-bit words now,  3 48-bit words before,  plus 3 48-bit words after
        .NONZERO_WORD (nz),
        .CEW0 (word[0]),     //*7 access four phases of 40 MHz cycle...frame sep. out from GTX
        .CEW1 (word[1]),
        .CEW2 (word[2]),
        .CEW3 (word[3]),     //*7 on CEW3_r (== CEW3 + 1) the RCV_DATA is valid, use to clock into pipeline
        .LTNCY_TRIG (rx_fc), //*7 flags when RX sees "FC" for latency msm't.  Send raw to TP  --sw8,7
        .RX_SYNC_DONE (synced_snapr) //*7  use inverse of this as Reset for GTXs etc?
	);

// JGhere, begin fiber Rx data storage section *7:
   fifo_48b_256_async comp_fifo (
	.rst    (reset),
	.wr_clk (rx_clk),        //*7
	.rd_clk (ck125),
	.din    (comp_dat_4r),   //*7
	.wr_en  (push_fifo),     //*7
	.rd_en  (JGhere),        //*7   The data needs to go someplace!
	.dout   (comp_dout),     //*7
	.full   (comp_overflow), //*7
	.empty  (no_comp_dav)    //*7
	);

   assign snap_wait = !(synced_snapr & ck160_locked);  // allows pattern checks when RX is ready

endmodule

// file: bufg_div8clk.v
// 
// (c) Copyright 2008 - 2011 Xilinx, Inc. All rights reserved.
// 
// This file contains confidential and proprietary information
// of Xilinx, Inc. and is protected under U.S. and
// international copyright and other intellectual property
// laws.
// 
// DISCLAIMER
// This disclaimer is not a license and does not grant any
// rights to the materials distributed herewith. Except as
// otherwise provided in a valid license issued to you by
// Xilinx, and to the maximum extent permitted by applicable
// law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
// WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
// AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
// BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
// INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
// (2) Xilinx shall not be liable (whether in contract or tort,
// including negligence, or under any other theory of
// liability) for any loss or damage of any kind or nature
// related to, arising under or in connection with these
// materials, including for any direct, or any indirect,
// special, incidental, or consequential loss or damage
// (including loss of data, profits, goodwill, or any type of
// loss or damage suffered as a result of any action brought
// by a third party) even if such damage or loss was
// reasonably foreseeable or Xilinx had been advised of the
// possibility of the same.
// 
// CRITICAL APPLICATIONS
// Xilinx products are not designed or intended to be fail-
// safe, or for use in any application requiring fail-safe
// performance, such as life-support or safety devices or
// systems, Class III medical devices, nuclear facilities,
// applications related to the deployment of airbags, or any
// other applications that could lead to death, personal
// injury, or severe property or environmental damage
// (individually and collectively, "Critical
// Applications"). Customer assumes the sole risk and
// liability of any use of Xilinx products in Critical
// Applications, subject only to applicable laws and
// regulations governing limitations on product liability.
// 
// THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
// PART OF THIS FILE AT ALL TIMES.
// 
//----------------------------------------------------------------------------
// User entered comments
//----------------------------------------------------------------------------
// None
//
//----------------------------------------------------------------------------
// "Output    Output      Phase     Duty      Pk-to-Pk        Phase"
// "Clock    Freq (MHz) (degrees) Cycle (%) Jitter (ps)  Error (ps)"
//----------------------------------------------------------------------------
// CLK_OUT1_____5.000______0.000______50.0______442.863____179.338
//
//----------------------------------------------------------------------------
// "Input Clock   Freq (MHz)    Input Jitter (UI)"
//----------------------------------------------------------------------------
// __primary__________40.000____________0.010

`timescale 1ps/1ps

(* CORE_GENERATION_INFO = "bufg_div8clk,clk_wiz_v3_6,{component_name=bufg_div8clk,use_phase_alignment=true,use_min_o_jitter=false,use_max_i_jitter=false,use_dyn_phase_shift=false,use_inclk_switchover=false,use_dyn_reconfig=false,feedback_source=FDBK_AUTO,primtype_sel=MMCM_ADV,num_out_clk=1,clkin1_period=25.000,clkin2_period=10.0,use_power_down=false,use_reset=true,use_locked=true,use_inclk_stopped=true,use_status=false,use_freeze=false,use_clk_valid=false,feedback_type=SINGLE,clock_mgr_type=MANUAL,manual_override=false}" *)
module bufg_div8clk
 (// Clock in ports
  input         CLK_IN1,
  input         RESET,
  // Clock out ports
  output        CLK_OUT1,
  // Status and control signals
  output        INPUT_CLK_STOPPED,
  output        LOCKED
 );

  // Input buffering
  //------------------------------------
  BUFG clkin1_buf
   (.O (clkin1),
    .I (CLK_IN1));


  // Clocking primitive
  //------------------------------------
  // Instantiation of the MMCM primitive
  //    * Unused inputs are tied off
  //    * Unused outputs are labeled unused
  wire [15:0] do_unused;
  wire        drdy_unused;
  wire        psdone_unused;
  wire        clkfbout;
  wire        clkfbout_buf;
  wire        clkfboutb_unused;
  wire        clkout0b_unused;
  wire        clkout1_unused;
  wire        clkout1b_unused;
  wire        clkout2_unused;
  wire        clkout2b_unused;
  wire        clkout3_unused;
  wire        clkout3b_unused;
  wire        clkout4_unused;
  wire        clkout5_unused;
  wire        clkout6_unused;
  wire        clkfbstopped_unused;

  MMCM_ADV
  #(.BANDWIDTH            ("OPTIMIZED"),
    .CLKOUT4_CASCADE      ("FALSE"),
    .CLOCK_HOLD           ("FALSE"),
    .COMPENSATION         ("ZHOLD"),
    .STARTUP_WAIT         ("FALSE"),
    .DIVCLK_DIVIDE        (1),
    .CLKFBOUT_MULT_F      (16.000),
    .CLKFBOUT_PHASE       (0.000),
    .CLKFBOUT_USE_FINE_PS ("FALSE"),
    .CLKOUT0_DIVIDE_F     (128.000),
    .CLKOUT0_PHASE        (0.000),
    .CLKOUT0_DUTY_CYCLE   (0.500),
    .CLKOUT0_USE_FINE_PS  ("FALSE"),
    .CLKIN1_PERIOD        (25.000),
    .REF_JITTER1          (0.010))
  mmcm_adv_inst
    // Output clocks
   (.CLKFBOUT            (clkfbout),
    .CLKFBOUTB           (clkfboutb_unused),
    .CLKOUT0             (clkout0),
    .CLKOUT0B            (clkout0b_unused),
    .CLKOUT1             (clkout1_unused),
    .CLKOUT1B            (clkout1b_unused),
    .CLKOUT2             (clkout2_unused),
    .CLKOUT2B            (clkout2b_unused),
    .CLKOUT3             (clkout3_unused),
    .CLKOUT3B            (clkout3b_unused),
    .CLKOUT4             (clkout4_unused),
    .CLKOUT5             (clkout5_unused),
    .CLKOUT6             (clkout6_unused),
     // Input clock control
    .CLKFBIN             (clkfbout_buf),
    .CLKIN1              (clkin1),
    .CLKIN2              (1'b0),
     // Tied to always select the primary input clock
    .CLKINSEL            (1'b1),
    // Ports for dynamic reconfiguration
    .DADDR               (7'h0),
    .DCLK                (1'b0),
    .DEN                 (1'b0),
    .DI                  (16'h0),
    .DO                  (do_unused),
    .DRDY                (drdy_unused),
    .DWE                 (1'b0),
    // Ports for dynamic phase shift
    .PSCLK               (1'b0),
    .PSEN                (1'b0),
    .PSINCDEC            (1'b0),
    .PSDONE              (psdone_unused),
    // Other control and status signals
    .LOCKED              (LOCKED),
    .CLKINSTOPPED        (INPUT_CLK_STOPPED),
    .CLKFBSTOPPED        (clkfbstopped_unused),
    .PWRDWN              (1'b0),
    .RST                 (RESET));

  // Output buffering
  //-----------------------------------
  BUFG clkf_buf
   (.O (clkfbout_buf),
    .I (clkfbout));

  BUFG clkout1_buf
   (.O   (CLK_OUT1),
    .I   (clkout0));




endmodule




///////////////////////////////////////////////////////////////////////////////
//   ____  ____
//  /   /\/   / 
// /___/  \  /    Vendor: Xilinx 
// \   \   \/     Version : 1.8
//  \   \         Application :  Virtex-6 FPGA GTX Transceiver Wizard
//  /   /         Filename : mgt_jgusrclk_source_mmcm.v
// /___/   /\     Timestamp : 
// \   \  /  \ 
//  \___\/\___\ 
//
//
// Module MGT_JGUSRCLK_SOURCE (for use with GTX Transceivers)
// Generated by Xilinx Virtex-6 FPGA GTX Transceiver Wizard
// 
// 
// (c) Copyright 2009-2010 Xilinx, Inc. All rights reserved.
// 
// This file contains confidential and proprietary information
// of Xilinx, Inc. and is protected under U.S. and
// international copyright and other intellectual property
// laws.
// 
// DISCLAIMER
// This disclaimer is not a license and does not grant any
// rights to the materials distributed herewith. Except as
// otherwise provided in a valid license issued to you by
// Xilinx, and to the maximum extent permitted by applicable
// law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
// WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
// AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
// BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
// INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
// (2) Xilinx shall not be liable (whether in contract or tort,
// including negligence, or under any other theory of
// liability) for any loss or damage of any kind or nature
// related to, arising under or in connection with these
// materials, including for any direct, or any indirect,
// special, incidental, or consequential loss or damage
// (including loss of data, profits, goodwill, or any type of
// loss or damage suffered as a result of any action brought
// by a third party) even if such damage or loss was
// reasonably foreseeable or Xilinx had been advised of the
// possibility of the same.
// 
// CRITICAL APPLICATIONS
// Xilinx products are not designed or intended to be fail-
// safe, or for use in any application requiring fail-safe
// performance, such as life-support or safety devices or
// systems, Class III medical devices, nuclear facilities,
// applications related to the deployment of airbags, or any
// other applications that could lead to death, personal
// injury, or severe property or environmental damage
// (individually and collectively, "Critical
// Applications"). Customer assumes the sole risk and
// liability of any use of Xilinx products in Critical
// Applications, subject only to applicable laws and
// regulations governing limitations on product liability.
// 
// THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
// PART OF THIS FILE AT ALL TIMES. 


`timescale 1ns / 1ps

//***********************************Entity Declaration*******************************
module MGT_JGUSRCLK_SOURCE_MMCM #
(
    parameter   MULT            =   8.0,
    parameter   DIVIDE          =   1,
    parameter   CLK_PERIOD      =   12.5,
    parameter   OUT0_DIVIDE     =   8.0,
    parameter   OUT1_DIVIDE     =   4
)
(
    input           CLK_IN,
    output          CLK0_OUT,
    output          CLK1_OUT,
    input           MMCM_RESET_IN,
    output          MMCM_LOCKED_OUT
);


`define DLY #1

//*********************************Wire Declarations**********************************

    wire    [15:0]  tied_to_ground_vec_i;
    wire            tied_to_ground_i;
    wire            clkout0_i;
    wire            clkout1_i;
    wire            clkfbout_i;

//*********************************** Beginning of Code *******************************

    //  Static signal Assigments    
    assign tied_to_ground_i             = 1'b0;
    assign tied_to_ground_vec_i         = 16'h0000;

    // Instantiate a MMCM module to divide the reference clock. Uses internal feedback
    // for improved jitter performance, and to avoid consuming an additional BUFG
    MMCM_ADV #
    (
         .COMPENSATION      ("ZHOLD"),
         .CLKFBOUT_MULT_F   (MULT),
         .DIVCLK_DIVIDE     (DIVIDE),
         .CLKFBOUT_PHASE    (0),
         .CLKIN1_PERIOD     (CLK_PERIOD),
         .CLKOUT0_DIVIDE_F  (OUT0_DIVIDE),
         .CLKOUT0_PHASE     (0),
         .CLKOUT1_DIVIDE    (OUT1_DIVIDE),
         .CLKOUT1_PHASE     (0),
         .CLOCK_HOLD        ("FALSE")
    )
    mmcm_adv_i   
    (
         .CLKIN1            (CLK_IN),
         .CLKIN2            (1'b0),
         .CLKINSEL          (1'b1),
         .CLKFBIN           (clkfbout_i),
         .CLKOUT0           (clkout0_i),
         .CLKOUT0B          (),
         .CLKOUT1           (clkout1_i),
         .CLKOUT1B          (),         
         .CLKOUT2           (),
         .CLKOUT2B          (),         
         .CLKOUT3           (),
         .CLKOUT3B          (),         
         .CLKOUT4           (),
         .CLKOUT5           (),
         .CLKOUT6           (),
         .CLKFBOUT          (clkfbout_i),
         .CLKFBOUTB         (),
         .CLKFBSTOPPED      (),
         .CLKINSTOPPED      (),
         .DO                (),
         .DRDY              (),
         .DADDR             (7'd0),
         .DCLK              (1'b0),
         .DEN               (1'b0),
         .DI                (16'd0),
         .DWE               (1'b0),
         .LOCKED            (MMCM_LOCKED_OUT),
         .PSCLK             (1'b0),
         .PSEN              (1'b0),         
         .PSINCDEC          (1'b0), 
         .PSDONE            (),         
         .PWRDWN            (1'b0),         
         .RST               (MMCM_RESET_IN)
    );
    
    BUFG clkout0_bufg_i  
    ( .O (CLK0_OUT), 
      .I (clkout0_i) ); 

    BUFG clkout1_bufg_i
    ( .O (CLK1_OUT),
      .I (clkout1_i) );

endmodule




///////////////////////////////////////////////////////////////////////////////
//   ____  ____
//  /   /\/   /
// /___/  \  /    Vendor: Xilinx 
// \   \   \/     Version : 1.8
//  \   \         Application : Virtex-6 FPGA GTX Transceiver Wizard
//  /   /         Filename : double_reset.v
// /___/   /\           --workaround from core_project examples--
// \   \  /  \ 
//  \___\/\___\
//
//
// Module DOUBLE_RESET
// Generated by Xilinx Virtex-6 FPGA GTX Transceiver Wizard
// 
`define DLY #1

//    --workaround from core_project examples--
module DOUBLE_RESET
(
	input  	CLK,
	input  	PLLLKDET,
        output  GTXTEST_DONE,
	output 	GTXTEST_BIT1
);

   reg        	    plllkdet_sync;
   reg              plllkdet_r;
   reg 	  [10:0]    reset_dly_ctr;
   reg        	    reset_dly_done;
   reg    [3:0]	    testdone_f;


	always @(posedge CLK)
	begin
   	  plllkdet_r    	<= `DLY PLLLKDET;
   	  plllkdet_sync 	<= `DLY plllkdet_r;
   	end

	assign GTXTEST_BIT1  = reset_dly_done; 
        assign GTXTEST_DONE  = (reset_dly_ctr == 11'd0)? testdone_f[0] : 1'b0;

	always @(posedge CLK)
        begin
    	   if (!plllkdet_sync) 
              reset_dly_ctr 	<= `DLY 11'h7FF;
    	   else if (reset_dly_ctr != 11'h000)
              reset_dly_ctr 	<= `DLY reset_dly_ctr - 1'b1;
        end

	always @(posedge CLK)
        begin
    	   if (!plllkdet_sync) 
              reset_dly_done 	<= `DLY 1'b0;
    	   else if (reset_dly_ctr[10] == 1'b0) 
              reset_dly_done 	<= `DLY reset_dly_ctr[8];
        end

	always @(posedge CLK)
        begin
     	   if (reset_dly_ctr != 11'd0)
       	      testdone_f     	<= `DLY 4'b1111;
           else
              testdone_f     	<= `DLY {1'b0, testdone_f[3:1]};
        end

endmodule



///////////////////////////////////////////////////////////////////////////////
//   ____  ____
//  /   /\/   /
// /___/  \  /    Vendor: Xilinx 
// \   \   \/     Version : 1.8
//  \   \         Application : Virtex-6 FPGA GTX Transceiver Wizard
//  /   /         Filename : tx_sync.v
// /___/   /\     Timestamp : 
// \   \  /  \ 
//  \___\/\___\
//
//
// Module TX_SYNC
// Generated by Xilinx Virtex-6 FPGA GTX Transceiver Wizard
// (c) Copyright 2009-2010 Xilinx, Inc. All rights reserved.
`timescale 1ns / 1ps
`define DLY #1

module TX_SYNC #
(
    parameter       SIM_TXPMASETPHASE_SPEEDUP   = 0
)
(
    output          TXENPMAPHASEALIGN,
    output          TXPMASETPHASE,
    output          TXDLYALIGNDISABLE,
    output          TXDLYALIGNRESET,
    output          SYNC_DONE,
    input           USER_CLK,
    input           RESET
);


//*******************************Register Declarations************************

    reg            begin_r;
    reg            phase_align_r;
    reg            ready_r;
    reg   [15:0]   sync_counter_r;
    reg   [5:0]    wait_before_setphase_counter_r;
    reg   [4:0]    align_reset_counter_r;
    reg            align_reset_r;
    reg            wait_before_setphase_r;
    
//*******************************Wire Declarations****************************
    
    wire           count_setphase_complete_r;
    wire           count_32_complete_r;
    wire           count_align_reset_complete_r;
    wire           next_phase_align_c;
    wire           next_ready_c;
    wire           next_align_reset_c;
    wire           next_wait_before_setphase_c;

//*******************************Main Body of Code****************************

    //________________________________ State machine __________________________
    // This state machine manages the TX phase alignment procedure of the GTX.
    // The module is held in reset till TXRESETDONE is asserted. Once TXRESETDONE 
    // is asserted, the state machine goes into the align_reset_r state, asserting
    // TXDLYALIGNRESET for 20 TXUSRCLK2 cycles. After this, it goes into the 
    // wait_before_setphase_r state for 32 cycles. After asserting TXENPMAPHASEALIGN and 
    // waiting 32 cycles, it goes into the phase_align_r state where the last 
    // part of the alignment procedure is completed. This involves asserting 
    // TXPMASETPHASE for 8192 (TXPLL_DIVSEL_OUT=1), 16384 (TXPLL_DIVSEL_OUT=2), 
    // or 32768 (TXPLL_DIVSEL_OUT=4) clock cycles. After completion of the phase 
    // alignment procedure, TXDLYALIGNDISABLE is deasserted.
    
    // State registers
    always @(posedge USER_CLK)
        if(RESET)
            {begin_r,align_reset_r,wait_before_setphase_r,phase_align_r,ready_r}  <=  `DLY    5'b10000;
        else
        begin
            begin_r                <=  `DLY    1'b0;
            align_reset_r          <=  `DLY    next_align_reset_c;
            wait_before_setphase_r <=  `DLY    next_wait_before_setphase_c;
            phase_align_r          <=  `DLY    next_phase_align_c;
            ready_r                <=  `DLY    next_ready_c;
        end

    // Next state logic    
    assign  next_align_reset_c          =   begin_r |
                                            (align_reset_r & !count_align_reset_complete_r);
    
    assign  next_wait_before_setphase_c =   (align_reset_r & count_align_reset_complete_r) |
                                            (wait_before_setphase_r & !count_32_complete_r);
                                        
    assign  next_phase_align_c          =   (wait_before_setphase_r & count_32_complete_r) |
                                            (phase_align_r & !count_setphase_complete_r);
                                        
    assign  next_ready_c                =   (phase_align_r & count_setphase_complete_r) |
                                            ready_r;

    //______ Counter for holding TXDLYALIGNRESET for 20 TXUSRCLK2 cycles ______
    always @(posedge USER_CLK)
    begin
        if (!align_reset_r)
            align_reset_counter_r <= `DLY 5'b00000;
        else
            align_reset_counter_r <= `DLY align_reset_counter_r +1'b1;
    end
    
    assign count_align_reset_complete_r = align_reset_counter_r[4]
                                        & align_reset_counter_r[2];

    //_______ Counter for waiting 32 clock cycles before TXPMASETPHASE ________
    always @(posedge USER_CLK)
    begin
        if (!wait_before_setphase_r)
           wait_before_setphase_counter_r  <= `DLY  6'b000000;
        else
           wait_before_setphase_counter_r  <= `DLY  wait_before_setphase_counter_r + 1'b1;
    end

    assign count_32_complete_r = wait_before_setphase_counter_r[5];

    //_______________ Counter for holding SYNC for SYNC_CYCLES ________________
    always @(posedge USER_CLK)
    begin
        if (!phase_align_r)
            sync_counter_r <= `DLY  16'h0000;
        else
            sync_counter_r <= `DLY  sync_counter_r + 1'b1;
    end

generate
if(SIM_TXPMASETPHASE_SPEEDUP==1)
begin:fast_simulation
    // 64 cycles of setphase for simulation 
    assign count_setphase_complete_r = sync_counter_r[6];
end
else
begin:no_fast_simulation
    // 8192 cycles of setphase for output divider of 1
    assign count_setphase_complete_r = sync_counter_r[13];
end
endgenerate

    //_______________ Assign the phase align ports into the GTX _______________

    assign TXDLYALIGNRESET   = align_reset_r;
    assign TXENPMAPHASEALIGN = !begin_r & !align_reset_r;
    assign TXPMASETPHASE     = phase_align_r;
    assign TXDLYALIGNDISABLE = !ready_r;

    //_______________________ Assign the sync_done port _______________________
    
    assign SYNC_DONE = ready_r;
    
    
endmodule



//
// PRBS39_TEST
//    J. Gilmore, 4/20/11
//-----------------------------------------------------------------------------
// 39-bit PRBS module, refs:
//   http://www.eng.auburn.edu/~strouce/polycorrect2.pdf
//   http://www.xilinx.com/support/documentation/application_notes/xapp052.pdf
//   http://www.eecs.berkeley.edu/~newton/Classes/CS150sp97/labs/lab5/lab5.html#primPolynomials
//   http://en.wikipedia.org/wiki/Linear_feedback_shift_register
//   http://www.springerlink.com/content/u590n42r72568803/
// Note that for any primitive polynomial, the reverse polynomial is also primitive!
//-----------------------------------------------------------------------------

module prbs39_test(
	input [38:0] init_dat,  // ==0 will disable the circuit (set constant)
	input        en,    // en allows data to go out and checks on the return
	input        in,    // this is the data bit received, Tpd < 10 ns?  Fmax=50Mhz
	output       out,   // this is the data bit to send out, on pos-clk
	output reg   good, err,   // these indicate a match/mismatch was detected
	output reg [15:0]  count, // this is a 16-bit counter for # of errors
        input  force_error,  // this needs to trigger a  _single_  bit-flip
	input        rst,
	input        clk);  // assume 10-20 MHz operation...NOW 40 Mhz!  later 80?

        wire 	    rawout;
        reg 	    in_r, out_r, out_rr, out_3r, ferr_r, ferr_rr, ferr_done;
        reg  [3:0]  en_r;

        prbs39 randombit(init_dat, en, rawout, rst, clk);
        assign out = rawout^ferr_rr;

	always @(posedge clk or posedge rst) begin
	   if(rst) begin
	      en_r <= 0;
	      count  <= 0;
	      ferr_r <= 0;
	      ferr_rr <= 0;	      
	      in_r <= 0;
	      out_r <= 0;
	      out_rr <= 0;
	      out_3r <= 0;
	      err <= 0;
	      good <= 0;
	   end

	   else begin
	      in_r <= in;
	      out_3r <= out_rr;
	      out_rr <= out_r^ferr_rr;
	      out_r <= rawout;
	      if (init_dat != 39'h0000000000) en_r[3:0] <= {en_r[2:0],en};
	      if ( (&en_r) && (in_r != out_3r)) begin
		 count <= count + 1'b1;
		 err <= 1'b1;
	      end
	      else err <= 0;

	      if ( (&en_r) && (in_r == out_3r)) good <= 1'b1;
	      else good <= 0;


	      ferr_r <= force_error;
	      if (!ferr_done) begin
		 ferr_rr <= ferr_r;
		 ferr_done <= ferr_r;
	      end
	      else begin
		 ferr_rr <= 0;
		 ferr_done <= force_error;
	      end

	   end
	end // always @ (posedge clk, posedge rst)

endmodule




//
// PRBS39
//    J. Gilmore, 4/20/11
//-----------------------------------------------------------------------------
// 39-bit PRBS module, refs:
//   http://www.eng.auburn.edu/~strouce/polycorrect2.pdf
//   http://www.xilinx.com/support/documentation/application_notes/xapp052.pdf
//   http://www.eecs.berkeley.edu/~newton/Classes/CS150sp97/labs/lab5/lab5.html#primPolynomials
//   http://en.wikipedia.org/wiki/Linear_feedback_shift_register
//   http://www.springerlink.com/content/u590n42r72568803/
// Note that for any primitive polynomial, the reverse polynomial is also primitive!
//-----------------------------------------------------------------------------

module prbs39(
	input [38:0] init_dat,
	input        en,
	output reg   out,
	input        rst,
	input        clk);

	reg  [39:1] lfsr;

	always @(posedge clk or posedge rst) begin
	   if(rst) begin
	      lfsr[39:1]  <= init_dat[38:0];
	      out <= 0;
	   end
	   else begin
	      out <= lfsr[38];
	      if (en) lfsr[39:1] <= {lfsr[38:1],lfsr[39]^lfsr[4]};
//	      else lfsr  <= lfsr;  // why do this???
	   end
	end // always
endmodule
