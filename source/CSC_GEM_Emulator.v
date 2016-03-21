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

// Clocks
    input 	      ck125n, ck125p,
    input 	      ck160n, ck160p,
    input 	      lhc_ckn, lhc_ckp,
    input 	      tmb_clock0,

    input 	      qpll_lock,

    output        gtl_loop,

// Switches
    input         pb,  // push button
    input  [8:7]  sw,

// Ethernet
    input 	      ck_gben, ck_gbep,
    input 	      gbe_rxn, gbe_rxp,
    input 	      gbe_fok,
    output 	      gbe_txn, gbe_txp,
    output 	      f_sclk,

    output 	      rst_qpll, fcs,

// LEDs
    output reg [7:0]    led_low,
    output reg [15:8]   led_hi,
    output reg [10:9]   test_led,

// Snap-12 Tranceivers
    input 	          t12_fault,
    input             r12_fok,
  //input  [7:1]      rxp,rxn,
    output [7:0]      txp,txn,
    output 	          t12_rst,
    output            t12_sclk,
    output            r12_sclk
) /* synthesis syn_useioff = 1 */;

//----------------------------------------------------------------------------------------------------------------------
// Command Parameters
//----------------------------------------------------------------------------------------------------------------------

    parameter CMD_WRITE   = 16'hf7f7; // writes to block ram
    parameter CMD_READ    = 16'hf3f3; // reads from block ram
    parameter CMD_DUMP    = 16'hfdfd; // dump block ram contents
    parameter CMD_PACK    = 16'hf1f1; // fill block ram through cluster packer

    // not used in sw
    parameter CMD_SEND    = 16'hfefe;
    parameter CMD_REWIND  = 16'hf0f0;

    parameter EOF         = 16'hf7fd; // 16'b1111011111111101

//----------------------------------------------------------------------------------------------------------------------
// Register Declarations and Interconnects
//----------------------------------------------------------------------------------------------------------------------

    // Counters
    //-------------------------
    reg [22:0]  free_count;
    reg 	      free_tc;
    reg 	      slow_tc;
    reg [7:0]   time_count;
    reg [7:0]   time_r=8'h00;
    reg [7:0]   time_r_i;

    wire        startup_done = time_r[7];

    // fixed length zeros
    //-------------------------
    wire        zero   =   1'b0;
    wire        one    =   1'b1;
    wire [1:0]  zero2  =   2'b0;
    wire [12:0] low    =  13'b0;


    // snap12 GTX signals
    //-------------------

    assign      gtl_loop = 1'b0; // JRG: always set LOW (SEU danger, make OPEN --PD)   **Now INPUT for Mezz 2012!**
    wire        synced_snapt;
    wire        snap_clk2, ck160_locked;
    wire [7:0]  tx_begin, tx_fc;
    reg  [7:0]  ferr_f=0;
    reg  [15:0] err_count;
    reg  [7:0]  time_r_snap;
    reg  [7:0]  time_snap;
    wire stopped, locked, lock40;

    parameter SEEDBASE = 64'h8731c6ef4a5b0d29;
    parameter SEEDSTEP = 16'hc01d;

    wire        ext_rst, force_err, stat0; // stat[3:0] =  cfebemul_in[2,3,1,4] on Emul board!


    // Clock Signals
    //---------------

    wire        ck125, ck160, lhc_ck, lhc_clk, qpll_ck40, slwclk; // ext 125 usr, QPLL160, ccb_ck40, QPLL40, ccb_ck40/25=1.6MHz
    wire        lhc_ck0, lhc_ck90, lhc_ck180, lhc_ck270, lhcckfbout, lhcckfbout_buf;

    wire        ck40, ck40buf, rdclk, wrclk;
    reg 	    sel_rdclk = 0;
    reg         sel_wrclk = 0;

    // pack state machine counters

    reg [15:0] pack_rd_adr = 0;
    reg [15:0] pack_wr_adr = 0;
    reg [3:0]  pack_delay = 0;
    reg pack_rd = 0;
    reg pack_wr = 0;
    reg packing = 0;
    reg chm_fb = 0;
    reg [11:0] chm_adr;

    // Add-ons for Ben dCFEB testing:
    //-------------------------------

    wire          tx_clk_out, tx_clk;
    wire          rx_strt, rx_valid, rx_match, rx_fc;
    reg           fiber_reset;

    // Physical Button
    //----------------

    reg hold_bit       =  0; // debounced !pb signal, held until button release
    reg debounced_bit  =  0; // sets one pulse for 200 ns  (5 MHz clock)

    // GbE and BRAM
    //-------------

    wire  gbe_refck;                  // GTXE1 ref clk, used for MGTref and DoubleReset clock
    wire  gbe_txclk2, gbe_txclk2_buf; // drives logic for Tx, the "internal" choice for USR clock
    wire  txoutclk_mmcm_lk;

    parameter MXBRAMS    =  12'd32;   // was bff for 256 Bram's: bff, or 8 -> b07.
    parameter GBE_LIM    =  16'h080b; // WORDcount limit. allow extra bytes for MAC preamble, addr's...
    parameter MX_RX_ADR  =  11'h7ff;  // 7ff=2047

    reg [15:0]  pkt_lim;     // BytecountLimit/2. Plus extra 22 bytes for MAC preamble etc.
    reg [15:0]  pkt_counter; // ^^^ 1st data word @ counter == 12; need 11 extra word counts, 22 bytes.
    reg [10:0]  tx_adr, rx_adr, rx_adr_r;
    wire [2:0]  iram_word = rx_adr[1:0];
    wire [2:0]  oram_word = tx_adr[1:0];
    reg [15:0]  gbe_rxcount=16'h0000;
    reg [7:0]   pkt_id=8'h0000;
    reg [15:0]  prev_cmd[3:0];
    reg [11:0]  bk_adr;

    reg         tx_resetdone_r   =  0;
    reg         rx_resetdone_r   =  0;
    reg         rx_resetdone_r2  =  0;
    reg         rx_resetdone_r3  =  0;

    reg         pkt_send;
    wire        tx_resetdone, rx_resetdone;
    reg  [7:0]  ovfl_packet;
    reg 	      rx_timeout    = 1'b0;
    wire        ckgbe;
    wire        rxpll_lk;
    wire [2:0]  rx_bufstat;
    reg  [1:0]  gbe_kout      = 2'h1;
    reg  [1:0]  tx_kout       = 0;
    reg  [1:0]  tx_kout_r     = 1'b1;
    reg 	      comma_align   = 0;
    wire [5:0]  rxer;
    reg  [15:0] gbe_txdat     = 16'h50bc;
    reg  [15:0] tx_out;
    reg  [15:0] tx_out_r      = 16'h50bc;
    wire [15:0] gbe_rxdat;
    reg  [15:0] l_gbe_rxdat;
    reg  [15:0] ll_gbe_rxdat;
    reg 	      l_kchar       = 0;
    reg         ll_kchar;
    reg         mac_seek      = 0;
    reg         mac_sync      = 0;
    reg         mac_ack       = 0;
    reg         counter_send;
    reg  [1:0]  kchar_r       = 2'b0;
    reg  [1:0]  sync_state    = 2'h0;
    reg  [1:0]  data_state    = 2'h0;



    wire tx_data      = (data_state==2'h0);
    wire tx_crc_byte0 = (data_state==2'h1);
    wire tx_crc_byte1 = (data_state==2'h2);
    wire tx_eof       = (data_state==2'h3);
    wire end_of_packet = data_state[1];

    reg  [15:0] data_bram, data_bram_r;  // these are the BRAM MUX bus & register

    wire [63:0] data_oram [MXBRAMS-1:0];
    reg  [63:0] data_iram;

    reg cycle4;       // use this to toggle bram WRITE every 4th GbE word during cmd=f7f7
    reg loading_bram; // was called cmdf7f7

    reg         crc_en, crc_rst;
    wire [31:0] crc_out;
    reg  [15:0] byte_count;

    wire [MXBRAMS-1:12'h000] sbiterr_oram,  dbiterr_oram;

    assign f_sclk   = 1'b1;  // to Finisar GbE Transceiver

    assign fcs      = 1'b1;  // drive high for Mezz 2012 compatibility, useful on the bench
    assign t12_rst  = 1'b1;  // low-true signal for Snap12 Transmitter
    assign t12_sclk = 1'b1;  // to Snap12 Transmitter
    assign r12_sclk = 1'b1;  // to Snap12 Receiver
    assign rst_qpll = 1'b1;  // reset is low-true, but in ExtControl mode (sw6 On) it becomes fSelect[5]
    //                                                 and autoRestart (sw5) becomes fSelect[4]
    //        note that sw1-4 are fSelect[0:3] but they only function in ExtControl mode (sw6 On),
    //        and fSelect sets the VCXO center frequency (because automatic calibration is Off)
    //   wire qpll_lock; // probably random in ExtControl mode (sw6 On)

    //   reg [11:0]  l1a_count; // count L1accepts

    //  -- right now we access this with CCB base+2a (CSRB6, write a 1 then a 0 to bit2): we get a random count each time

    reg         alct_cfg_out, tmb_cfg_out, results_hold, late_load_done;
    reg  [1:0]  tmb_l1a_relreq;

    integer     i;

  // GbE Transceiver

    reg   event_done_r = 0, dump_done_r = 0, dump_loop = 0;

    reg [7:0]  nbx_i=0, nbx=0;


//----------------------------------------------------------------------------------------------------------------------
// Clock Generation
//----------------------------------------------------------------------------------------------------------------------

    // lhc clock from tmb baseboard
    BUFG lhcck(.I(tmb_clock0), .O(lhc_ck)); // only goes to mmcm now for 4-phase generation

    // Differential LHC clock from QPLL
    IBUFGDS #(.DIFF_TERM("TRUE"), .IOSTANDARD("LVDS_25"))  qpll40  (.I(lhc_ckp), .IB(lhc_ckn), .O(qpll_ck40));
    IBUFGDS #(.DIFF_TERM("FALSE"),.IOSTANDARD("LVDS_25"))  clock125(.I(ck125p) , .IB(ck125n) , .O(ck125));

    // Differential 160 mhz from QPLL
    IBUFDS_GTXE1  clock160(.I(ck160p) , .IB(ck160n) , .O(ck160), .ODIV2(), .CEB(zero));

    // 125 MHz GbE Clock
    IBUFDS_GTXE1  clock_gbe(.I(ck_gbep) , .IB(ck_gben) , .O(ckgbe), .ODIV2(), .CEB(1'b0));
    BUFG gbe_refclock(.I(ckgbe), .O(gbe_refck));

    // 5 MHz clock derived from 40Mhz LHC clock
    bufg_div8clk clk1p6(lhc_clk,!lhc_locked,slwclk,stopped,locked); // slwclk is now 5 MHz again (was 1.6 MHz with div25 and lhc_clk, then 5 MHz using ck125)

    // MMCM for 4-phase LHC clock
    MMCM_ADV #(
        .BANDWIDTH            ("OPTIMIZED"),
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
    mmcm_lhc4phase (
    // Output clocks
        .CLKFBOUT            (lhcckfbout),
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
        .RST                 (1'b0)
    );

    // Output buffering
    //-----------------------------------
    BUFG lhcclkf_buf     ( .O (lhcckfbout_buf), .I (lhcckfbout));
    BUFG lhcclkout0_buf  ( .O (lhc_clk),        .I (lhc_ck0));

    wire clk_sump = lhc_ck180 | lhc_ck270 | ck94 | ck87 | stopped;//ck15 ;

//----------------------------------------------------------------------------------------------------------------------
// Snap-12 Tx
//----------------------------------------------------------------------------------------------------------------------

    // resets
    //------------------------------------------------------------------------------
    wire gtx_ready = tx_resetdone_r & rx_resetdone_r & txoutclk_mmcm_lk & lock40 & ck160_locked;

    // synchronize reset to 40MHz clock
    reg reset, reset_sync;
    always @(posedge ck40) begin
      reset_sync <= (!sw[7] & !pb);
      reset <= reset_sync;
    end

    wire gtx_reset = reset | !gtx_ready;
    wire rxdv      = ~(|rxer | rx_bufstat[2]) & gbe_fok; // idle is K28.5,D16.2 = BC,50 in time order

    wire [8:0]  rd_addr;
    wire [8:0]  wr_addr;
    reg  [15:0] rd_ptr=0;

    // we split the 125 MHz 16 bit cmd into two 62.5 MHz 8-bit commands.
    reg [15:0]  cmd_code    = 0; // 160 MHz from fiber
    reg  [7:0]  cmd_code_r  = 0; // 40/62.5 MHz s0
    reg  [7:0]  cmd_code_rr = 0; // 40/62.5 MHz s1

    wire dump_enable_next  =  (!dump_enable&debounced_bit) | (cmd_code==CMD_DUMP);
    wire rxdat_is_cmd      =  (gbe_rxdat&16'hf0f0)==16'hf0f0 && (gbe_rxdat&16'h0f0f)!=16'h0f0f;
    wire synchronized      =  startup_done && !sync_state[0];

    reg  [7:0]  ibx=0;
    reg  [7:0]  nbx_r=0;

    reg dump_loop_i=0;
    reg dump_loop_r=0;

    reg dump_enable    = 0; // 160 MHz
    reg dump_enable_r  = 0; // 40/62.5 MHz s0
    reg dump_enable_rr = 0; // 40/62.5 MHz s1

    reg event_enable   = 0; // 160 MHz
    reg event_enable_r = 0; // 40/62.5 MHz s0


    reg dump_done=0;
    reg event_done=0;

    wire send_event = dump_enable_rr || (event_enable_r & !event_done);  // use this OR (f3f3 && (bk_adr == v)) to Read Enable the BRAMs

    // reset if last command is rewind or write
    wire last_cmd_is_rewind = {cmd_code_r,cmd_code_rr}==CMD_REWIND;
    wire last_cmd_is_write  = {cmd_code_r,cmd_code_rr}==CMD_WRITE;

    wire ibx_reset = (gtx_reset || last_cmd_is_rewind || last_cmd_is_write);

    // Read Clock Domain
    //-----------------------------------
    always @(posedge rdclk) begin
    // rdclk switches between 40MHz and 62.5 MHz clock domains based on the state of sel_rdclk

        fiber_reset    <= gtx_reset;
        dump_enable_r  <= dump_enable;
        dump_loop_r    <= dump_loop & dump_enable_r; // dump_loop is set in gbe cmd section
        event_enable_r <= event_enable;

        nbx_r          <= nbx;

        cmd_code_r     <= cmd_code[7:0];
        cmd_code_rr    <= cmd_code_r;

        dump_done      <= (!gtx_reset && dump_enable_rr) ? (&rd_ptr[8:1]) : 1'b0; // end dump when rd_ptr is 1FE or 1FF
        dump_enable_rr <= (!dump_done | dump_loop_r) & dump_enable & dump_enable_r;

        // event_done is used in gbe cmd section
        event_done     <= (!gtx_reset && event_enable_r) ? (ibx==nbx_r) : 1'b0;

        //
        if (ibx_reset || !(dump_enable_rr | event_enable_r))
            ibx <= 8'h0;
        else
            ibx <= (dump_enable_r) ? 8'h0 : (ibx<nbx_r) ? ibx + 1'b1 : ibx;

        // JGhere, use rd_ptr as BRAM RdAddr unless f3f3 is set ? use sel_rdclk for it!
        if(!pack_rd) rd_ptr <= ibx_reset ? 16'b0 : (dump_enable_rr || event_enable_r) && (!event_done) ? rd_ptr + 1'b1 : rd_ptr;
        else rd_ptr <= {2'b0,pack_rd_adr[15:2]};

    end // close rdclk

    wire [55:0] gem_packet0;
    wire [55:0] gem_packet1;

    reg [55:0] gem_fiber_out [3:0];
    reg [47:0] cfeb_fiber_out[3:0];
    always @(negedge ck40) begin  // 80 MHz derived from GTX_TxPLL
         gem_fiber_out[0][55:0] <= (send_event) ? data_oram[0][55:0] : 56'hffffffffffffff;
        cfeb_fiber_out[0][47:0] <= (send_event) ? data_oram[1][47:0] : 48'h000000000000;
        cfeb_fiber_out[1][47:0] <= (send_event) ? data_oram[2][47:0] : 48'h000000000000;
        cfeb_fiber_out[2][47:0] <= (send_event) ? data_oram[3][47:0] : 48'h000000000000;
        cfeb_fiber_out[3][47:0] <= (send_event) ? data_oram[4][47:0] : 48'h000000000000;
        gem_fiber_out[1][55:0] <= (send_event) ? data_oram[5][55:0] : 56'hffffffffffffff;
        gem_fiber_out[2][55:0] <= (send_event) ? data_oram[6][55:0] : 56'hffffffffffffff;
        gem_fiber_out[3][55:0] <= (send_event) ? data_oram[7][55:0] : 56'hffffffffffffff;
    end // always

//----------------------------------------------------------------------------------------------------------------------
// GbE Clocks
//----------------------------------------------------------------------------------------------------------------------

    // GbE clock buffer
    //----------------------------------------------------------
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

    BUFGMUX wrclk_buf (.O (wrclk), .I0 (ck40buf),.I1 (gbe_txclk2_buf), .S(sel_wrclk));

//----------------------------------------------------------------------------------------------------------------------
// Block Ram Generation for Pattern Injection
//----------------------------------------------------------------------------------------------------------------------

    assign rd_addr = (sel_rdclk) ? tx_adr[10:2] : rd_ptr[8:0];
    assign wr_addr = (pack_wr) ? pack_wr_adr[10:2] : rx_adr_r[10:2];

    wire [31:0] data_in0;
    wire [31:0] data_in1;

    wire valid_gem0;

    assign valid_gem0 = ~&{gem_packet0[10:9],gem_packet0[24:23]};

    assign data_in0 = packing ? (chm_fb ? gem_packet1[31:0] : gem_packet0[31:0]) : data_iram[31:0];
    assign data_in1 = packing ? (chm_fb ? {8'b0,gem_packet1[55:32]} : {8'b0,gem_packet0[55:32]}) : data_iram[63:32];

    wire bram_rd_en[MXBRAMS-1:0];
    wire bram_wr_en[MXBRAMS-1:0];

    genvar ibram;
    generate
    for (ibram=12'h000; ibram<MXBRAMS; ibram=ibram+1'b1) begin:bramgen

        assign bram_rd_en[ibram] = send_event || ((cmd_code==CMD_READ) & (bk_adr==ibram) & gtx_ready) || (pack_rd && ibram > 7);
        assign bram_wr_en[ibram] = (cycle4 & (bk_adr==ibram) & gtx_ready) || (pack_wr && 
            ((chm_adr == 0 && chm_fb ==0 && ibram == 0) || (chm_adr == 0 && chm_fb == 1 && ibram == 5) || (chm_adr == 1 && chm_fb ==0 && ibram == 6) || (chm_adr == 1 && chm_fb == 1 && ibram == 7)));

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
            // read and write address
            .ADDRARDADDR    ({1'b1, rd_addr  [8:0], 6'h3f}),  // 16 bit RDADDR, but only 14:6 are used w/ECC.  1/3f?
            .ADDRBWRADDR    ({1'b1, wr_addr  [8:0], 6'h3f}),  // 16 bit WRADDR, but only 14:6 are used w/ECC.  1/3f?

            // read and write clock
            .CLKARDCLK      (rdclk),                                // RDCLK
            .CLKBWRCLK      (wrclk),                           // WRCLK    and ensure safe setup time for ADR when EN is Enabled!

            // data in
            .DIADI          (data_in0),  // DI low 32-bit
            .DIBDI          (data_in1), // DI high 32-bit

            // data out
            .DOADO          (data_oram[ibram][31:0]),               // DO low 32-bit
            .DOBDO          (data_oram[ibram][63:32]),              // DO high 32-bit

            // in port read enable
            .ENARDEN        (bram_rd_en[ibram]),                    // RDEN
            .ENBWREN        (bram_wr_en[ibram]), // WREN  Alfke: "WE off" is not sufficient to protect

            // bit errors
            .SBITERR        (sbiterr_oram[ibram]),
            .DBITERR        (dbiterr_oram[ibram]),

            // misc

            .WEA            (4'h0),                                 // WEA, NA for SDP

            .REGCEAREGCE    (1'b0),                                 // REGCE, NA if DO_REG=0
            .RSTRAMARSTRAM  (1'b0),

            .RSTRAMB        (1'b0),                                 // NA if SDP
            .RSTREGARSTREG  (1'b0),                                 // NA if SDP or DO_REG=0
            .RSTREGB        (1'b0),                                 // NA if SDP or DO_REG=0
            .WEBWE          (8'hFF),                                // WE?  WEBWE ( 8 bits)
            .REGCEB         (1'b0),                                 // REGCEB        Init data, So require stable clocks before EN is set,
            .INJECTSBITERR  (1'b0),
            .INJECTDBITERR  (1'b0)

            // not valid if DO_REG=0:  REGCEAREGCE, REGCEB, RSTREGARSTREG, RSTREGB
            // not used for SDP:  CASCADE*, REGCEB, RSTRAMB, RSTREGARSTREG, RSTREGB, WEA
            //.RST  (reset) // err.mon: SBITERR DBITERR
        );
    end
    endgenerate

//----------------------------------------------------------------------------------------------------------------------
// Clock Domain Crossing ffs
//----------------------------------------------------------------------------------------------------------------------

    // slow clock startup counter
    //----------------------------------------------------------------------------------------------------------------------

    always @(posedge slwclk or posedge reset) // everything that uses 1.6 MHz clock with simple Reset (from lhc_clk)
    begin
        if (reset)
            time_count <= 0;
        else begin
            if (gtx_ready && time_count < 8'hfe)
                time_count <= time_count + 1'b1; // use to delay startup; ends.
        end
    end


    // transfer slow clk startup counter to GbE USR clock domain
    always @(posedge gbe_txclk2) begin
        time_r_i  <= time_count;
    end

    always @(posedge gbe_txclk2 or posedge reset) begin
        time_r    <= (reset) ? 8'h00 : time_r_i;
    end

    // everything that uses GbE USR clock, no Reset
    always @(posedge gbe_txclk2) begin
        tx_out_r  <= tx_out;
        tx_kout_r <= tx_kout;
    end

    // everything using GbE USR clock w/simple Reset
    always @(posedge gbe_txclk2 or posedge reset)
    begin
        tx_resetdone_r <= (reset) ? 1'b0 : tx_resetdone;
        rx_resetdone_r <= (reset) ? 1'b0 : rx_resetdone;
    end

//----------------------------------------------------------------------------------------------------------------------
// Ethernet CRC calculation
//----------------------------------------------------------------------------------------------------------------------

        mac_crc  mac_crc32(
            .crc_dat ( gbe_txdat),
            .crc_en  ( crc_en),
            .crc_out ( crc_out),
            .rst     ( crc_rst),
            .clk     ( gbe_txclk2)
        );

//----------------------------------------------------------------------------------------------------------------------
// Fiber out Dump Control
//----------------------------------------------------------------------------------------------------------------------

    always @(posedge gbe_txclk2 or posedge gtx_reset) // everything using GbE USR clock w/GTX_Reset
    begin
    if (gtx_reset) begin
        pkt_lim     <= 16'd64;
        sel_rdclk   <= 1'b0;
        dump_enable <= 1'b0;
        event_enable <= 1'b0;
    end
    else begin

        // load counter2 to READ registers and send it out
        //-------------------------------------------------------------
        if      (cmd_code == CMD_READ)  pkt_lim <= 16'd2061; // 16'd61 special test size 100 bytes.  later will be 4 KB
        else if (cmd_code == CMD_WRITE) pkt_lim <= 16'd2048; // just over 4 KB for good Rx/Tx BRAM loading overlap
        else                            pkt_lim <= 16'd36;   // our usual 50-byte size is default; F3F3 is the special 4 KB count

        byte_count <= 16'hffff&((pkt_lim*2) - 16'd22); // Need BYTES; subtract preamble bytes etc.

        // dump enable
        // dumps entire bram contents
        //---------------------------------------------
        if      (dump_enable_next)        dump_enable <= 1'b1;         // was !dump_done_r ...before that was 1'b1;.  dump_done is driven by ck40
        else if (cmd_code[15:12] == 4'hF) dump_enable <= 1'b0;         // stop dump on any non-FDFD command: F0F0, F3F3, F7F7, FEFE...
        else if (dump_enable)             dump_enable <= !dump_done_r; // dump_done is driven by ck40   ... was 1'b1;
        else                              dump_enable <= dump_enable;

        // event enable
        // sends a single bx's data from the bram
        //---------------------------------------------

        if      (cmd_code == CMD_SEND)    event_enable <= !event_done_r;  // event_done is driven by ck40
        else if (cmd_code[15:12] == 4'hF) event_enable <= 1'b0;           // stop dump on any non-FEFE command: F0F0, F3F3, F7F7, FDFD...
        else                              event_enable <= event_enable;

        // control rdclk and wrclk source
        //---------------------------------------------
        sel_rdclk <= (cmd_code==CMD_READ); // JGhere, maybe this will change the clock before the Read is finished, so delay it?
    end
    end

//----------------------------------------------------------------------------------------------------------------------
// Block RAM Data Readout
//----------------------------------------------------------------------------------------------------------------------

    always @(posedge gbe_txclk2 or posedge gtx_reset) // everything using GbE USR clock w/GTX_Reset
    begin
    if (gtx_reset) begin
        pkt_counter     <= 16'h0000;
        comma_align     <= 0;
        crc_rst         <= 1;
        data_bram       <= 16'hd1d0;
        data_bram_r     <= 16'he4e2;
        dump_done_r     <= 0;
        dump_loop       <= 0;
        event_done_r    <= 0;
        nbx             <= 0;
        pkt_id          <= 8'h0000;
        rx_resetdone_r2 <= 0;
        rx_resetdone_r3 <= 0;
        pkt_send        <= 0; // 1st data word @counter == 12 --> 11 extra words, so subtract 22 bytes.
    end

    else begin // Not Reset case

        // domain crossing ffs
        event_done_r    <= event_done;
        dump_done_r     <= dump_done;

        rx_resetdone_r2 <= rx_resetdone_r;
        rx_resetdone_r3 <= rx_resetdone_r2;

        // maybe use time_r > 8'h10 ?
        // driven high to signal the gtx to perform comma alignment
        // enable once the tranceiver is reset
        //-------------------------------------------------------------
        comma_align <= (rx_resetdone_r3)    ? 1'b1 : comma_align;


        // needs one cycle during state 3
        // finished sending 2nd crc byte, now reset the mac_crc machine
        //--------------------------------------------------------------
        crc_rst <= (tx_crc_byte1) ? 1'b1 : 1'b0;

        // latch mode to set FPGA for continuous loop of sending all patterns
        //-------------------------------------------------------------
        dump_loop <= (dump_done_r) ? 1'b0 : dump_loop_i;

        // latch number of bx to send (8 bits)
        //-----------------------------------------
        nbx <= (event_done_r) ? 8'b0 : nbx_i;


        // block ram data readout multiplexer
        //----------------------------------------------------------------------------



        data_bram_r  <= data_bram;

        if (bk_adr>(MXBRAMS-1))
            data_bram <= 16'h0a0f; // limited range of bk_adr space
        else if (cmd_code == CMD_READ) begin
            case (oram_word)
                2'b01: data_bram <= data_oram[bk_adr][15: 0];
                2'b10: data_bram <= data_oram[bk_adr][31:16];
                2'b11: data_bram <= data_oram[bk_adr][47:32];
                2'b00: data_bram <= data_oram[bk_adr][63:48]; // correct for latency of BRAM readout response
            endcase
        end
        else begin
            data_bram <= 16'hdfd8;
        end


        pkt_counter <= (end_of_packet || rx_timeout)  ? 16'h0000 : (synchronized && pkt_send) ? pkt_counter + 1'b1 : pkt_counter;

        pkt_id      <=  end_of_packet && tx_crc_byte1 ? pkt_id + 1'b1 : pkt_id;

        if      (end_of_packet || rx_timeout )  pkt_send <= 1'b0;
        else if (synchronized && (|cmd_code))   pkt_send <= 1'b1;
        else                                    pkt_send <= pkt_send;

    end
    end

//----------------------------------------------------------------------------------------------------------------------
// GbE Tx to PC
//----------------------------------------------------------------------------------------------------------------------


    always @(posedge gbe_txclk2 or posedge gtx_reset) // everything using GbE USR clock w/GTX_Reset
    begin
    if (gtx_reset) begin
        counter_send    <= 0;
        crc_en          <= 0;
        data_state      <= 0;
        gbe_kout        <= 2'h1;
        gbe_txdat       <= 16'h50bc;  // <-- idle 28.5/16.2
        kchar_r         <= 0;
        l_gbe_rxdat     <= 0;
        l_kchar         <= 0;
        ll_gbe_rxdat    <= 0;
        ll_kchar        <= 0;
        mac_ack         <= 0;
        mac_seek        <= 0;
        mac_sync        <= 0;
        sync_state      <= 0;
        tx_adr          <= 0;
    end

    else begin // Not Reset case


        if (pkt_counter!=0 && pkt_counter<=pkt_lim) begin  // put data on gbe_txdat after preamble, MACaddr's & Length

            counter_send <= 1;

            // word 1
            if (pkt_counter==16'd1) begin
                gbe_txdat <= 16'h55fb;
                gbe_kout  <= 2'h1;
                tx_adr    <= 11'h0;
            end

            // words 2, 3, 4
            else if (pkt_counter<16'd5) begin
                gbe_txdat <= {pkt_counter[2],15'h5555};
                gbe_kout  <= 2'h0;
            end

            // words 5, 6 (crc)
            else if (pkt_counter<16'd7) begin
                crc_en    <= 1;
                gbe_txdat <= 16'hffff; // MAC32 No Longer inverts gbe_txdat input w/counter == 5 or 6; just send ones.
            end

            // words 8, 9 10
            else if (pkt_counter<16'd11) begin
                gbe_txdat <= 16'h0000;
            end

            // word 11
            else if (pkt_counter<16'd12) begin  // only one cycle for this step!  counter == 11
                gbe_txdat <= {byte_count[7:0],byte_count[15:8]};
                tx_adr    <= tx_adr + 1'b1; // shifting 2-deep tx_bram pipeline
            end

            // word 12
            else if (pkt_counter<16'd13) begin  // only one cycle for this step!  counter == 12
                gbe_txdat <= cmd_code;    //  <-- first word returns the cmd_code
                tx_adr    <= tx_adr + 1'b1; // shifting 2-deep tx_bram pipeline
            end

            // data
            // words 13--xxx
            else begin
                // start incrementing tx ram address
                tx_adr <= tx_adr+1'b1;

                // not read, or rewind
                if (cmd_code[2]) begin  // Function4 is active, or f7 or fc, ff.  send cmd_code first.
                    if (pkt_counter < 16'd14) gbe_txdat <= {4'hd,bk_adr[11:0]};         //  <-- 2nd word returns the bk_adr
                    else                      gbe_txdat <= {pkt_id[4:0],rx_adr[10:0]};  // send the sequential packet ID for each packet, plus 11 bit sub-address
                end

                // rewind read address pointer
                else if (~|cmd_code[3:0]) begin // cmd_rewind
                    gbe_txdat <= prev_cmd[pkt_counter[1:0]];  //  returns preveious few cmd_codes, repeats to end of packet
                end

                // block ram readout
                else if (bk_adr < MXBRAMS) begin  // this is data_bram range.  for f3f3, f8f8, fbfb...
                    if (pkt_counter < 16'd14) gbe_txdat <= {4'hd,bk_adr[11:0]}; // <-- 2nd word returns the bk_adr
                    else                      gbe_txdat <= data_bram_r[15:0];   // added a pipe register for data_bram, allows better timing?
                end

                // frame synchronization
                else begin
                    gbe_txdat <= 16'h50bc;  // <-- idle 28.5/16.2
                    gbe_kout  <= 2'h1;
                end

            end // if (pkt_counter >= 16'd13)
        end // if (pkt_counter > 0 && pkt_counter <= GBE_LIM)


        // MAC Synchronization
        //----------------------------------------------------------------------------
        //jg note: the MAC_ACK etc still don't work (v11p5)...do we care?
        else if (!synchronized) begin  // send Sync "pairs" for a while after Reset
            if (time_r[6] && time_r[1]) begin
                mac_seek <= (ll_kchar && !l_kchar && ll_gbe_rxdat == 16'h42bc);
                if (mac_seek && l_kchar && l_gbe_rxdat == 16'hb5bc)
                    mac_sync <= 1'b1;
                if (mac_seek && mac_sync && !ll_kchar && ll_gbe_rxdat == 16'h4000)
                    mac_ack <= 1'b1;
            end
            else
                mac_seek <= 0;

            case (sync_state)
                2'b00:   gbe_txdat <= 16'h42bc;
                2'b10:   gbe_txdat <= 16'hb5bc;
                default: gbe_txdat <= {1'b0,mac_sync,14'h0000}; // <-- we want this to be the final sync state.
            endcase
            gbe_kout  <= ((~sync_state) & 2'b01);
            sync_state <= sync_state + 1'b1;
        end

        else  begin // otherwise send Idles, allows time for Rx Checking
            // at reset data_state = 0
            // when counter_send goes high, data state is set to one, then it starts counting


            if      (counter_send)  data_state <= 2'h1;
            else if (data_state==0) data_state <= data_state;
            else                    data_state <= data_state + 1'b1;

            counter_send <= 0;
            crc_en       <= 0;
            gbe_txdat    <= 16'h50bc;  // <-- idle 28.5/16.2
            gbe_kout     <= 2'h1;
            tx_adr       <= 0;
        end

        ll_kchar     <=  l_kchar;
        l_kchar      <= |rxer[1:0];
        kchar_r      <=  rxer[1:0];
        ll_gbe_rxdat <=  l_gbe_rxdat;
        l_gbe_rxdat  <=  gbe_rxdat;

    end // not reset case
    end // posedge clock

//----------------------------------------------------------------------------------------------------------------------
// GbE Rx from PC
//----------------------------------------------------------------------------------------------------------------------

    reg start_pack = 0;

    always @(posedge gbe_txclk2 or posedge gtx_reset) // everything using GbE USR clock w/GTX_Reset
    begin
        if (gtx_reset) begin
            ovfl_packet     <= 0;
            rx_timeout      <= 1'b0;
            gbe_rxcount     <= 16'h0000;
            bk_adr          <= 12'h000;
            rx_adr_r        <= 0;
            cmd_code        <= 16'h0000;
            dump_loop_i     <= 0;
            nbx_i           <= 0;
            cycle4          <= 0;
            loading_bram    <= 0;
            prev_cmd[0]     <= 16'h0;
            prev_cmd[1]     <= 16'h0;
            prev_cmd[2]     <= 16'h0;
            prev_cmd[3]     <= 16'h0;
            rx_adr          <= 0;
            data_iram[15:0] <= 16'h0000;
            data_iram[31:16]<= 16'h0000;
            data_iram[47:32]<= 16'h0000;
            data_iram[63:48]<= 16'h0000;
        end
        else begin
            bk_adr   <= (end_of_packet || rx_timeout) ? 12'h000  : bk_adr;
            cmd_code <=  end_of_packet && |cmd_code ? 16'h0000 : cmd_code;

            if (startup_done && rxdv) begin // get command from first 1-2 data bytes of packet
                // GbE Rx Counter
                // -----------------------------------------------------
                if (gbe_rxcount == 16'd4407) begin     // tuned to handle weird .5 sec rxdv case after Reset.
                    ovfl_packet <= ovfl_packet + 1'b1; // tracks when a rx packet times-out, over jumbo GbE limit
                    rx_timeout  <= 1'b1;               // only set for pretty big jumbo packet, over 8810 bytes
                    gbe_rxcount <= gbe_rxcount;
                end
                else begin
                    gbe_rxcount <= gbe_rxcount + 1'b1;
                end
                // rx_adr to block ram zeroed if we are not updating. Does this even matter ?
                if      (cmd_code != CMD_WRITE) rx_adr_r <= 11'h000;
                else if (cycle4)                rx_adr_r <= rx_adr;
                // timeout reset
                //-----------------------------------------------------
                rx_adr <= (rx_timeout) ? 11'h0 : rx_adr;
                // Break down GbE packet Rx from the PC:
                //-----------------------------------------------------
                if ( rx_timeout | event_done_r | dump_done_r )
                begin
                    cmd_code      <= 16'h0000;
                    dump_loop_i   <= 0;
                    nbx_i         <= 0;
                    cycle4        <= 0;
                    loading_bram  <= 0;
                end

                // latch command (word 3)
                //-------------------------
                else if ((gbe_rxcount==16'd3) && rxdat_is_cmd)
                begin
                        cmd_code     <= gbe_rxdat; //   ^^^ make sure cmd code looks valid ^^^
                        prev_cmd[0]  <= gbe_rxdat;
                        prev_cmd[1]  <= prev_cmd[0];
                        prev_cmd[2]  <= prev_cmd[1];
                        prev_cmd[3]  <= prev_cmd[2];
                        rx_adr[10:0] <= 11'h000;
                        cycle4       <= 0;
                        loading_bram <= 0;
                end

                // parse command and latch address (word 4)
                //-----------------------------------------------
                else if ((gbe_rxcount==16'd4) && |cmd_code)
                begin
                    dump_loop_i <= (cmd_code==CMD_DUMP) ? (gbe_rxdat[3:0]==4'hC) : 1'b0;
                    nbx_i       <= (cmd_code==CMD_SEND) ? (gbe_rxdat[7:0])       : 8'b0;
                    bk_adr      <= (cmd_code==CMD_READ || cmd_code==CMD_WRITE || cmd_code==CMD_PACK) && (gbe_rxdat[11:8]==4'h0) ? gbe_rxdat[11:0] : 12'b0;
                    start_pack  <= cmd_code == CMD_PACK;
                    sel_wrclk <= (cmd_code==CMD_WRITE) ? 1'b1 : 1'b0;
                    data_iram[15:0] <= 16'hdddd;
                end

                // parse_data
                //------------------------------------------------------------------------------------------
                else if (gbe_rxcount > 16'd4 && cmd_code==CMD_WRITE && bk_adr<MXBRAMS)
                begin
                    // loading block ram flag
                    loading_bram <= (rx_adr==MX_RX_ADR) ? 1'b0 : (gbe_rxcount==16'h0005) ? 1'b1 : loading_bram;


                    // produces a synchronization pulse when the last word of a ram block is being read
                    cycle4 <= loading_bram && (iram_word==2'h3); // cannot begin before rx_count=6

                    // increment rx_addr until we reach the maximum counter
                    rx_adr <= (rx_adr==11'h7ff) ? rx_adr : rx_adr + 1'b1; // 7ff==2047; 2047/4 cyclces = 511.... 512 entires in bram. voila

                    case (iram_word)
                        2'h0: data_iram[15:0]  <= gbe_rxdat[15:0];
                        2'h1: data_iram[31:16] <= gbe_rxdat[15:0];
                        2'h2: data_iram[47:32] <= gbe_rxdat[15:0];
                        2'h3: data_iram[63:48] <= gbe_rxdat[15:0];
                    endcase
                end

                // fini
                //-----------------------------------------
                else begin
                    cycle4 <= 0;
                end // else: !if(gbe_rxcount == 3 or 4, or cmd=f7f7)

                if(start_pack) start_pack <= 1'b0;

            end  // if (startup_done & rxdv & correct switch state)

            else  begin  // if (rxdv == 0) or !startup_done
                gbe_rxcount  <= 16'h0000;
                rx_timeout   <= 1'b0;
                cycle4       <= 0;
                loading_bram <= 0;
            end
        end // else: !if(gtx_reset)
    end // @(posedge gbe_txclk2 or posedge gtx_reset)

//----------------------------------------------------------------------------------------------------------------------
// pack state machine
//----------------------------------------------------------------------------------------------------------------------

    reg [1:0] pack_state = 2'd0;

    wire state3;

    assign state3 = (pack_state == 2'd2);

    always @(posedge snap_clk2) begin
        case (pack_state)
            2'd0: begin
                if(start_pack)
                begin
                    pack_state <= 2'd1;
                    pack_rd <= 1'b1;
                    packing <= 1'b1;
                    chm_fb <= 1'b0;
                    chm_adr <= bk_adr;
                end
            end
            2'd1: begin
                pack_rd_adr <= pack_rd_adr + 1;
                pack_delay <= pack_delay + 1;
                if(pack_delay == 4'd13) begin
                    pack_wr <= 1'b1;
                    pack_state <= 2'd2;
                    pack_delay <= 4'b0;
                end
            end
            2'd2: begin
                pack_rd_adr <= pack_rd_adr + 1;
                pack_wr_adr <= pack_wr_adr + 1;
                if(pack_wr_adr == 16'd2047) begin
                    pack_state <= 2'd3;
                    pack_wr_adr <= 16'd0;
                    chm_fb <= 1'b1;
                end
            end
            2'd3: begin
                pack_rd_adr <= pack_rd_adr + 1;
                pack_wr_adr <= pack_wr_adr + 1;
                if(pack_wr_adr == 16'd2047) begin
                    pack_rd <= 1'b0;
                    pack_wr <= 1'b0;
                    pack_state <= 2'd0;
                    pack_rd_adr <= 16'd0;
                    pack_wr_adr <= 16'd0;
                    packing <= 1'b0;
                end
            end
        endcase
    end

//----------------------------------------------------------------------------------------------------------------------
// transmit data multiplexer
//----------------------------------------------------------------------------------------------------------------------

    always @(*) begin
        case (data_state)
            2'h0: tx_out = gbe_txdat[15:0 ];
            2'h1: tx_out = crc_out  [15:0 ];  // send out CRC "MSB-first"; it's already inverted and backwards
            2'h2: tx_out = crc_out  [31:16];
            2'h3: tx_out = EOF;
        endcase

        case (data_state)
            2'h0: tx_kout = gbe_kout;
            2'h1: tx_kout = 2'h0;
            2'h2: tx_kout = 2'h0;
            2'h3: tx_kout = 2'h3;
        endcase
    end  // always @ (*)

//----------------------------------------------------------------------------------------------------------------------
// GTX Instantiation
//----------------------------------------------------------------------------------------------------------------------

    wire [1:0]  rx_comma;
    wire [1:0]  rx_disp;  // other signals for GTX
    wire [3:0]  ignore;  // outputs from GTX we don't care about
    wire [1:0]  rx_lostsync;
    wire        gbe_tx_outclk; // out from Tx PLL

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

    wire gbe_sump =  (|rx_comma) |(|rx_disp) |(|ignore) |(|rx_lostsync);

//----------------------------------------------------------------------------------------------------------------------
// GEM Data Processing
//----------------------------------------------------------------------------------------------------------------------

    wire [63:0] vfat_sbits [23:0];

    genvar ivfat;
    generate;
    for (ivfat=0; ivfat<24; ivfat=ivfat+1) begin: fatloop
        assign vfat_sbits[ivfat] = data_oram[ivfat+8];
    end
    endgenerate

    wire [13:0] cluster [7:0];

    cluster_packer u_cluster_packer (

        .clock4x (snap_clk2),

        .global_reset (reset),

        .vfat0  (vfat_sbits[0]),
        .vfat1  (vfat_sbits[1]),
        .vfat2  (vfat_sbits[2]),
        .vfat3  (vfat_sbits[3]),
        .vfat4  (vfat_sbits[4]),
        .vfat5  (vfat_sbits[5]),
        .vfat6  (vfat_sbits[6]),
        .vfat7  (vfat_sbits[7]),
        .vfat8  (vfat_sbits[8]),
        .vfat9  (vfat_sbits[9]),
        .vfat10 (vfat_sbits[10]),
        .vfat11 (vfat_sbits[11]),
        .vfat12 (vfat_sbits[12]),
        .vfat13 (vfat_sbits[13]),
        .vfat14 (vfat_sbits[14]),
        .vfat15 (vfat_sbits[15]),
        .vfat16 (vfat_sbits[16]),
        .vfat17 (vfat_sbits[17]),
        .vfat18 (vfat_sbits[18]),
        .vfat19 (vfat_sbits[19]),
        .vfat20 (vfat_sbits[20]),
        .vfat21 (vfat_sbits[21]),
        .vfat22 (vfat_sbits[22]),
        .vfat23 (vfat_sbits[23]),

        .truncate_clusters (1'b0),

        .cluster0 (cluster[0]),
        .cluster1 (cluster[1]),
        .cluster2 (cluster[2]),
        .cluster3 (cluster[3]),
        .cluster4 (cluster[4]),
        .cluster5 (cluster[5]),
        .cluster6 (cluster[6]),
        .cluster7 (cluster[7])
    );

    assign gem_packet0 = {cluster[0], cluster[1], cluster[2], cluster[3]};
    assign gem_packet1 = {cluster[4], cluster[5], cluster[6], cluster[7]};
    wire gem_sump = (|gem_packet1);

//----------------------------------------------------------------------------------------------------------------------
// qpll lock lost latch
//----------------------------------------------------------------------------------------------------------------------

    reg qpll_lock_lost;
    always @(posedge lhc_clk or posedge reset) // everything that uses lhc_clk w/simple Reset
    begin
        if (reset) begin
            qpll_lock_lost <= 0;
        end
        else begin
            qpll_lock_lost <= !qpll_lock | qpll_lock_lost;
        end
    end

//----------------------------------------------------------------------------------------------------------------------
// snap12 mmcm clock manager
//----------------------------------------------------------------------------------------------------------------------

    SRL16E #(.INIT(16'h7FFF)) SRL16TXPLL(.Q(txpll_rst), .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b1), .CE (1'b1), .CLK(qpll_ck40), .D(1'b0));

    bufg_x2div2plus snap_mmcm (
        .CLK_IN1              ( tx_clk_out    ) , // 80 MHz
        .CLK_OUT1             ( tx_clk        ) , // 80 MHz
        .CLK_OUT2             ( snap_clk2     ) , // 160 MHz
        .CLK_OUT3             ( ck40          ) , // 40 MHz
        .RESET                ( !ck160_locked ) ,
        .LOCKED               ( lock40        ) ,
        .CLK_OUT3BUF          ( ck40buf       ) // from Tx GTX PLL out clk
    );

//----------------------------------------------------------------------------------------------------------------------
// GEM + CFEB Fiber Outputs
//----------------------------------------------------------------------------------------------------------------------

    // JGhere, fiber Tx "from DCFEB" section:
    //   was Snap12 module, not cfeb-tmb test code
    gem_fiber_out  gem0out   (
        .RST                 (reset),                 // Manual only
        .TRG_SIGDET          (),                      // from IPAD to IBUF.  N/A?
        .TRG_RX_N            (),                      // empty
        .TRG_RX_P            (),                      // empty
        .TRG_TDIS            (),                      // OBUF output, for what?  N/A?
        .TRG_TX_N            (txn[0]),                // pick a fiber
        .TRG_TX_P            (txp[0]),                // pick a fiber

        .GEM_DATA            (gem_fiber_out[0][55:0 ]),
        .GEM_OVERFLOW        (1'b0),

        .TRG_TX_REFCLK       (ck160),                 // QPLL 160 from MGT clk
        .TRG_TXUSRCLK        (snap_clk2),             // get 160 from TXOUTCLK (times 2)
        .TRG_CLK80           (tx_clk),                // get 80 from TXOUTCLK
        .TRG_GTXTXRST        (1'b0     ),             // maybe Manual "reset" only
        .TRG_TX_PLLRST       (txpll_rst),             // Tie LOW.
        .TRG_RST             (fiber_reset),           // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
        .ENA_TEST_PAT        (1'b0),                 // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
        //.ENA_TEST_PAT        (sw[8]),                 // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
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

    dcfeb_fiber_out  dcfeb1out (
        .RST                 ( reset),               // Manual only
        .TRG_SIGDET          (),                      // from IPAD to IBUF.  N/A?
        .TRG_RX_N            ( ),                    // empty
        .TRG_RX_P            ( ),                    // empty
        .TRG_TDIS            ( ),                    // OBUF output, for what?  N/A?
        .TRG_TX_N            ( txn[1]),              // pick a fiber
        .TRG_TX_P            ( txp[1]),              // pick a fiber
        .G1C                 ( cfeb_fiber_out[0][7:0]),   // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
        .G2C                 ( cfeb_fiber_out[0][15:8]),  // if ENA_TEST_PAT then it's prbs so these don't matter...
        .G3C                 ( cfeb_fiber_out[0][23:16]), // but good for testing low-rate non-zero triad data:
        .G4C                 ( cfeb_fiber_out[0][31:24]),
        .G5C                 ( cfeb_fiber_out[0][39:32]),
        .G6C                 ( cfeb_fiber_out[0][47:40]),
        .TRG_TX_REFCLK       ( ck160),               // QPLL 160 from MGT clk
        .TRG_TXUSRCLK        ( snap_clk2),           // get 160 from TXOUTCLK (times 2)
        .TRG_CLK80           ( tx_clk),              // get 80 from TXOUTCLK
        .TRG_GTXTXRST        ( txpll_rst),           // maybe Manual "reset" only
        .TRG_TX_PLLRST       ( txpll_rst),           // Tie LOW.
        .TRG_RST             ( fiber_reset),         // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
        //.ENA_TEST_PAT        ( sw[8]),               // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
        .ENA_TEST_PAT        (1'b0),                 // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
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

    dcfeb_fiber_out  dcfeb2out (
        .RST                 ( reset),               // Manual only
        .TRG_SIGDET          (),                      // from IPAD to IBUF.  N/A?
        .TRG_RX_N            ( ),                    // empty
        .TRG_RX_P            ( ),                    // empty
        .TRG_TDIS            ( ),                    // OBUF output, for what?  N/A?
        .TRG_TX_N            ( txn[2]),              // pick a fiber
        .TRG_TX_P            ( txp[2]),              // pick a fiber
        .G1C                 ( cfeb_fiber_out[1][7:0]),   // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
        .G2C                 ( cfeb_fiber_out[1][15:8]),  // if ENA_TEST_PAT then it's prbs so these don't matter...
        .G3C                 ( cfeb_fiber_out[1][23:16]), // but good for testing low-rate non-zero triad data:
        .G4C                 ( cfeb_fiber_out[1][31:24]),
        .G5C                 ( cfeb_fiber_out[1][39:32]),
        .G6C                 ( cfeb_fiber_out[1][47:40]),
        .TRG_TX_REFCLK       ( ck160),               // QPLL 160 from MGT clk
        .TRG_TXUSRCLK        ( snap_clk2),           // get 160 from TXOUTCLK (times 2)
        .TRG_CLK80           ( tx_clk),              // get 80 from TXOUTCLK
        .TRG_GTXTXRST        ( 1'b0),                // maybe Manual "reset" only
        .TRG_TX_PLLRST       ( 1'b0),                // Tie LOW.
        .TRG_RST             ( fiber_reset),         // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
        //.ENA_TEST_PAT        ( sw[8]),               // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
        .ENA_TEST_PAT        (1'b0),                 // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
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

    dcfeb_fiber_out  dcfeb3out (
        .RST                 ( reset),               // Manual only
        .TRG_SIGDET          (),                      // from IPAD to IBUF.  N/A?
        .TRG_RX_N            ( ),                    // empty
        .TRG_RX_P            ( ),                    // empty
        .TRG_TDIS            ( ),                    // OBUF output, for what?  N/A?
        .TRG_TX_N            ( txn[3]),              // pick a fiber
        .TRG_TX_P            ( txp[3]),              // pick a fiber
        .G1C                 ( cfeb_fiber_out[2][7:0]),   // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
        .G2C                 ( cfeb_fiber_out[2][15:8]),  // if ENA_TEST_PAT then it's prbs so these don't matter...
        .G3C                 ( cfeb_fiber_out[2][23:16]), // but good for testing low-rate non-zero triad data:
        .G4C                 ( cfeb_fiber_out[2][31:24]),
        .G5C                 ( cfeb_fiber_out[2][39:32]),
        .G6C                 ( cfeb_fiber_out[2][47:40]),
        .TRG_TX_REFCLK       ( ck160),               // QPLL 160 from MGT clk
        .TRG_TXUSRCLK        ( snap_clk2),           // get 160 from TXOUTCLK (times 2)
        .TRG_CLK80           ( tx_clk),              // get 80 from TXOUTCLK
        .TRG_GTXTXRST        ( 1'b0),                // maybe Manual "reset" only
        .TRG_TX_PLLRST       ( 1'b0),                // Tie LOW.
        .TRG_RST             ( fiber_reset),         // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
        //.ENA_TEST_PAT        ( sw[8]),               // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
        .ENA_TEST_PAT        (1'b0),                 // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
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

    dcfeb_fiber_out  dcfeb4out (
        .RST                 ( reset),               // Manual only
        .TRG_SIGDET          (),                      // from IPAD to IBUF.  N/A?
        .TRG_RX_N            ( ),                    // empty
        .TRG_RX_P            ( ),                    // empty
        .TRG_TDIS            ( ),                    // OBUF output, for what?  N/A?
        .TRG_TX_N            ( txn[4]),              // pick a fiber
        .TRG_TX_P            ( txp[4]),              // pick a fiber
        .G1C                 ( cfeb_fiber_out[3][7:0]),   // Comp data for TX to TMB...use to send a low-rate pattern on !sw8 & sw7 & !PB
        .G2C                 ( cfeb_fiber_out[3][15:8]),  // if ENA_TEST_PAT then it's prbs so these don't matter...
        .G3C                 ( cfeb_fiber_out[3][23:16]), // but good for testing low-rate non-zero triad data:
        .G4C                 ( cfeb_fiber_out[3][31:24]),
        .G5C                 ( cfeb_fiber_out[3][39:32]),
        .G6C                 ( cfeb_fiber_out[3][47:40]),
        .TRG_TX_REFCLK       ( ck160),               // QPLL 160 from MGT clk
        .TRG_TXUSRCLK        ( snap_clk2),           // get 160 from TXOUTCLK (times 2)
        .TRG_CLK80           ( tx_clk),              // get 80 from TXOUTCLK
        .TRG_GTXTXRST        ( 1'b0),                // maybe Manual "reset" only
        .TRG_TX_PLLRST       ( 1'b0),                // Tie LOW.
        .TRG_RST             ( fiber_reset),         // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
        //.ENA_TEST_PAT        ( sw[8]),               // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
        .ENA_TEST_PAT        (1'b0),                 // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
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

    gem_fiber_out  gem1out   (
        .RST                 (reset),                 // Manual only
        .TRG_SIGDET          (),                      // from IPAD to IBUF.  N/A?
        .TRG_RX_N            (),                      // empty
        .TRG_RX_P            (),                      // empty
        .TRG_TDIS            (),                      // OBUF output, for what?  N/A?
        .TRG_TX_N            (txn[5]),                // pick a fiber
        .TRG_TX_P            (txp[5]),                // pick a fiber

        .GEM_DATA            (gem_fiber_out[1][55:0 ]),
        .GEM_OVERFLOW        (1'b0),

        .TRG_TX_REFCLK       (ck160),                 // QPLL 160 from MGT clk
        .TRG_TXUSRCLK        (snap_clk2),             // get 160 from TXOUTCLK (times 2)
        .TRG_CLK80           (tx_clk),                // get 80 from TXOUTCLK
        .TRG_GTXTXRST        (1'b0     ),             // maybe Manual "reset" only
        .TRG_TX_PLLRST       (txpll_rst),             // Tie LOW.
        .TRG_RST             (fiber_reset),           // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
        .ENA_TEST_PAT        (1'b0),                 // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
        //.ENA_TEST_PAT        (sw[8]),                 // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
        .INJ_ERR             (ferr_f[5] & sw[8]),     // use my switch/PB combo logic for this, high-true? Pulse high once.
        .TRG_SD              (),                      // from IBUF, useless output. N/A
        .TRG_TXOUTCLK        (),                      // 80 MHz; This has to go to MCM to generate 160/80
        .TRG_TX_PLL_LOCK     (),                      // inverse holds the MCM in Reset                                                                                                                                                                       // Tx GTX PLL Ref lock
        .TRG_TXRESETDONE     (),                      // N/A
        .TX_SYNC_DONE        (),                      // not used in DCFEB tests
        .STRT_LTNCY          (tx_begin[5]),           // after every Reset, to TP for debug only  -- !sw7 ?
        .LTNCY_TRIG          (tx_fc[5]),              // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
        .MON_TX_SEL          (),                      // N/A
        .MON_TRG_TX_ISK      (),                      // N/A returns 4 bits
        .MON_TRG_TX_DATA     ()                       // N/A returns 32 bits
    );

    gem_fiber_out  gem2out   (
        .RST                 (reset),                 // Manual only
        .TRG_SIGDET          (),                      // from IPAD to IBUF.  N/A?
        .TRG_RX_N            (),                      // empty
        .TRG_RX_P            (),                      // empty
        .TRG_TDIS            (),                      // OBUF output, for what?  N/A?
        .TRG_TX_N            (txn[6]),                // pick a fiber
        .TRG_TX_P            (txp[6]),                // pick a fiber

        .GEM_DATA            (gem_fiber_out[2][55:0 ]),
        .GEM_OVERFLOW        (1'b0),

        .TRG_TX_REFCLK       (ck160),                 // QPLL 160 from MGT clk
        .TRG_TXUSRCLK        (snap_clk2),             // get 160 from TXOUTCLK (times 2)
        .TRG_CLK80           (tx_clk),                // get 80 from TXOUTCLK
        .TRG_GTXTXRST        (1'b0     ),             // maybe Manual "reset" only
        .TRG_TX_PLLRST       (txpll_rst),             // Tie LOW.
        .TRG_RST             (fiber_reset),           // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
        .ENA_TEST_PAT        (1'b0),                 // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
        //.ENA_TEST_PAT        (sw[8]),                 // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
        .INJ_ERR             (ferr_f[6] & sw[8]),     // use my switch/PB combo logic for this, high-true? Pulse high once.
        .TRG_SD              (),                      // from IBUF, useless output. N/A
        .TRG_TXOUTCLK        (),                      // 80 MHz; This has to go to MCM to generate 160/80
        .TRG_TX_PLL_LOCK     (),                      // inverse holds the MCM in Reset                                                                                                                                                                       // Tx GTX PLL Ref lock
        .TRG_TXRESETDONE     (),                      // N/A
        .TX_SYNC_DONE        (),                      // not used in DCFEB tests
        .STRT_LTNCY          (tx_begin[6]),           // after every Reset, to TP for debug only  -- !sw7 ?
        .LTNCY_TRIG          (tx_fc[6]),              // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
        .MON_TX_SEL          (),                      // N/A
        .MON_TRG_TX_ISK      (),                      // N/A returns 4 bits
        .MON_TRG_TX_DATA     ()                       // N/A returns 32 bits
    );

    gem_fiber_out  gem3out   (
        .RST                 (reset),                 // Manual only
        .TRG_SIGDET          (),                      // from IPAD to IBUF.  N/A?
        .TRG_RX_N            (),                      // empty
        .TRG_RX_P            (),                      // empty
        .TRG_TDIS            (),                      // OBUF output, for what?  N/A?
        .TRG_TX_N            (txn[7]),                // pick a fiber
        .TRG_TX_P            (txp[7]),                // pick a fiber

        .GEM_DATA            (gem_fiber_out[3][55:0 ]),
        .GEM_OVERFLOW        (1'b0),

        .TRG_TX_REFCLK       (ck160),                 // QPLL 160 from MGT clk
        .TRG_TXUSRCLK        (snap_clk2),             // get 160 from TXOUTCLK (times 2)
        .TRG_CLK80           (tx_clk),                // get 80 from TXOUTCLK
        .TRG_GTXTXRST        (1'b0     ),             // maybe Manual "reset" only
        .TRG_TX_PLLRST       (txpll_rst),             // Tie LOW.
        .TRG_RST             (fiber_reset),           // gtx_reset =  PBrst | !TxSyncDone | !RxSyncDone
        .ENA_TEST_PAT        (1'b0),                 // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
        //.ENA_TEST_PAT        (sw[8]),                 // HIGH for PRBS! (Low will send data from GxC registers)  Use This Later, send low-rate pattern.
        .INJ_ERR             (ferr_f[7] & sw[8]),     // use my switch/PB combo logic for this, high-true? Pulse high once.
        .TRG_SD              (),                      // from IBUF, useless output. N/A
        .TRG_TXOUTCLK        (),                      // 80 MHz; This has to go to MCM to generate 160/80
        .TRG_TX_PLL_LOCK     (),                      // inverse holds the MCM in Reset                                                                                                                                                                       // Tx GTX PLL Ref lock
        .TRG_TXRESETDONE     (),                      // N/A
        .TX_SYNC_DONE        (),                      // not used in DCFEB tests
        .STRT_LTNCY          (tx_begin[7]),           // after every Reset, to TP for debug only  -- !sw7 ?
        .LTNCY_TRIG          (tx_fc[7]),              // bring out to TP.  Signals when TX sends "FC" (once every 128 BX).  Send raw to TP  --sw8,7
        .MON_TX_SEL          (),                      // N/A
        .MON_TRG_TX_ISK      (),                      // N/A returns 4 bits
        .MON_TRG_TX_DATA     ()                       // N/A returns 32 bits
    );

    wire fiberout_sump = (|tx_begin[7:0]) | (|tx_fc[7:0]) |synced_snapt | t12_fault | r12_fok;

//----------------------------------------------------------------------------------------------------------------------
// LED Assignments
//----------------------------------------------------------------------------------------------------------------------

x_flashsm #(22) led0 (.trigger(chm_adr==1),         .hold(1'b0), .clock(gbe_txclk2), .out(loading_bram_led));
x_flashsm #(22) led1 (.trigger(cmd_code==CMD_DUMP),   .hold(1'b0), .clock(gbe_txclk2), .out(dump_led));
x_flashsm #(22) led2 (.trigger(cmd_code==CMD_WRITE),  .hold(1'b0), .clock(gbe_txclk2), .out(cmd_code_led));
x_flashsm #(22) led3 (.trigger(chm_fb==1), .hold(1'b0), .clock(gbe_txclk2), .out(rxdat_led));
x_flashsm #(22) led4 (.trigger(gbe_rxcount>16'd4 && cmd_code==CMD_WRITE && bk_adr<MXBRAMS && gbe_rxcount==16'h5), .hold(1'b0), .clock(gbe_txclk2), .out(loading_bram_led2));
x_flashsm #(22) led5 (.trigger(gbe_rxcount>16'd4 && cmd_code==CMD_WRITE && bk_adr<MXBRAMS && rx_adr==11'h7ff), .hold(1'b0), .clock(gbe_txclk2), .out(loading_bram_done));

    wire sump = gbe_sump | fiberout_sump | clk_sump | gem_sump;

    always @(*)
    begin                               // JG, v1p16: swap LED 3<>4, notes and all
        led_low[0] =  gem_sump;
        led_low[1] =  1'b0;  // 1'b0;
        led_low[2] =  1'b0;             // qpll_lock;                              // always ON!   was !_ccb_rx[31] == _alct_adb_pulse_async
        led_low[3] =  1'b0;             // qpll_lock_lost;                         // always OFF?  was _ccb_rx[35] == mpc_in1, always ON
        led_low[4] =  1'b0;             // lhc_clk;
        led_low[5] =  1'b0;             // ck160_locked;                           // always ON!                                              // Tx GTX PLL Ref lock
        led_low[6] =  1'b0;             // 1'b0;                                   // --> ON!  comes from mmcm driven by tx_pll_ck160 in FPGA
        led_low[7] =  1'b0;             // lhc_clk;                                // just reset, includes ccb_rx[1]==L1reset

        led_hi[8]  = ~ (1'b0 ^ loading_bram_led  ); //!(hold_bit | gtx_reset) ; // M1: synced with PB & errors at crate Rx
        led_hi[9]  = ~ (1'b0 ^ dump_led          ); // qpll_lock_lost         ; // 0
        led_hi[10] = ~ (1'b0 ^ cmd_code_led      ); //!cmd_code[7]            ; // !gtx_reset                              ; // 0
        led_hi[11] = ~ (1'b0 ^ rxdat_led         ); //!rd_ptr[8]              ; // ~6.25 usec  // gtx_ready                ; // 1
        led_hi[12] = ~ (1'b0 ^ loading_bram_led2 ); //!dump_enable_rr         ; // 12.5 usec   //
        led_hi[13] = ~ (1'b0 ^ loading_bram_done ); //!dump_enable_r          ; // 12.5 usec   // (locked & lhc_locked)    ; // 1
        led_hi[14] = ~ (1'b0 ^ packing              ); //!dump_enable            ; // 12.5 usec
        led_hi[15] = ~ (1'b0 ^ loading_bram_led  ); //!dump_done              ; // 50ns

        test_led[9]   = lhc_ck;
        test_led[10]  = 1'b0;

    end

//----------------------------------------------------------------------------------------------------------------------
endmodule
//----------------------------------------------------------------------------------------------------------------------
