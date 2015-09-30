/*  JRG, 25 Jan 2015
Count how many bits are High in "sbit" as fast as possible, and also indicate if it's 5 or less.
The result is ready after a single clock (one clock latency) for any 24-bit pattern.
This is set to count the High bits in a 24-bit argument, but up to 36 should perform similarly with properly modified code.
Note... not sure how fast it can run, but probably faster than 40 MHz...
*/

module fast_count  (
    lhc_clock,
    sbit,
    count,
    lte_n,
    lte_threshold
);

input lhc_clock;
input  [23:0] sbit;
input  lte_threshold;

output [5:0]  count = fast_sum;
output lte_n = less_or_equal_n;

reg  [5:0]   fast_sum = 0;
reg      less_or_equal_n = 0;
wire [2:0]   sa = fast6count(sbit[5:0]);
wire [2:0]   sb = fast6count(sbit[11:6]);
wire [2:0]   sc = fast6count(sbit[17:12]);
wire [2:0]   sd = fast6count(sbit[23:18]);
//   wire [2:0]      se = fast6count(sbit[29:24]);
//   wire [2:0]      sf = fast6count(sbit[35:30]);
wire [3:0]   absum = sa + sb;
wire [3:0]   cdsum = sc + sd;
//   wire [3:0]      efsum = se + sf;


always @(posedge lhc_clock) begin
    fast_sum <= absum + cdsum;   // Adr 186 --JGhere, test: count bits in phaser_lock + alct_load_cfg
    less_or_equal_n <= ((absum + cdsum) <= lte_threshold);
    //      fast_sum <= absum + cdsum + efsum;   // Adr 186 --JGhere, test: count bits in phaser_lock + alct_load_cfg
    //      less_or_equal_n <= ((absum + cdsum + efsum) <= N);
end


function [2:0] fast6count;  // do a fast count of 6 bits with just 3 LUTs (the best you can do in a single logic step)
    input [5:0] s;
begin
    fast6count[0] = ^s[5:0];   // an odd number of bits are High, will always turn on the lowest bit of the counter

    fast6count[1] =
    s==6'b111111 |  // all 6 bits are High, or exactly 3, or exactly 2
    // set 3 out of 6, 20 ways:
    ( s==6'b000111 | s==6'b111000 | s==6'b001011 | s==6'b001101 | s==6'b001110  |  s==6'b010011 | s==6'b010101 | s==6'b010110  |  s==6'b100011 | s==6'b100101 | s==6'b100110 | s==6'b011001 | s==6'b011010 | s==6'b011100  |  s==6'b101001 | s==6'b101010 | s==6'b101100  |  s==6'b110001 | s==6'b110010 | s==6'b110100 ) |
    // set 2 out of 6, 15 ways:
    ( s==6'b000011 | s==6'b000101 | s==6'b000110  |  s==6'b001001 | s==6'b001010 | s==6'b001100  | s==6'b010001 | s==6'b010010 | s==6'b010100 | s==6'b011000 | s==6'b100001 | s==6'b100010 | s==6'b100100 | s==6'b101000 | s==6'b110000 );

    fast6count[2] = s==6'b111111 |  // all 6 are High, or exactly 4, or exactly 5
    // set 4 out of 6, 15 ways:
    ( s==6'b111100 | s==6'b111010 | s==6'b111001  |  s==6'b110110 | s==6'b110101 | s==6'b110011  | s==6'b101110 | s==6'b101101 | s==6'b101011 | s==6'b100111 | s==6'b011110 | s==6'b011101 | s==6'b011011 | s==6'b010111 | s==6'b001111 ) |
    // set 5 out of 6, just 6 ways:
    ( s==6'b111110 | s==6'b111101 | s==6'b111011  |  s==6'b110111 | s==6'b101111 | s==6'b011111 );
end
endfunction

//------------------------------------------------------------------------------------------------------------------
endmodule
//------------------------------------------------------------------------------------------------------------------
