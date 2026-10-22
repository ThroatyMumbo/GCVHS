// SMPTE timecode word fields, the 64 data bits LTC and VITC share.
function tc_digits_ok(input [63:0] w);  // the CRC alone misses two errors in one column
    tc_digits_ok = w[3:0] <= 4'd9 && w[9:8] <= 2'd2 && w[19:16] <= 4'd9 && w[26:24] <= 3'd5 &&
                   w[35:32] <= 4'd9 && w[42:40] <= 3'd5 && w[51:48] <= 4'd9 && {w[57:56], w[51:48]} <= 6'h23;
endfunction
function [31:0] tc_bcd(input [63:0] w);     // BCD {hh, mm, ss, ff}
    tc_bcd = {2'b0, w[57:56], w[51:48], 1'b0, w[42:40], w[35:32], 1'b0, w[26:24], w[19:16], 2'b0, w[9:8], w[3:0]};
endfunction
function [31:0] tc_user(input [63:0] w);
    tc_user = {w[63:60], w[55:52], w[47:44], w[39:36], w[31:28], w[23:20], w[15:12], w[7:4]};
endfunction
