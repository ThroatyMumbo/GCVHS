// SMPTE LTC (30 fps) off the right line-in into a field count; intervals in codec frames (~20.4/bit).
// A word ends with its frame, so the nearest field start begins the next: A/V skew under half a field is absorbed.
module ltc_rx #(
    parameter signed [15:0] HYST = 16'sd1024,  // Schmitt threshold either side of zero
    parameter [4:0]  SHORT_MIN = 5'd5,
    parameter [4:0]  LONG_MIN  = 5'd15,
    parameter [4:0]  LONG_MAX  = 5'd27,
    parameter [19:0] FIELD     = 20'd834168,   // CLOCK_50 cycles per 59.94 Hz field
    parameter [5:0]  HOLD      = 6'd30         // fields an agreeing word keeps `locked` up
) (
    input  wire               clk,             // CLOCK_50
    input  wire               rst,
    input  wire               x_valid,
    input  wire signed [15:0] x,
    input  wire               field_start,
    output reg  [23:0]        field = 24'd0,   // current field: 2 * frame + field within it
    output reg  [31:0]        tc = 32'd0,      // BCD {hh, mm, ss, ff} of the current field's frame
    output wire               locked,
    output reg  [31:0]        user = 32'd0,    // user bits of the last good word
    output reg  [15:0]        words = 16'd0,   // good words
    output reg  [15:0]        bad = 16'd0,     // sync words closing a damaged word
    output reg  [15:0]        jumps = 16'd0,   // flywheel reloads
    output reg  [19:0]        phase = 20'd0,   // last good word's end, CLOCK_50 cycles after a field start
    output wire [15:0]        peak             // largest |x| over the last second
);
    `include "timecode.vh"
    function [8:0] bump(input [7:0] d, input [7:0] top);  // {carry, next} of a two-digit BCD field
        bump = d == top ? 9'h100 : d[3:0] == 4'd9 ? {1'b0, d[7:4] + 4'd1, 4'd0} : {1'b0, d[7:4], d[3:0] + 4'd1};
    endfunction
    function [31:0] tc_inc(input [31:0] t);
        reg [8:0] f, s, m, h;
        begin
            f = bump(t[7:0], 8'h29);
            s = bump(t[15:8], 8'h59);
            m = bump(t[23:16], 8'h59);
            h = bump(t[31:24], 8'h23);
            tc_inc = {f[8] && s[8] && m[8] ? h[7:0] : t[31:24], f[8] && s[8] ? m[7:0] : t[23:16],
                      f[8] ? s[7:0] : t[15:8], f[7:0]};
        end
    endfunction

    // Bits: two short intervals are a 1, one long a 0. `clean` counts bits since the last bad interval.
    reg               lvl = 1'b0, half = 1'b0;
    reg  [4:0]        cnt = 5'd0;
    reg  [79:0]       sr = 80'd0;
    reg  [6:0]        clean = 7'd0;
    wire [4:0]        iv   = cnt == 5'd31 ? cnt : cnt + 1'b1;
    wire              xing = x_valid && (lvl ? x < -HYST : x > HYST);
    wire              is_short = iv >= SHORT_MIN && iv < LONG_MIN;
    wire              is_long  = iv >= LONG_MIN && iv <= LONG_MAX;
    wire              shift    = xing && (is_long || (is_short && half));
    wire [79:0]       sr_n     = {is_long ? 1'b0 : 1'b1, sr[79:1]};
    wire [6:0]        clean_n  = is_long && half ? 7'd0 : clean == 7'd80 ? clean : clean + 1'b1;
    wire              sync_end = shift && sr_n[79:64] == 16'hBFFC;

    reg               w1 = 1'b0;
    reg  [31:0]       w_bcd;
    always @(posedge clk) begin
        w1 <= 1'b0;
        if (rst) begin
            half  <= 1'b0;
            clean <= 7'd0;
            words <= 16'd0;
            bad   <= 16'd0;
        end else if (x_valid) begin
            cnt <= xing ? 5'd0 : iv;
            if (xing) lvl <= ~lvl;
            if (xing && !is_short && !is_long) begin
                half  <= 1'b0;
                clean <= 7'd0;
            end else if (xing && is_short && !half) half <= 1'b1;
            else if (shift) begin
                half  <= 1'b0;
                sr    <= sr_n;
                clean <= clean_n;
                if (sync_end) begin
                    if (clean_n == 7'd80 && tc_digits_ok(sr_n[63:0]) && !(^sr_n)) begin
                        w1      <= 1'b1;
                        w_bcd   <= tc_bcd(sr_n[63:0]);
                        user    <= tc_user(sr_n[63:0]);
                        words   <= words + (words != 16'hFFFF);
                    end else bad <= bad + (bad != 16'hFFFF);
                end
            end
        end
    end

    // The word's frame number; w_bcd holds until the next good word.
    reg  [2:0]  wp = 3'd0;                     // w1 through the tc_frames stages
    reg         w2 = 1'b0;
    wire [21:0] w_frame;
    always @(posedge clk) {w2, wp} <= {wp, w1};
    tc_frames w_frames (.clk(clk), .tc(w_bcd), .frame(w_frame));

    // Flywheel, stepping for HOLD fields after an agreeing word; a disagreeing one drops `locked` until the next
    // agrees with the counter, or with it (a reload). A word on a field start's cycle is taken on the next.
    reg  [19:0] since = 20'd0;                 // at the take: a field start during the decode reads as just after it
    reg  [23:0] pend = 24'd0;                  // a disagreeing word's field, advanced with the fields
    reg         pend_ok = 1'b0, w3 = 1'b0;
    reg  [5:0]  age = HOLD;                    // fields since a word agreed
    wire        run = age < HOLD;
    assign locked = run && !pend_ok;
    wire        take     = (w2 || w3) && !field_start;
    wire        at_start = since < FIELD / 2;
    wire [23:0] cand     = {1'b0, w_frame, 1'b0} + (at_start ? 24'd2 : 24'd1);
    always @(posedge clk) begin
        since <= field_start ? 20'd0 : since + (since != 20'hFFFFF);
        w3    <= (w2 || w3) && field_start;
        if (rst) begin
            field   <= 24'd0;
            tc      <= 32'd0;
            pend_ok <= 1'b0;
            age     <= HOLD;
            jumps   <= 16'd0;
        end else if (field_start) begin
            pend <= pend + 1'b1;
            age  <= age + (age != HOLD);
            if (run) begin                     // no word for HOLD fields (a paused or cueing deck mutes its Hi-Fi audio): hold
                field <= field + 1'b1;
                if (field[0]) tc <= tc_inc(tc);
            end
        end else if (take) begin
            phase <= since;
            if (cand == field) begin
                age     <= 6'd0;
                pend_ok <= 1'b0;
            end else if (pend_ok && cand == pend) begin
                field   <= cand;
                tc      <= at_start ? tc_inc(w_bcd) : w_bcd;
                jumps   <= jumps + (jumps != 16'hFFFF);
                age     <= 6'd0;
                pend_ok <= 1'b0;
            end else begin
                pend    <= cand;
                pend_ok <= 1'b1;
            end
        end
    end

    peak_meter meter (.clk(clk), .en(x_valid), .x(x), .peak(peak));
endmodule
