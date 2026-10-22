// SMPTE VITC (vcr_sim/src/streams.c vitc_make) off one luma line: 9 groups of sync 1 0 + 8 bits LSB first,
// cells of H / 115 (7.461 Y samples) read mid-cell, re-timed on each group's 1-0 edge; CRC x^8 + 1 leaves columns even.
module vitc_slicer #(
    parameter [7:0]  THR  = 8'd100,     // between blanking and 80 IRE
    parameter [17:0] CELL = 18'd955     // Y samples per bit, in 1/128
) (
    input  wire        clk,
    input  wire        start,           // SAV
    input  wire        y_en,
    input  wire [7:0]  y,
    input  wire        done,            // EAV
    output reg         ok,              // valid with done's next cycle
    output reg  [63:0] data,            // groups 1-8, bit i of group g at 8 (g-1) + i: the LTC's layout
    output reg  [2:0]  why,             // with ok: 0 ok, 1 no edge, 2 cells missing, 3 sync, 4 CRC, 5 digits
    output reg  [3:0]  why_grp,         // first group with a bad sync, 15 if none
    output reg         like             // the first white run is under 2 cells: VITC, not the tag
);
    `include "timecode.vh"
    reg         seen, hi_d, sync_ok, in_run;
    reg  [5:0]  run;
    reg  [3:0]  grp, bad_grp;
    reg  [10:0] cur;                    // index of the last sample, from the first high one
    reg  [17:0] nxt;                    // next cell centre, 1/128 sample
    reg  [6:0]  k;                      // cells read
    reg  [3:0]  km10;                   // k mod 10
    reg  [7:0]  col;                    // XOR of each bit column k mod 8
    wire        hi     = y > THR;
    wire [17:0] half   = {1'b0, CELL[17:1]};
    wire [17:0] now    = {cur + 1'b1, 7'd0};
    wire [17:0] fall   = now - 18'd64 + half;       // centre of the cell a 1-0 edge just before now starts
    wire [17:0] off    = fall > nxt ? fall - nxt : nxt - fall;
    wire        resync = km10 == 4'd1 && hi_d && !hi && off < half;
    wire [17:0] due    = resync ? fall : nxt;
    wire        digits_ok = tc_digits_ok(data);

    always @(posedge clk) begin
        if (start) begin
            seen <= 1'b0;
            k    <= 7'd0;
        end else if (y_en) begin
            hi_d <= hi;
            if (!seen) begin
                if (hi) begin
                    seen    <= 1'b1;
                    cur     <= 11'd0;
                    nxt     <= half - 18'd64;
                    km10    <= 4'd0;
                    col     <= 8'd0;
                    sync_ok <= 1'b1;
                    in_run  <= 1'b1;
                    run     <= 6'd1;
                    grp     <= 4'd0;
                    bad_grp <= 4'd15;
                end
            end else if (k != 7'd90) begin
                cur <= cur + 1'b1;
                if (in_run && hi) run <= run + (run != 6'h3F);
                else in_run <= 1'b0;
                nxt <= due;
                if (now + 18'd64 > due) begin
                    nxt  <= due + CELL;
                    k    <= k + 1'b1;
                    km10 <= km10 == 4'd9 ? 4'd0 : km10 + 1'b1;
                    if (km10 == 4'd9) grp <= grp + 1'b1;
                    col[k[2:0]] <= col[k[2:0]] ^ hi;
                    if ((km10 == 4'd0 && !hi) || (km10 == 4'd1 && hi)) begin
                        sync_ok <= 1'b0;
                        if (bad_grp == 4'd15) bad_grp <= grp;
                    end
                    if (km10 >= 4'd2 && k < 7'd80) data <= {hi, data[63:1]};
                end
            end
        end
        if (done) begin
            ok      <= seen && k == 7'd90 && sync_ok && col == 8'd0 && digits_ok;
            why     <= !seen ? 3'd1 : k != 7'd90 ? 3'd2 : !sync_ok ? 3'd3 : col != 8'd0 ? 3'd4 : !digits_ok ? 3'd5 : 3'd0;
            why_grp <= seen ? bad_grp : 4'd15;
            like    <= seen && run < 6'd15;
        end
    end
endmodule
