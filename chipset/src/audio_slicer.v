// In-line audio (vcr_sim/src/streams.c, draw_audio): one sample of stream sel per stored line with a pilot,
// crossfaded over 32 lines on a switch, pops replaced by the 3-line median, mu-law expanded if expand.
module audio_slicer #(
    parameter integer    CELLS = 8,
    parameter integer    N     = 6,         // of 11.3 samples per cell: 8 let the neighbours in at -46 dB
    parameter [15:0]     FIRST = 16'd3361,  // Q8 DE1 luma samples after SAV: cell 0's window, + 0.5 to round
    parameter [15:0]     STEP  = 16'd2898,  // Q8: 6 GCV2 pixels (12 DAC samples)
    parameter [11:0]     P_THR = 12'd720,   // pilot sum: 4 samples above 180
    parameter [10:0]     MID   = 11'd792,   // cell sum at 55 IRE, near enough: the DC block takes the rest
    parameter [15:0]     POP   = 16'd8192,  // off the 3-line median by more: a dropout. 1024 chewed up speech
    parameter [15:0]     FULL  = 16'd5930,  // int8 127 at aud_level 300, measured off GCV2 tones (mu_expand.mem too)
    parameter integer    CLK_HZ = 27_000_000
) (
    input  wire              clk,
    input  wire              rst,
    input  wire              start,         // SAV of a stored line
    input  wire              y_en,
    input  wire [7:0]        y,
    input  wire              done,          // EAV of a stored line
    input  wire              field,         // SAV of a field's first stored line
    input  wire [1:0]        n_log2,        // cells on the line: 1 << n_log2, the pilot right after them
    input  wire [2:0]        sel,
    input  wire              expand,
    output reg               valid,
    output reg signed [15:0] sample,
    output reg  [15:0]       rate,          // samples in the last second
    output reg  [15:0]       no_pilot,      // stored lines without a pilot in the last second
    output reg               field_tick,    // pulse at each field's start: field_good holds the last field's verdict
    output reg               field_good,    // nearly every stored line of the last field had a pilot
    output reg  [10:0]       cell0,         // last line's cell 0 and pilot sums, for position checks
    output reg  [11:0]       pilot,
    output wire              line_v,        // pulse: cells holds a line with a pilot, until the next start
    output wire [11*CELLS-1:0] cells        // every cell's raw sum, cell 0 in [10:0]
);
    reg  [9:0]  n;
    reg  [15:0] pos;
    reg  [3:0]  c;
    reg  [2:0]  m;
    reg  [10:0] acc;
    reg  [11:0] pacc;
    reg  [10:0] sums [0:CELLS-1];
    genvar gi;
    generate for (gi = 0; gi < CELLS; gi = gi + 1) begin : g_cells
        assign cells[11*gi +: 11] = sums[gi];
    end endgenerate
    // First of 4 pilot samples for 1, 2, 4, 8 cells: a sample ahead of where the next cell's window would start.
    localparam [9:0] PIL1 = ((FIRST + 1 * STEP) >> 8) - 1, PIL2 = ((FIRST + 2 * STEP) >> 8) - 1,
                     PIL4 = ((FIRST + 4 * STEP) >> 8) - 1, PIL8 = ((FIRST + 8 * STEP) >> 8) - 1;
    reg  [9:0]  pil;
    always @* case (n_log2)
        2'd0:    pil = PIL1;
        2'd1:    pil = PIL2;
        2'd2:    pil = PIL4;
        default: pil = PIL8;
    endcase

    always @(posedge clk)
        if (start) begin
            n    <= 10'd0;
            pos  <= FIRST;
            c    <= 4'd0;
            m    <= 3'd0;
            acc  <= 11'd0;
            pacc <= 12'd0;
        end else if (y_en) begin
            n <= n + 1'b1;
            if (c < CELLS && n >= pos[15:8]) begin
                if (m == N - 1) begin
                    sums[c] <= acc + y;
                    acc     <= 11'd0;
                    c       <= c + 1'b1;
                    pos     <= pos + STEP;
                    m       <= 3'd0;
                end else begin
                    acc <= acc + y;
                    m   <= m + 1'b1;
                end
            end
            if (n >= pil && n < pil + 10'd4) pacc <= pacc + y;
        end

    reg  [2:0]  from = 3'd0, to = 3'd0;
    reg  [5:0]  alpha = 6'd32;              // 0..32: weight of stream to
    reg         d1, d2, ok;
    reg  signed [15:0] a, b;
    wire signed [22:0] mix = a * 23'sd32 + (b - a) * $signed({1'b0, alpha});
    wire        swap = sel != to && alpha == 6'd32;  // a fade runs to the end; sel is taken after it
    wire [2:0]  a_s  = swap ? to : from;    // the pair this line plays, so a switch starts from to, not from
    wire [2:0]  b_s  = swap ? sel : to;
    function signed [15:0] med3(input signed [15:0] p, input signed [15:0] q, input signed [15:0] r);
        reg signed [15:0] lo, hi;
        begin
            lo   = p < q ? p : q;
            hi   = p < q ? q : p;
            med3 = r < lo ? lo : r > hi ? hi : r;
        end
    endfunction

    reg                d3, lin_v;
    reg  signed [15:0] raw, h0, h1, lin;       // lin: the pop-filtered sample, before any expansion
    wire signed [15:0] md  = med3(raw, h0, h1);
    wire signed [16:0] dev = h0 - md;
    assign line_v = d2 && ok;
    always @(posedge clk) begin
        {d3, d2, d1} <= {d2, d1, done};
        lin_v    <= d3 && ok;
        if (done) begin
            ok    <= pacc > P_THR;
            cell0 <= sums[0];
            pilot <= pacc;
            a     <= $signed({5'b0, sums[a_s]} - {5'b0, MID}) <<< 4;
            b     <= $signed({5'b0, sums[b_s]} - {5'b0, MID}) <<< 4;
            if (swap) begin
                from  <= to;
                to    <= sel;
                alpha <= 6'd0;
            end else if (alpha != 6'd32) alpha <= alpha + 1'b1;
        end
        if (d2 && ok) raw <= mix >>> 5;
        if (d3) begin
            h0     <= raw;
            h1     <= h0;
            lin    <= dev > $signed({1'b0, POP}) || dev < -$signed({1'b0, POP}) ? md : h0;
        end
    end

    // Expander: take the compressed-domain DC out (~1.2 s), then |x| / FULL through a 257-point table,
    // interpolated. The linear path is delayed to match.
    reg  signed [15:0] l1, l2, l3;
    reg  [15:0]        rom [0:256];
    initial $readmemh("src/mu_expand.mem", rom);   // not .hex: Quartus reads that as Intel HEX
    reg  signed [23:0] dc;                              // Q8
    reg  signed [16:0] xc;
    reg  [2:0]         ev;
    reg                neg, neg2;
    reg  [13:0]        t;                               // |x| in table steps, Q5
    reg  [4:0]         fr;
    reg  [15:0]        lo, hi;
    wire [16:0]        ax  = xc[16] ? -xc : xc;
    wire [15:0]        au  = ax > FULL ? FULL : ax[15:0];
    localparam [13:0]  T_SCALE = (256 * 32 * 8192 - 1) / FULL;  // 256 * 32 / FULL in Q13, so t < 256 * 32
    wire [29:0]        tw  = au * T_SCALE;
    wire [8:0]         ti  = t[13:5];
    wire [16:0]        v   = lo + (({1'b0, hi} - {1'b0, lo}) * fr >> 5);
    always @(posedge clk)
        if (rst) begin
            dc <= 24'sd0;
            ev <= 3'd0;
            valid <= 1'b0;
        end else begin
            ev    <= {ev[1:0], lin_v};
            valid <= ev[2];
            if (lin_v) begin
                xc <= lin - $signed(dc[23:8]);
                l1 <= lin;
                dc <= dc + (($signed({lin, 8'd0}) - dc) >>> 14);
            end
            if (ev[0]) begin
                neg <= xc[16];
                t   <= tw[26:13];
                l2  <= l1;
            end
            if (ev[1]) begin
                lo   <= rom[ti];
                hi   <= rom[ti + 1'b1];
                fr   <= t[4:0];
                neg2 <= neg;
                l3   <= l2;
            end
            if (ev[2]) sample <= !expand ? l3 : neg2 ? -$signed(v[15:0]) : $signed(v[15:0]);
        end

    reg [24:0] sec;
    reg [15:0] rate_w, bad_w;
    always @(posedge clk)
        if (rst) begin
            sec    <= 25'd0;
            rate_w <= 16'd0;
            bad_w  <= 16'd0;
        end else begin
            if (d2) begin
                rate_w <= rate_w + ok;
                bad_w  <= bad_w + !ok;
            end
            if (sec == CLK_HZ - 1) begin
                sec      <= 25'd0;
                rate     <= rate_w;
                no_pilot <= bad_w;
                rate_w   <= 16'd0;
                bad_w    <= 16'd0;
            end else sec <= sec + 1'b1;
        end

    reg [9:0] f_ok, f_bad;
    always @(posedge clk)
        if (rst) begin
            f_ok       <= 10'd0;
            f_bad      <= 10'd0;
            field_tick <= 1'b0;
            field_good <= 1'b0;
        end else begin
            field_tick <= field;
            if (field) begin
                field_good <= f_ok != 10'd0 && f_bad < {4'd0, f_ok[9:4]};
                f_ok       <= 10'd0;
                f_bad      <= 10'd0;
            end else if (d2) begin
                f_ok  <= f_ok + ok;
                f_bad <= f_bad + !ok;
            end
        end
endmodule
