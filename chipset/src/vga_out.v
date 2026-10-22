// 720x480p out, one frame per input field, always 525 lines: frame-locked in play by trimming the line (858 +-4 clocks).
// The stream is scaled into a window, rows fetched two lines ahead; view and window change only at a frame start.
module vga_out (
    input  wire        clk,             // 27 MHz off CLOCK_50; not the ov_wclk port
    input  wire        fin,             // pulse: an input frame start to lock to (the deck plays)
    input  wire [7:0]  view,            // {slot[1:0], zoom x, right, zoom y, bottom, line pick, parity}
    input  wire [9:0]  fx,              // bit 0 up: invert, grey, poster, scanline, rotate, negative, jitter, soft, track, HUD
    input  wire        mask_audio,      // black out source pixels under the in-line audio cells
    input  wire        fit,             // audio cells present: stretch the picture right of them (full or half)
    input  wire [1:0]  aud_n_log2,      // audio cells: 1 << aud_n_log2
    input  wire [19:0] tape,            // HUD tape counter, BCD {H, M tens, M, S tens, S}
    input  wire        hud_cpu,         // HUD values from the game CPU instead of the demo
    input  wire [25:0] hud_vals,        // {score BCD[19:0], lives[1:0], hp[3:0]}
    input  wire        ov_wclk,         // clock of ov_we..pal_data (src/cpu/overlay.v)
    input  wire        ov_we,
    input  wire [16:0] ov_waddr,
    input  wire [7:0]  ov_wdata,
    input  wire        ov_back,
    input  wire        pal_we,
    input  wire [7:0]  pal_addr,
    input  wire [24:0] pal_data,
    input  wire        ov_enable,
    input  wire        ov_front,
    input  wire [9:0]  win_x,           // video window, output pixels
    input  wire [8:0]  win_y,
    input  wire [9:0]  win_w,           // 0: no video
    input  wire [8:0]  win_h,
    input  wire [19:0] win_hstep,       // Q16 full-screen source pixels per output pixel (720 / win_w)
    input  wire [19:0] win_vstep,       // Q16 display rows per output line (240 / win_h)
    input  wire [24:0] win_pan,         // {wrap, inset per side Q8 of the region, pan Q16 of the period}
    input  wire [19:0] ov_hwin,        // overlay columns {w, x}, output pixels
    input  wire [24:0] ov_hstep,        // {first framebuffer x, Q16 framebuffer pixels per output pixel}
    output reg         frame_top,       // pulse: active line 0 begins
    output reg         vblank,
    output reg         fetch,           // pulse: load row fetch_row into read buffer slot fetch_slot
    output reg  [12:0] fetch_row,
    output reg  [1:0]  fetch_slot,
    output reg  [11:0] raddr,
    input  wire [15:0] rdata,           // {Y, C}, one cycle after raddr
    input  wire [7:0]  rdata_p,         // C of pixel raddr ^ 1, the other half of its Cb/Cr pair
    output reg  [7:0]  r,
    output reg  [7:0]  g,
    output reg  [7:0]  b,
    output wire        hs_n,
    output wire        vs_n,
    output wire        blank_n
);
    `include "sdram_map.vh"
    localparam integer H_ACT = 720, H_FP = 16, H_SYNC = 62, H_TOT = 858;
    localparam integer V_ACT = 480, V_FP = 9, V_SYNC = 6;
    localparam integer V_TOT = 525;
    localparam integer V_AIM = 520;     // where an input frame start should land: 5 lines ahead of the frame top
    localparam integer TRIM = 4;        // most clocks a line moves: 0.5%, 2.4 lines of phase per frame
    localparam integer LINES  = 240;
    localparam integer BAR_H  = 24;     // VHS tracking band height, output lines

    reg  [9:0] h, v;
    reg  [7:0] show;                    // view, latched per frame

    // Phase error at each input frame start, lines, wrapped to +-262: late (positive) lengthens our lines.
    // Within a line it holds (fields alternate 262/263); no start for a whole frame drops the trim.
    wire signed [10:0] e0  = $signed({1'b0, v}) - V_AIM;
    wire signed [10:0] err = e0 > V_TOT / 2 ? e0 - V_TOT : e0 < -(V_TOT / 2) ? e0 + V_TOT : e0;
    reg  signed [3:0]  trim;
    reg                fin_seen;

    // Stored line shown on display row n: a 120-line half (zoom y) or every other line (line pick).
    function [12:0] row_of(input [7:0] vw, input [7:0] n);
        row_of = video_row(vw[7:6], vw[1] ? {n[7:1], vw[0]} : vw[3] ? {1'b0, n[7:1]} + (vw[2] ? 8'd120 : 8'd0) : n);
    endfunction

    wire       eol    = $signed({2'b00, h}) >= H_TOT - 1 + trim;   // >=: the trim may shrink mid-line
    wire [9:0] v_next = v == V_TOT - 1 ? 10'd0 : v + 1'b1;
    wire       fs     = eol && v_next == 10'd0;
    // Line fetched as line v_next begins: two ahead, across the wrap.
    wire [9:0] t_next = v_next >= V_TOT - 2 ? v_next + 2'd2 - V_TOT : v_next + 2'd2;

    always @(posedge clk) begin
        if (fin) begin
            fin_seen <= 1'b1;
            trim <= err > TRIM ? TRIM : err < -TRIM ? -TRIM : err >= -1 && err <= 1 ? 4'sd0 : err[3:0];
        end
        if (fs) begin
            if (!fin_seen && !fin) trim <= 4'sd0;
            fin_seen <= fin;
        end
    end

    // Window, latched per frame.
    reg  [9:0]  wx, ww;
    reg  [8:0]  wy, wh;
    reg  [19:0] whs, wvs;
    reg  [24:0] wpan;
    reg  [19:0] ovw;
    reg  [24:0] ovs;

    // {needed, stored row} for output line t: the display row at (t - y) * vstep, clamped to the field.
    function [13:0] fetch_for(input [7:0] vw, input [9:0] t, input [8:0] y, input [8:0] hh, input [19:0] vs);
        reg [9:0]  rel;
        reg [28:0] n;
        begin
            rel = t - {1'b0, y};
            n   = rel * vs;
            fetch_for = {t >= {1'b0, y} && t < {1'b0, y} + hh && t < V_ACT,
                         row_of(vw, n[28:16] > LINES - 1 ? LINES - 1 : n[23:16])};
        end
    endfunction
    wire [13:0] ff_next  = fetch_for(show, t_next, wy, wh, wvs);

    wire [31:0]        rng;
    wire signed [10:0] shift;
    wire [9:0]         bar;
    xorshift32 prng (.clk(clk), .q(rng));
    fx_timebase #(.V_ACT(V_ACT), .BAR_H(BAR_H)) timebase (
        .clk(clk), .frame_start(fs), .eol(eol), .v_next(v_next),
        .jitter(fx[6]), .track(fx[8]), .rng(rng), .shift(shift), .bar(bar)
    );

    // Source pixel for h, moved by the time-base shift; zoom x stretches one GCV2 half across the line.
    // The halves span DE1 pixels 5.23-359.97 and 359.97-714.46 (measured off the Magewell, 2026-09-26).
    localparam [25:0] X_LEFT  = 26'd342753;        // 5.23 in Q16
    localparam [25:0] X_RIGHT = 26'd23590834;      // 359.97 in Q16
    localparam [15:0] X_STEP  = 16'd32277;         // 354.6 / 720 in Q16
    // With 1, 2, 4 or 8 audio cells the picture starts at GCV2 pixel 14, 20, 32 or 56 (streams.c AUD_W)
    // and its halves split mid-way: left edge and split in DE1 pixels, Q16, and the stretch per pixel.
    reg  [25:0] a_left, a_right;
    reg  [15:0] a_hstep, a_fstep;
    reg  [9:0]  a_mask;                 // DE1 pixels under the cells
    reg  [1:0]  show_nl;
    always @* case (show_nl)
        2'd0: {a_left, a_right, a_fstep, a_hstep, a_mask} =
                  {26'd2074001, 26'd24456012, 16'd62151, 16'd31076, 10'd32};    // 31.65, 373.17
        2'd1: {a_left, a_right, a_fstep, a_hstep, a_mask} =
                  {26'd2815964, 26'd24826734, 16'd61121, 16'd30560, 10'd43};    // 42.97, 378.83
        2'd2: {a_left, a_right, a_fstep, a_hstep, a_mask} =
                  {26'd4299890, 26'd25568178, 16'd59060, 16'd29530, 10'd66};    // 65.61, 390.14
        default: {a_left, a_right, a_fstep, a_hstep, a_mask} =
                  {26'd7267743, 26'd27051067, 16'd54938, 16'd27469, 10'd111};  // 110.90, 412.77
    endcase
    // Window-relative position; the window step scales the full-screen mapping (1:1, or the zoom/fit one).
    reg                show_fit;
    reg  [19:0]        step;
    wire               zoomed = show[5] || show_fit;
    wire signed [11:0] rel    = $signed({2'b00, h}) - $signed({2'b00, wx});
    wire signed [11:0] pos    = rel - shift;
    wire [25:0]        z_base = show_fit ? (show[5] && show[4] ? a_right : a_left) : show[4] ? X_RIGHT : X_LEFT;
    wire [15:0]        z_step = show_fit ? (show[5] ? a_hstep : a_fstep) : X_STEP;
    wire [35:0]        zs_w   = z_step * whs;
    // Pan: the window starts pan into a period of the region less the inset each side, and wraps within it.
    reg  [26:0]        p_rw, p_in, p_per, p_off;
    wire [34:0]        p_in_w  = p_rw * wpan[23:16];
    wire [42:0]        p_off_w = p_per * wpan[15:0];
    always @(posedge clk) begin
        p_rw  <= zoomed ? z_step * 10'd720 : 27'd720 << 16;
        p_in  <= wpan[24] ? p_in_w[34:8] : 27'd0;
        p_per <= p_rw - {p_in[25:0], 1'b0};
        p_off <= wpan[24] ? p_off_w[42:16] : 27'd0;
    end
    wire [31:0]        lin    = pos[9:0] * step + p_off;
    wire [31:0]        off    = wpan[24] && lin >= p_per ? lin - p_per : lin;
    wire [31:0]        zsrc   = (zoomed ? {6'd0, z_base} : 32'd0) + p_in + off;
    wire [9:0]         src    = zsrc[25:16];
    wire               in_win = rel >= 0 && rel < $signed({2'b00, ww}) && v >= {1'b0, wy} && v < {1'b0, wy} + wh;
    wire               src_ok = in_win && pos >= 0 && zsrc[31:16] < H_ACT;
    always @(posedge clk) step <= zoomed ? zs_w[35:16] : whs;

    always @(posedge clk) begin
        fetch     <= 1'b0;
        frame_top <= fs;
        vblank    <= v >= V_ACT;
        if (eol) begin
            h <= 10'd0;
            v <= v_next;
            if (v_next == V_TOT - 3) begin      // the next frame's view, before line 523 fetches its row 0
                show     <= view;
                show_fit <= fit;
                show_nl  <= aud_n_log2;
                {wx, wy, ww, wh, whs, wvs, wpan} <= {win_x, win_y, win_w, win_h, win_hstep, win_vstep, win_pan};
                {ovw, ovs} <= {ov_hwin, ov_hstep};
            end
            fetch      <= ff_next[13];
            fetch_row  <= ff_next[12:0];
            fetch_slot <= t_next[1:0];
        end else h <= h + 1'b1;
        raddr <= {v[1:0], src_ok ? src : 10'd0};    // output line v lives in slot v[1:0]
    end

    // Pipeline, relative to the cycle holding (h, v): raddr +1, rdata +2, w1 +3 (pixel h selected,
    // fx_ycc), colour products +4 (ycc2rgb), RGB +5 (fx_rgb). Sync and blank are delayed 5 to match.
    wire       de = h < H_ACT && v < V_ACT;
    wire       hs = h >= H_ACT + H_FP && h < H_ACT + H_FP + H_SYNC;
    wire       vs = v >= V_ACT + V_FP && v < V_ACT + V_FP + V_SYNC;
    reg  [4:0] de_d, hs_d, vs_d;
    reg  [3:0] oob_d;
    reg  [2:0] odd_d;
    always @(posedge clk) begin
        de_d  <= {de_d[3:0], de};
        hs_d  <= {hs_d[3:0], hs};
        vs_d  <= {vs_d[3:0], vs};
        oob_d <= {oob_d[2:0], !src_ok || (mask_audio && !show_fit && src < a_mask)};
        odd_d <= {odd_d[1:0], src[0]};
    end
    assign blank_n = de_d[4];
    assign hs_n    = ~hs_d[4];
    assign vs_n    = ~vs_d[4];

    // Chroma is co-sited on even pixels: Cb in the even word, Cr in the odd word of each pair; p1 is
    // the pair's other word. w2, w1, rdata are the previous, current and next pixel read.
    reg  [15:0] w1, w2;
    reg  [7:0]  p1;
    always @(posedge clk) {w2, w1, p1} <= {w1, rdata, rdata_p};
    wire [7:0] cb8 = odd_d[2] ? p1 : w1[7:0];
    wire [7:0] cr8 = odd_d[2] ? w1[7:0] : p1;

    wire signed [9:0] ys, cbs, crs;
    fx_ycc #(.BAR_H(BAR_H)) ycc_fx (
        .clk(clk), .de(de_d[2]), .v(v), .bar(bar), .rng(rng),
        .soft(fx[7]), .track(fx[8]), .negative(fx[5]), .grey(fx[1]),
        .y_prev(w2[15:8]), .y_cur(w1[15:8]), .y_next(rdata[15:8]), .cb(cb8), .cr(cr8),
        .ys(ys), .cbs(cbs), .crs(crs)
    );

    wire [7:0] r0, g0, b0, r1, g1, b1;
    ycc2rgb csc (.clk(clk), .ys(ys), .cbs(cbs), .crs(crs), .r(r0), .g(g0), .b(b0));

    // v still holds the pixel's line here: the 5-cycle pipeline only lags into blanking.
    fx_rgb rgb_fx (
        .invert(fx[0]), .posterize(fx[2]), .scanline(fx[3]), .rotate(fx[4]), .odd_line(v[0]),
        .r_in(r0), .g_in(g0), .b_in(b0), .r(r1), .g(g1), .b(b1)
    );

    // HUD values from the game CPU (else hud_demo), drawn at screen (h, v) so it holds still over jitter.
    // Its outputs are 2 cycles behind (h, v); two more registers line them up with the RGB stage.
    wire [3:0]  d_hp;
    wire [1:0]  d_lives;
    wire [19:0] d_score;
    hud_demo demo (
        .clk(clk), .frame_start(fs), .hp(d_hp), .lives(d_lives), .score(d_score)
    );
    wire [3:0]  hp    = hud_cpu ? hud_vals[3:0] : d_hp;
    wire [1:0]  lives = hud_cpu ? hud_vals[5:4] : d_lives;
    wire [19:0] score = hud_cpu ? hud_vals[25:6] : d_score;

    wire        hud_on, hud_dim;
    wire [23:0] hud_rgb;
    hud overlay (
        .clk(clk), .enable(fx[9]), .h(h), .v(v),
        .hp(hp), .lives(lives), .score(score), .tape(tape),
        .on(hud_on), .dim(hud_dim), .rgb(hud_rgb)
    );

    // Game CPU overlay, above the video and effects, below the HUD.
    wire        ov_on;
    wire [23:0] ov_rgb;
    overlay cpu_ov (
        .wclk(ov_wclk), .fb_we(ov_we), .fb_addr(ov_waddr), .fb_data(ov_wdata), .back(ov_back),
        .pal_we(pal_we), .pal_addr(pal_addr), .pal_data(pal_data),
        .clk(clk), .enable(ov_enable), .front(ov_front), .x0(ovw[9:0]), .w(ovw[19:10]), .src0(ovs[24:16]),
        .step(ovs[15:0]), .h(h), .v(v), .on(ov_on), .rgb(ov_rgb)
    );
    reg  [1:0]  hud_on_d, hud_dim_d;
    reg  [23:0] hud_rgb_d1, hud_rgb_d2;
    always @(posedge clk) begin
        hud_on_d  <= {hud_on_d[0], hud_on};
        hud_dim_d <= {hud_dim_d[0], hud_dim};
        {hud_rgb_d2, hud_rgb_d1} <= {hud_rgb_d1, hud_rgb};
    end

    always @(posedge clk)
        if (!de_d[3])          {r, g, b} <= 24'd0;
        else if (hud_on_d[1])  {r, g, b} <= hud_rgb_d2;
        else if (ov_on)        {r, g, b} <= ov_rgb;
        else if (oob_d[3])     {r, g, b} <= 24'd0;
        else if (hud_dim_d[1]) {r, g, b} <= {1'b0, r1[7:1], 1'b0, g1[7:1], 1'b0, b1[7:1]};
        else                   {r, g, b} <= {r1, g1, b1};
endmodule
