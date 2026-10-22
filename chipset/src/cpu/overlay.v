// Game CPU overlay: two 320x240 8-bit buffers, palette {transparent, r, g, b}; rows doubled, columns stepped from
// src0 by a Q16 step over output x0..x0 + w. Outputs are 4 cycles behind (h, v), in step with the HUD.
module overlay (
    input  wire        wclk,            // sd_clk
    input  wire        fb_we,
    input  wire [16:0] fb_addr,         // pixel in the back buffer, y * 320 + x
    input  wire [7:0]  fb_data,
    input  wire        back,            // buffer fb_we writes
    input  wire        pal_we,          // consecutive pulses are one update, shown whole from a vertical blank
    input  wire [7:0]  pal_addr,
    input  wire [24:0] pal_data,
    input  wire        clk,             // vclk (vga_out's clock)
    input  wire        enable,
    input  wire        front,           // buffer shown
    input  wire [9:0]  x0,              // first output column
    input  wire [9:0]  w,               // output columns
    input  wire [8:0]  src0,            // framebuffer x at x0
    input  wire [15:0] step,            // Q16 framebuffer pixels per output column
    input  wire [9:0]  h,
    input  wire [9:0]  v,
    output reg         on,
    output reg  [23:0] rgb
);
    `include "link_proto.vh"
    localparam integer PIX = LINK_FB_W * LINK_FB_H;

    reg [7:0] mem [0:2 * PIX - 1];
    always @(posedge wclk)
        if (fb_we && fb_addr < PIX) mem[back ? fb_addr + PIX : fb_addr] <= fb_data;

    // Palette writes land in stage. A pal_we burst (one PAL_WR, cpu_cmd.v) ends after 16 quiet clocks and
    // toggles done_w; bursts are thus 170 ns apart or more, so vclk sees every toggle.
    function [24:0] rgb332(input [7:0] c);
        rgb332 = {c == 8'd0, c[7:5], c[7:5], c[7:6], c[4:2], c[4:2], c[4:3], c[1:0], c[1:0], c[1:0], c[1:0]};
    endfunction
    reg [24:0] stage [0:255];
    reg [24:0] pal [0:511];             // two banks: one shown, the other filled from stage
    integer k;
    initial for (k = 0; k < 256; k = k + 1) begin
        stage[k]     = rgb332(k);
        pal[k]       = rgb332(k);
        pal[k + 256] = rgb332(k);
    end
    reg [4:0] hold_w = 5'd0;
    reg       busy_w = 1'b0, done_w = 1'b0;
    always @(posedge wclk) begin
        if (pal_we) stage[pal_addr] <= pal_data;
        hold_w <= pal_we ? 5'd16 : hold_w - (hold_w != 5'd0);
        busy_w <= pal_we || hold_w > 5'd1;
        if (!pal_we && hold_w == 5'd1) done_w <= ~done_w;
    end

    // vclk: once bursts have finished, copy all of stage into the hidden bank, then show it from vertical blanking.
    // A copy that a burst overlapped is redone.
    reg [1:0] busy_s = 2'd0;
    reg [2:0] done_s = 3'd0;
    reg       bank = 1'b0, copying = 1'b0, ready = 1'b0, pend = 1'b0, dirty, cv;
    reg [8:0] ck;
    reg [7:0] cw;
    reg [24:0] cq;
    wire      done_e = done_s[2] ^ done_s[1];
    wire      swap   = !copying && ready && v >= LINK_OUT_H;
    wire      start  = !copying && !swap && pend && !busy_s[1];
    always @(posedge clk) begin
        busy_s <= {busy_s[0], busy_w};
        done_s <= {done_s[1:0], done_w};
        cq     <= stage[ck[7:0]];
        cw     <= ck[7:0];
        cv     <= copying && !ck[8];
        if (cv) pal[{~bank, cw}] <= cq;
        pend <= done_e || (pend && !start);
        if (swap) begin
            bank  <= ~bank;
            ready <= 1'b0;
        end
        if (start) begin
            copying <= 1'b1;
            ready   <= 1'b0;
            dirty   <= 1'b0;
            ck      <= 9'd0;
        end
        if (copying) begin
            ck <= ck + 1'b1;
            if (busy_s[1] || done_e) dirty <= 1'b1;
            if (ck == 9'd260) begin     // 4 clocks past the last read, so a burst under it shows in busy_s
                copying <= 1'b0;
                ready   <= !dirty && !busy_s[1] && !done_e && !pend;
            end
        end
    end

    // sx is the framebuffer column of output column h: loaded the column before x0, or all blanking if x0 is 0.
    reg  [26:0] sx;
    wire        load = x0 == 10'd0 ? h >= LINK_OUT_W : h == x0 - 10'd1;
    always @(posedge clk) sx <= load ? {2'd0, src0, 16'd0} : sx + step;

    wire [9:0]  x    = h - x0;
    wire        in_w = h >= x0 && x < w && v < LINK_OUT_H && sx[26:16] < LINK_FB_W;
    reg  [17:0] addr;
    reg  [2:0]  in_d;
    reg  [7:0]  pix;
    reg  [24:0] pe;
    always @(posedge clk) begin
        addr <= {9'd0, v[9:1]} * LINK_FB_W + sx[24:16] + (front ? PIX : 0);
        in_d <= {in_d[1:0], in_w && enable};
        pix  <= mem[addr];
        pe   <= pal[{bank, pix}];
        on   <= in_d[2] && !pe[24];
        rgb  <= pe[23:0];
    end
endmodule
