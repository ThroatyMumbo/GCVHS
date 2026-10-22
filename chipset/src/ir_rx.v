// Demodulated IR receiver: records each frame's mark and space durations (us, mark first) for
// JTAG readout. A space of GAP_US ends a frame; the idle level is whichever holds for IDLE_US.
module ir_rx #(
    parameter integer CLK_HZ  = 50_000_000,
    parameter integer GAP_US  = 8000,       // longer than any in-frame space, shorter than any frame gap
    parameter integer IDLE_US = 50000       // longer than any mark (NEC's 9 ms header)
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         rx,
    input  wire [3:0]   page,           // entries page*16 .. page*16+15 of the last frame
    output wire [255:0] page_data,      // entry page*16 in the low bits
    output reg  [15:0]  seq,            // frames completed
    output reg  [31:0]  t_start,        // us since start-up at the last frame's first mark
    output reg  [8:0]   len,            // entries in the last frame
    output reg          over,           // the last frame had more than 256
    output reg          idle_level
);
    localparam integer DIV = CLK_HZ / 1_000_000;
    reg [$clog2(DIV)-1:0] div = 0;
    wire us = div == DIV - 1;
    always @(posedge clk) div <= us ? 1'b0 : div + 1'b1;
    reg [31:0] now = 0, t_first;
    always @(posedge clk) now <= now + us;

    localparam integer GLITCH = CLK_HZ / 250_000;  // 4 us a level must hold; marks and spaces are 400 us up
    reg  [2:0]  rx_s = 3'b111;
    reg  [$clog2(GLITCH)-1:0] gl = 0;
    reg         lvl = 1'b1, lvl_d = 1'b1;
    always @(posedge clk)
        if (rx_s[2] == lvl) gl <= 1'b0;
        else if (gl == GLITCH - 1) begin
            lvl <= rx_s[2];
            gl  <= 1'b0;
        end else gl <= gl + 1'b1;

    reg  [15:0] dur;
    reg         in_frame, over_w, wbank, rbank, we;
    reg  [8:0]  n;
    reg  [8:0]  waddr;
    reg  [15:0] wdata;

    always @(posedge clk) begin
        rx_s  <= {rx_s[1:0], rx};
        lvl_d <= lvl;
        we    <= 1'b0;
        if (rst) begin
            in_frame   <= 1'b0;
            dur        <= 16'd0;
            wbank      <= 1'b0;
            rbank      <= 1'b1;
            seq        <= 16'd0;
            len        <= 9'd0;
            over       <= 1'b0;
            idle_level <= 1'b1;
        end else if (lvl != lvl_d) begin
            if (in_frame) begin
                if (n[8]) over_w <= 1'b1;
                else begin
                    we    <= 1'b1;
                    waddr <= {wbank, n[7:0]};
                    wdata <= dur;
                    n     <= n + 1'b1;
                end
            end else if (lvl != idle_level) begin
                in_frame <= 1'b1;
                t_first  <= now;
                n        <= 9'd0;
                over_w   <= 1'b0;
            end
            dur <= 16'd0;
        end else if (us && dur != 16'hFFFF) begin
            dur <= dur + 1'b1;
            if (dur == GAP_US - 1 && in_frame && lvl == idle_level) begin
                in_frame <= 1'b0;
                rbank    <= wbank;
                wbank    <= ~wbank;
                len      <= n;
                t_start  <= t_first;
                over     <= over_w;
                seq      <= seq + 1'b1;
            end
            if (dur == IDLE_US - 1) begin
                idle_level <= lvl;
                in_frame   <= 1'b0;
            end
        end
    end

    // 16 lanes so a whole page reads at once.
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : lane
            reg [15:0] m [0:31];
            reg [15:0] q;
            always @(posedge clk) begin
                if (we && waddr[3:0] == i) m[{waddr[8], waddr[7:4]}] <= wdata;
                q <= m[{rbank, page}];
            end
            assign page_data[i*16 +: 16] = q;
        end
    endgenerate
endmodule
