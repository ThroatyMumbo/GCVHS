// Per-line picture shift: a -4..4 pixel random walk plus head-switch skew over the last 16 lines, and a tear in the
// tracking band, which rolls up a line per frame and restarts BAR_GAP lines below the bottom (off-screen BAR_GAP frames).
module fx_timebase #(
    parameter integer V_ACT   = 480,
    parameter integer BAR_H   = 24,     // tracking band height, output lines
    parameter integer BAR_GAP = 60
) (
    input  wire               clk,
    input  wire               frame_start,
    input  wire               eol,      // output line v_next begins next cycle
    input  wire [9:0]         v_next,
    input  wire               jitter,
    input  wire               track,
    input  wire [31:0]        rng,
    output reg  signed [10:0] shift,    // current line's picture shift, pixels right
    output reg  [9:0]         bar       // tracking band's first output line
);
    localparam [9:0] BAR_TOP     = 1024 - BAR_H;
    localparam [9:0] BAR_RESTART = V_ACT + BAR_GAP;

    reg  signed [4:0]  walk;

    wire [9:0]         hsw_d = v_next - 10'd464;
    wire [9:0]         bar_n = v_next - bar;
    wire signed [10:0] jit   = walk + (v_next >= 464 && v_next < V_ACT ? $signed({5'd0, hsw_d[3:0], 2'd0}) : 11'sd0);
    wire signed [10:0] tear  = track && bar_n < BAR_H ? $signed({5'd0, rng[10:5]}) : 11'sd0;

    always @(posedge clk) begin
        if (frame_start) bar <= bar == BAR_TOP ? BAR_RESTART : bar - 1'b1;
        if (eol) begin
            shift <= (jitter ? jit : 11'sd0) + tear;
            if (!v_next[0]) begin
                if (rng[1:0] == 2'd0 && walk > -5'sd4) walk <= walk - 1'b1;
                if (rng[1:0] == 2'd1 && walk <  5'sd4) walk <= walk + 1'b1;
            end
        end
    end
endmodule
