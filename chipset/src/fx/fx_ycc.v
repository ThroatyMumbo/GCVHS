// YCbCr effects, no latency (only filter state is registered): soft blurs luma and smears chroma right, track adds
// grain and snow streaks in the band, negative inverts luma, grey drops chroma. Outputs offset-removed for ycc2rgb.
module fx_ycc #(
    parameter integer BAR_H = 24
) (
    input  wire              clk,
    input  wire              de,        // pixel is in the active picture
    input  wire [9:0]        v,         // output line
    input  wire [9:0]        bar,       // tracking band's first output line
    input  wire [31:0]       rng,
    input  wire              soft,
    input  wire              track,
    input  wire              negative,
    input  wire              grey,
    input  wire [7:0]        y_prev,
    input  wire [7:0]        y_cur,
    input  wire [7:0]        y_next,
    input  wire [7:0]        cb,
    input  wire [7:0]        cr,
    output wire signed [9:0] ys,
    output wire signed [9:0] cbs,
    output wire signed [9:0] crs
);
    wire [9:0] ysum = y_prev + {y_cur, 1'b0} + y_next;
    wire [7:0] y8   = soft ? ysum[9:2] : y_cur;

    wire signed [9:0]  ys0  = $signed({2'b00, y8}) - 10'sd16;
    wire signed [9:0]  cbs0 = $signed({2'b00, cb}) - 10'sd128;
    wire signed [9:0]  crs0 = $signed({2'b00, cr}) - 10'sd128;

    // Chroma smear: one-pole low-pass (x16 fixed point, time constant 8 pixels) that relaxes to
    // grey in blanking, so colour trails to the right of every edge.
    reg  signed [13:0] cbf, crf;
    wire signed [13:0] cb16 = de ? cbs0 : 10'sd0;
    wire signed [13:0] cr16 = de ? crs0 : 10'sd0;
    always @(posedge clk) begin
        cbf <= cbf + (((cb16 <<< 4) - cbf) >>> 3);
        crf <= crf + (((cr16 <<< 4) - crf) >>> 3);
    end
    wire signed [9:0]  cbs1 = soft ? $signed(cbf[13:4]) : cbs0;
    wire signed [9:0]  crs1 = soft ? $signed(crf[13:4]) : crs0;

    // Tracking: grain on every pixel, stronger inside the band, where random white streaks start
    // (densest in the middle of the band) and run for 2-33 pixels, washing out the colour.
    wire [9:0]         bar_d  = v - bar;
    wire               in_bar = track && bar_d < BAR_H;
    wire [6:0]         dens   = bar_d >= BAR_H / 4 && bar_d < BAR_H * 3 / 4 ? 7'd4 : 7'd1;
    reg  [5:0]         streak;
    always @(posedge clk)
        if (streak != 0) streak <= streak - 1'b1;
        else if (in_bar && rng[31:25] < dens) streak <= {1'b0, rng[20:16]} + 6'd2;
    wire               snow   = track && streak != 0;
    wire signed [9:0]  grain  = !track ? 10'sd0 :
                                in_bar ? $signed({4'd0, rng[13:8]}) - 10'sd32 : $signed({6'd0, rng[11:8]}) - 10'sd8;
    wire signed [9:0]  ys1    = snow ? 10'sd190 + $signed({5'd0, rng[4:0]}) : ys0 + grain;

    assign ys  = negative ? 10'sd219 - ys1 : ys1;
    assign cbs = grey || snow ? 10'sd0 : cbs1;
    assign crs = grey || snow ? 10'sd0 : crs1;
endmodule
