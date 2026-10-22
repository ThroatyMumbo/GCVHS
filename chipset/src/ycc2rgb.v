// BT.601 studio-range YCbCr (offset removed: Y - 16, Cb - 128, Cr - 128) to full-range RGB.
// One clock of latency: products registered, sums and clamps combinational.
module ycc2rgb (
    input  wire              clk,
    input  wire signed [9:0] ys,
    input  wire signed [9:0] cbs,
    input  wire signed [9:0] crs,
    output wire [7:0]        r,
    output wire [7:0]        g,
    output wire [7:0]        b
);
    reg  signed [21:0] ky, rcr, gcr, gcb, bcb;  // x256 fixed point
    always @(posedge clk) begin
        ky  <= ys  * 22'sd298;
        rcr <= crs * 22'sd409;
        gcr <= crs * 22'sd208;
        gcb <= cbs * 22'sd100;
        bcb <= cbs * 22'sd516;
    end

    function [7:0] clamp(input signed [21:0] x);
        clamp = x < 0 ? 8'd0 : x > 22'sd65535 ? 8'd255 : x[15:8];
    endfunction

    assign r = clamp(ky + rcr + 22'sd128);
    assign g = clamp(ky - gcr - gcb + 22'sd128);
    assign b = clamp(ky + bcb + 22'sd128);
endmodule
