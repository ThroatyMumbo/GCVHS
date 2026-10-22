// BCD {hh, mm, ss, ff} at 30 fps to a frame count, one register per product: frame follows tc 4 cycles later.
module tc_frames (
    input  wire        clk,
    input  wire [31:0] tc,
    output reg  [21:0] frame
);
    reg [4:0]  h, f;
    reg [5:0]  m, s;
    reg [10:0] hm;
    reg [16:0] hms;
    always @(posedge clk) begin
        h     <= tc[27:24] + tc[29:28] * 4'd10;
        m     <= tc[19:16] + tc[22:20] * 4'd10;
        s     <= tc[11:8] + tc[14:12] * 4'd10;
        f     <= tc[3:0] + tc[5:4] * 4'd10;
        hm    <= h * 6'd60 + m;
        hms   <= hm * 6'd60 + s;
        frame <= hms * 5'd30 + f;
    end
endmodule
