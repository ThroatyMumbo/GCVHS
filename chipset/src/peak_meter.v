// Largest |x| over each window of N samples.
module peak_meter #(
    parameter [15:0] N = 16'd48828      // 1 s at the codec's 48.83 kHz
) (
    input  wire              clk,
    input  wire              en,
    input  wire signed [15:0] x,
    output reg  [15:0]       peak = 16'd0
);
    reg  [15:0] n = 16'd0, pk = 16'd0;
    wire [15:0] mag = x[15] ? -x : x;
    always @(posedge clk)
        if (en) begin
            if (n == N - 1'b1) begin
                n    <= 16'd0;
                peak <= pk;
                pk   <= mag;
            end else begin
                n <= n + 1'b1;
                if (mag > pk) pk <= mag;
            end
        end
endmodule
