// Two-flop synchronizer for a level, or a slow bus where a torn read only flickers a readout.
module sync2 #(
    parameter integer W = 1
) (
    input  wire         clk,
    input  wire [W-1:0] d,
    output wire [W-1:0] q
);
    reg [W-1:0] m, s;
    always @(posedge clk) {s, m} <= {m, d};
    assign q = s;
endmodule
