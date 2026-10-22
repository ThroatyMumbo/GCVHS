// xorshift32 pseudo-random generator, one step per clock. A nonzero seed never reaches zero.
module xorshift32 #(
    parameter [31:0] SEED = 32'h2545_F491
) (
    input  wire        clk,
    output wire [31:0] q
);
    reg  [31:0] s = SEED;
    wire [31:0] s1 = s ^ (s << 13);
    wire [31:0] s2 = s1 ^ (s1 >> 17);
    always @(posedge clk) s <= s2 ^ (s2 << 5);
    assign q = s;
endmodule
