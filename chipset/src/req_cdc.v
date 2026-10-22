// A request pulse and its data across clock domains: the source toggles a flag and holds the data, which the
// destination samples 2-3 dclk after the toggle, so requests must be several destination clocks apart.
module req_cdc #(
    parameter integer W = 14
) (
    input  wire         sclk,
    input  wire         sreq,           // pulse
    input  wire [W-1:0] sdata,
    input  wire         dclk,
    output reg          dreq,           // pulse
    output reg  [W-1:0] ddata
);
    reg         tog = 1'b0;
    reg [W-1:0] held;
    reg [2:0]   s = 3'd0;

    always @(posedge sclk)
        if (sreq) begin
            tog  <= ~tog;
            held <= sdata;
        end

    always @(posedge dclk) begin
        s    <= {s[1:0], tog};
        dreq <= s[2] ^ s[1];
        if (s[2] ^ s[1]) ddata <= held;
    end
endmodule
