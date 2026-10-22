// A multi-bit value across clock domains with a handshake: held only changes once the destination has
// acknowledged it, so no capture is torn, and a request while one is in flight sends the latest value after.
module snap_cdc #(
    parameter integer W = 8
) (
    input  wire         sclk,
    input  wire         srst,           // holds off sending; the value is sent once it drops
    input  wire         sreq,           // the value changed; held high, it is resent as fast as acknowledged
    input  wire [W-1:0] sdata,
    input  wire         dclk,
    output reg          dreq,           // pulse: ddata was updated
    output reg  [W-1:0] ddata
);
    reg         tog = 1'b0, ack = 1'b0, dirty = 1'b1, req_d = 1'b0;
    reg [1:0]   ack_s = 2'd0;
    reg [W-1:0] held;
    reg [2:0]   s = 3'd0;

    // A request is acted on a cycle later, so sdata already carries what it was for.
    always @(posedge sclk) begin
        req_d <= sreq;
        ack_s <= {ack_s[0], ack};
        if (srst) dirty <= 1'b1;
        else if ((dirty || req_d) && tog == ack_s[1]) begin
            held  <= sdata;
            tog   <= ~tog;
            dirty <= 1'b0;
        end else if (req_d) dirty <= 1'b1;
    end

    always @(posedge dclk) begin
        s    <= {s[1:0], tog};
        dreq <= s[2] ^ s[1];
        if (s[2] ^ s[1]) begin
            ddata <= held;
            ack   <= s[1];
        end
    end
endmodule
