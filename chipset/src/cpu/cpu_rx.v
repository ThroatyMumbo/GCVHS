// DN front end: pins[0] is the strobe, each edge one byte on pins[8:1]; oversampled, no pin clocking.
// A strobe level counts once held two samples: data-line switching couples 8 ns glitches onto it.
module cpu_rx (
    input  wire       clk,              // 250 MHz
    input  wire [8:0] pins,
    output reg        valid,
    output reg  [7:0] data
);
    reg [8:0] s1, s2, s3;
    reg       lvl = 1'b0;
    always @(posedge clk) begin
        {s3, s2, s1} <= {s2, s1, pins};
        valid <= 1'b0;
        if (s3[0] == s2[0] && s2[0] != lvl) begin
            lvl   <= s2[0];
            valid <= 1'b1;
            data  <= s2[8:1];
        end
    end
endmodule
