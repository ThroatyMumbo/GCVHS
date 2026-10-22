// PGA2350 link test: pins[0] is a strobe (each edge is one word), pins[7:1] a PRBS-7 (x^7 + x^6 + 1).
// Oversampled, no pin clocking. One corrupted word counts as two errors (itself, and the next against it).
module link_rx (
    input  wire        clk,
    input  wire        clr,              // level from JTAG, synchronised here
    input  wire [7:0]  pins,
    output reg  [31:0] words,            // snapshots, updated every 2^25 clocks
    output reg  [31:0] errs,
    output reg  [31:0] rate,             // words in the last 2^25 clocks
    output reg  [6:0]  last
);
    reg [7:0] s1, s2, s3;
    reg [1:0] clr_s;
    always @(posedge clk) begin
        {s3, s2, s1} <= {s2, s1, pins};
        clr_s        <= {clr_s[0], clr};
    end

    wire       stb  = s2[0] ^ s3[0];
    wire [6:0] word = s2[7:1];

    reg        have, hit, bad;
    reg [6:0]  prev;
    reg [24:0] tick;
    reg [31:0] n_words, n_errs, n_tick;
    always @(posedge clk) begin
        hit <= stb;
        if (stb) begin
            bad  <= have && word != {prev[5:0], prev[6] ^ prev[5]};
            prev <= word;
            have <= 1'b1;
        end

        tick <= tick + 1'b1;
        if (tick == 25'd0) begin
            words  <= n_words;
            errs   <= n_errs;
            rate   <= n_tick;
            last   <= prev;
            n_tick <= hit;
        end else
            n_tick <= n_tick + hit;
        n_words <= n_words + hit;
        n_errs  <= n_errs + (hit & bad);

        if (clr_s[1]) begin
            have    <= 1'b0;
            n_words <= 32'd0;
            n_errs  <= 32'd0;
        end
    end
endmodule
