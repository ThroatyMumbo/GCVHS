// LEDs and HEX: HEX5 stream, HEX4 standard, HEX3 fields/cycle, HEX2 regions, HEX1-0 fields/s.
// Static readouts straight from the 27 MHz counters: a torn value only flickers a digit.
module front_panel (
    input  wire        clk27,           // TD_CLK27
    input  wire        bist_ok,
    input  wire        tag_seen,
    input  wire [15:0] bad_codes,
    input  wire        parity_ok,
    input  wire        codes_seen,
    input  wire [7:0]  status1,
    input  wire        i2c_err,
    input  wire        cfg_done,
    input  wire [7:0]  fields_per_sec,
    input  wire [3:0]  tag_scheme,
    input  wire [1:0]  r_log2,
    input  wire        raw,
    input  wire [3:0]  stream,
    output wire [9:0]  ledr,
    output wire [6:0]  hex0,
    output wire [6:0]  hex1,
    output wire [6:0]  hex2,
    output wire [6:0]  hex3,
    output wire [6:0]  hex4,
    output wire [6:0]  hex5
);
    reg [24:0] beat;
    always @(posedge clk27) beat <= beat + 1'b1;

    assign ledr = {beat[24], bist_ok, tag_seen, bad_codes != 16'd0, parity_ok, codes_seen,
                   status1[2], status1[0], i2c_err, cfg_done & ~i2c_err};

    wire [7:0]  fps    = fields_per_sec > 8'd99 ? 8'd99 : fields_per_sec;
    wire [7:0]  tens   = fps / 8'd10;
    wire [7:0]  ones   = fps % 8'd10;
    wire [7:0]  digits = {tens[3:0], ones[3:0]};
    wire        blank1 = tens == 8'd0;

    // AD_RESULT: 0-1 NTSC, 2-4 and 6 PAL variants, 5 and 7 SECAM ('5' reads as S).
    reg [4:0] std_code;
    always @* case (status1[6:4])
        3'd0, 3'd1:             std_code = 5'h11;
        3'd2, 3'd3, 3'd4, 3'd6: std_code = 5'h12;
        default:                std_code = 5'h05;
    endcase

    hex7 h0 (.code({1'b0, digits[3:0]}), .seg(hex0));
    hex7 h1 (.code(blank1 ? 5'h10 : {1'b0, digits[7:4]}), .seg(hex1));
    hex7 h2 (.code(!tag_seen ? 5'h13 : tag_scheme[1:0] == 2'd3 ? 5'h11 : {2'b0, 3'd1 << r_log2}), .seg(hex2));
    hex7 h3 (.code(tag_seen ? {2'b0, 3'd1 << tag_scheme[3:2]} : 5'h13), .seg(hex3));
    hex7 h4 (.code(status1[0] ? std_code : 5'h13), .seg(hex4));
    hex7 h5 (.code(raw ? 5'h13 : {1'b0, stream}), .seg(hex5));
endmodule
