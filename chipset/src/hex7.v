// Active-low 7-segment patterns (segment 0 = a ... 6 = g) for the DE1-SoC HEX displays.
module hex7 (
    input  wire [4:0] code,             // 0-F hex digits, 16 blank, 17 'n', 18 'P', 19 '-'
    output reg  [6:0] seg
);
    always @* case (code)
        5'h00: seg = 7'h40;
        5'h01: seg = 7'h79;
        5'h02: seg = 7'h24;
        5'h03: seg = 7'h30;
        5'h04: seg = 7'h19;
        5'h05: seg = 7'h12;
        5'h06: seg = 7'h02;
        5'h07: seg = 7'h78;
        5'h08: seg = 7'h00;
        5'h09: seg = 7'h10;
        5'h0A: seg = 7'h08;
        5'h0B: seg = 7'h03;
        5'h0C: seg = 7'h46;
        5'h0D: seg = 7'h21;
        5'h0E: seg = 7'h06;
        5'h0F: seg = 7'h0E;
        5'h11: seg = 7'h2B;
        5'h12: seg = 7'h0C;
        5'h13: seg = 7'h3F;
        default: seg = 7'h7F;
    endcase
endmodule
