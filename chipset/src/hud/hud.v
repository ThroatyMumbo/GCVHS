// HUD text in 45x30 cells of 16x16 (8x8 glyphs doubled): HP, score and lives at the top, PLAY and the tape counter below.
// Two cycles of latency. on: draw rgb over the video; dim: darken it, for the strips behind the text.
module hud (
    input  wire        clk,
    input  wire        enable,
    input  wire [9:0]  h,
    input  wire [9:0]  v,
    input  wire [3:0]  hp,              // 0-10
    input  wire [1:0]  lives,           // 0-3
    input  wire [19:0] score,           // 5 BCD digits, shown with a trailing 0
    input  wire [19:0] tape,            // BCD {H, M tens, M, S tens, S}
    output reg         on,
    output reg         dim,
    output reg  [23:0] rgb
);
    // Character codes: 0-9 are the digits, so BCD indexes them directly.
    localparam [4:0] SP = 5'd10, A = 5'd11, C = 5'd12, E = 5'd13, H = 5'd14, I = 5'd15, L = 5'd16,
                     O = 5'd17, P = 5'd18, R = 5'd19, S = 5'd20, V = 5'd21, Y = 5'd22,
                     COLON = 5'd23, HEART = 5'd24, BAR = 5'd25, BAR_E = 5'd26, PLAY = 5'd27;

    localparam [23:0] WHITE = 24'hF0F0F0, YELLOW = 24'hF8D820, RED = 24'hF03030,
                      GREEN = 24'h40E040, GREY = 24'h707070;

    // 8x8 glyphs, top row in the high byte, leftmost pixel in the high bit.
    function [63:0] glyph(input [4:0] ch);
        case (ch)
            5'd0:    glyph = 64'h7CC6CEDEF6E67C00;
            5'd1:    glyph = 64'h307030303030FC00;
            5'd2:    glyph = 64'h78CC0C3860CCFC00;
            5'd3:    glyph = 64'h78CC0C380CCC7800;
            5'd4:    glyph = 64'h1C3C6CCCFE0C1E00;
            5'd5:    glyph = 64'hFCC0F80C0CCC7800;
            5'd6:    glyph = 64'h3860C0F8CCCC7800;
            5'd7:    glyph = 64'hFCCC0C1830303000;
            5'd8:    glyph = 64'h78CCCC78CCCC7800;
            5'd9:    glyph = 64'h78CCCC7C0C187000;
            A:       glyph = 64'h3078CCCCFCCCCC00;
            C:       glyph = 64'h3C66C0C0C0663C00;
            E:       glyph = 64'hFE6268786862FE00;
            H:       glyph = 64'hCCCCCCFCCCCCCC00;
            I:       glyph = 64'h7830303030307800;
            L:       glyph = 64'hF06060606266FE00;
            O:       glyph = 64'h386CC6C6C66C3800;
            P:       glyph = 64'hFC66667C6060F000;
            R:       glyph = 64'hFC66667C6C66E600;
            S:       glyph = 64'h78CCE0701CCC7800;
            V:       glyph = 64'hCCCCCCCCCC783000;
            Y:       glyph = 64'hCCCCCC7830307800;
            COLON:   glyph = 64'h0030300000303000;
            HEART:   glyph = 64'h6CFEFEFE7C381000;
            BAR:     glyph = 64'h00FEFEFEFEFEFE00;
            BAR_E:   glyph = 64'h00FE82828282FE00;
            PLAY:    glyph = 64'hC0F0FCFFFCF0C000;
            default: glyph = 64'h0;
        endcase
    endfunction

    wire [4:0] row = v[8:4];
    wire [5:0] col = h[9:4];

    // Cell contents for (row, col).
    reg  [4:0]  ch;
    reg  [23:0] colour;
    always @* begin
        ch     = SP;
        colour = WHITE;
        case (row)
            5'd1: case (col)
                6'd3:  ch = H;
                6'd4:  ch = P;
                6'd29: ch = S;
                6'd30: ch = C;
                6'd31: ch = O;
                6'd32: ch = R;
                6'd33: ch = E;
                6'd40: begin ch = 5'd0; colour = YELLOW; end
                default:
                    if (col >= 6 && col < 16) begin
                        ch     = col - 6 < hp ? BAR : BAR_E;
                        colour = col - 6 >= hp ? GREY : hp <= 3 ? RED : GREEN;
                    end else if (col >= 35 && col < 40) begin
                        ch     = {1'b0, score[(39 - col) * 4 +: 4]};
                        colour = YELLOW;
                    end
            endcase
            5'd2: case (col)
                6'd3:  ch = L;
                6'd4:  ch = I;
                6'd5:  ch = V;
                6'd6:  ch = E;
                6'd7:  ch = S;
                default:
                    if (col >= 9 && col < 12 && col - 9 < lives) begin
                        ch     = HEART;
                        colour = RED;
                    end
            endcase
            5'd27: case (col)
                6'd3:  begin ch = PLAY; colour = GREEN; end
                6'd5:  ch = P;
                6'd6:  ch = L;
                6'd7:  ch = A;
                6'd8:  ch = Y;
                6'd34: ch = {1'b0, tape[19:16]};
                6'd35: ch = COLON;
                6'd36: ch = {1'b0, tape[15:12]};
                6'd37: ch = {1'b0, tape[11:8]};
                6'd38: ch = COLON;
                6'd39: ch = {1'b0, tape[7:4]};
                6'd40: ch = {1'b0, tape[3:0]};
                default: ;
            endcase
            default: ;
        endcase
    end

    // Darkened strips behind the text, with a little padding.
    wire strip = h >= 40 && h < 680 && ((v >= 12 && v < 52) || (v >= 428 && v < 452));

    // Stage 1: cell lookup. Stage 2: glyph pixel.
    reg  [4:0]  ch1;
    reg  [23:0] colour1;
    reg  [2:0]  gx1, gy1;
    reg         strip1;
    always @(posedge clk) begin
        ch1     <= ch;
        colour1 <= colour;
        gx1     <= h[3:1];
        gy1     <= v[3:1];
        strip1  <= strip;
    end

    wire [63:0] g    = glyph(ch1);
    wire [7:0]  bits = g[(7 - gy1) * 8 +: 8];
    always @(posedge clk) begin
        on  <= enable && bits[~gx1];
        dim <= enable && strip1;
        rgb <= colour1;
    end
endmodule
