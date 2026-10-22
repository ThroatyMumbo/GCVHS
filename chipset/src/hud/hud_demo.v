// HUD values while the game CPU doesn't supply them: score up each frame, HP down every 20, a life per refill (3, 2, 1).
module hud_demo (
    input  wire        clk,
    input  wire        frame_start,
    output reg  [3:0]  hp = 4'd10,      // 0-10
    output reg  [1:0]  lives = 2'd3,    // 0-3
    output reg  [19:0] score            // 5 BCD digits, shown with a fixed trailing 0
);
    function [19:0] bcd_inc(input [19:0] x);
        integer i;
        reg     carry;
        begin
            carry = 1'b1;
            for (i = 0; i < 5; i = i + 1)
                if (carry) begin
                    if (x[4*i +: 4] == 4'd9) x[4*i +: 4] = 4'd0;
                    else begin
                        x[4*i +: 4] = x[4*i +: 4] + 1'b1;
                        carry = 1'b0;
                    end
                end
            bcd_inc = x;
        end
    endfunction

    reg [4:0] hp_frames;                // 0-19

    always @(posedge clk)
        if (frame_start) begin
            score <= bcd_inc(score);

            hp_frames <= hp_frames == 5'd19 ? 5'd0 : hp_frames + 1'b1;
            if (hp_frames == 5'd19) begin
                if (hp == 4'd1) begin
                    hp    <= 4'd10;
                    lives <= lives == 2'd1 ? 2'd3 : lives - 1'b1;
                end else hp <= hp - 1'b1;
            end
        end
endmodule
