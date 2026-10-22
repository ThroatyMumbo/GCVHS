// Flags a field that is almost all blue-back (a stopped deck, or the ADV7180's own free-run blue).
// Blue is Y < 96, Cb >= 150, Cr < 150: the GCV2's (Y 35, Cb 212, Cr 114) and the Panasonic's (Y 60, Cb ~165, Cr ~130).
module blue_detect (
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] data,
    output reg        blue          // the last two fields were blue
);
    reg  [7:0] b0, b1, b2, b3;
    wire       is_code = b3 == 8'hFF && b2 == 8'h00 && b1 == 8'h00;
    wire       f = b0[6], v = b0[5], h = b0[4];
    wire       prot_ok = b0[7] && b0[3:0] == {v ^ h, f ^ h, f ^ v, f ^ v ^ h};
    wire       eav = is_code && prot_ok && h;
    wire       sav = is_code && prot_ok && !h;

    reg  [10:0] pos;                // byte after SAV: Cb Y Cr Y ...
    reg         act, prev_v;
    reg  [7:0]  cb, yv;
    reg  [17:0] total, hits;
    reg         last_blue;
    reg  [22:0] idle;

    // Pixels 40-679: clear of the line edges and of the in-line audio cells.
    wire in_x = pos >= 11'd80 && pos < 11'd1360;

    always @(posedge clk) begin
        {b3, b2, b1, b0} <= {b2, b1, b0, data};
        if (rst) begin
            act <= 1'b0; prev_v <= 1'b0; total <= 18'd0; hits <= 18'd0;
            last_blue <= 1'b0; blue <= 1'b0; idle <= 23'd0;
        end else begin
            idle <= idle + (idle != 23'h7FFFFF);
            if (idle == 23'd2_700_000) begin          // 100 ms with no field: nothing to judge
                blue <= 1'b0; last_blue <= 1'b0;
            end
            if (sav) begin
                act <= !v;
                pos <= 11'd1;                          // data is already Cb of pixel 0
            end else if (eav) begin
                act <= 1'b0;
                prev_v <= v;
                if (v && !prev_v) begin
                    last_blue <= total != 18'd0 && hits >= total - (total >> 3);
                    blue      <= last_blue && total != 18'd0 && hits >= total - (total >> 3);
                    total     <= 18'd0;
                    hits      <= 18'd0;
                    idle      <= 23'd0;
                end
            end else if (act) begin
                pos <= pos + 1'b1;
                if (pos[1:0] == 2'd0) cb <= data;
                if (pos[1:0] == 2'd1) yv <= data;
                // Every 8th pixel: Cr of the pair, judged with its Cb and first Y.
                if (in_x && pos[3:0] == 4'd2) begin
                    total <= total + 1'b1;
                    if (yv < 8'd96 && cb >= 8'd150 && data < 8'd150) hits <= hits + 1'b1;
                end
            end
        end
    end
endmodule
