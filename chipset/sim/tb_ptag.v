// ptag_slicer, and tag_slicer that must reject these lines, against tools/ptag.py stim (720 luma samples a line,
// one every other clock as in BT.656). Writes per line: reads, the first three positions, tag ok.
`timescale 1ns / 1ps
module tb_ptag;
    `include "out/ptag_params.vh"

    reg clk = 0;
    always #5 clk = ~clk;

    reg [7:0] ys [0:NL * 720 - 1];
    initial $readmemh("out/ptag_lines.txt", ys);

    reg        start = 0, y_en = 0, done = 0;
    reg  [7:0] y = 0;
    wire       hit, tag_ok;
    wire [15:0] pos;
    wire [6:0] at;
    wire [3:0] scheme;
    wire [7:0] count;
    ptag_slicer dut (.clk(clk), .start(start), .y_en(y_en), .y(y), .hit(hit), .pos(pos), .at(at));
    tag_slicer tag (.clk(clk), .start(start), .y_en(y_en), .y(y), .done(done), .ok(tag_ok), .scheme(scheme),
                    .count(count));

    integer ln = 0, c = 0, n = 0, out;
    reg [15:0] p [0:2];
    initial out = $fopen("out/ptag_out.txt", "w");
    always @(posedge clk) begin
        start <= c == 0;
        y_en  <= c > 4 && c < 1445 && c[0];
        if (c > 4 && c < 1445 && c[0]) y <= ys[ln * 720 + (c - 5) / 2];
        done  <= c == 1450;
        if (c == 0) n <= 0;
        else if (hit) begin
            if (n < 3) p[n] <= pos;
            n <= n + 1;
        end
        if (c == 1453) $fdisplay(out, "%0d %h %h %h %0d", n, p[0], p[1], p[2], tag_ok);
        c <= c == 1460 ? 0 : c + 1;
        if (c == 1460) begin
            ln <= ln + 1;
            if (ln == NL - 1) begin
                $fclose(out);
                $finish;
            end
        end
    end
endmodule
