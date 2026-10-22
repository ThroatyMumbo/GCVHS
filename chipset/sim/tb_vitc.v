// vitc_slicer (and tag_slicer, which must reject VITC lines) against tools/vitc.py stim: 720 luma
// samples per line, one every other clock as in BT.656. One line out per line: ok, data, tag ok.
`timescale 1ns / 1ps
module tb_vitc;
    `include "out/vitc_params.vh"

    reg clk = 0;
    always #5 clk = ~clk;

    reg [7:0] ys [0:NL * 720 - 1];
    initial $readmemh("out/vitc_lines.txt", ys);

    reg        start = 0, y_en = 0, done = 0;
    reg  [7:0] y = 0;
    wire       ok, tag_ok;
    wire [63:0] data;
    wire [3:0] scheme;
    wire [7:0] count;
    vitc_slicer dut (.clk(clk), .start(start), .y_en(y_en), .y(y), .done(done), .ok(ok), .data(data));
    tag_slicer tag (.clk(clk), .start(start), .y_en(y_en), .y(y), .done(done), .ok(tag_ok), .scheme(scheme),
                    .count(count));

    integer ln = 0, c = 0, out;
    initial out = $fopen("out/vitc_out.txt", "w");
    always @(posedge clk) begin
        start <= c == 0;
        y_en  <= c > 4 && c < 1445 && c[0];
        if (c > 4 && c < 1445 && c[0]) y <= ys[ln * 720 + (c - 5) / 2];
        done  <= c == 1450;
        if (c == 1453) $fdisplay(out, "%0d %h %0d", ok, data, tag_ok);
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
