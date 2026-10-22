// ltc_rx against tools/ltc.py stim: codec frames every 16 ticks and DE1 field starts, on a clock
// scaled by 64 (the field length comes scaled in ltc_params.vh). One line per field out.
`timescale 1ns / 1ps
module tb_ltc;
    `include "out/ltc_params.vh"

    reg clk = 0;
    always #5 clk = ~clk;

    reg [15:0] xs [0:NS - 1];
    reg [31:0] fs [0:NF - 1];
    initial begin
        $readmemh("out/ltc_x.hex", xs);
        $readmemh("out/ltc_fs.hex", fs);
    end

    reg               x_valid = 0, field_start = 0;
    reg signed [15:0] x = 0;
    wire [23:0] field;
    wire [31:0] tc, user;
    wire        locked;
    wire [15:0] words, bad, jumps, peak;
    wire [19:0] phase;
    ltc_rx #(.FIELD(FIELD_T)) dut (
        .clk(clk), .rst(1'b0), .x_valid(x_valid), .x(x), .field_start(field_start),
        .field(field), .tc(tc), .locked(locked), .user(user),
        .words(words), .bad(bad), .jumps(jumps), .phase(phase), .peak(peak)
    );

    integer tick = 0, si = 0, fi = 0, out;
    reg [2:0] fs_d = 0;
    initial out = $fopen("out/ltc_fields.txt", "w");
    always @(posedge clk) begin
        x_valid     <= tick % 16 == 0 && si < NS;
        if (tick % 16 == 0 && si < NS) begin
            x  <= xs[si];
            si <= si + 1;
        end
        field_start <= fi < NF && tick == fs[fi];
        if (fi < NF && tick == fs[fi]) fi <= fi + 1;
        fs_d <= {fs_d[1:0], field_start};
        if (fs_d[2])
            $fdisplay(out, "%0d %0d %0d %h %0d %0d %0d %0d", tick, field, locked, tc, words, bad, jumps, phase);
        tick <= tick + 1;
        if (si == NS && fi == NF) begin
            $fclose(out);
            $finish;
        end
    end
endmodule
