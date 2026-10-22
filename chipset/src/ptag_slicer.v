// Position tags (vcr_sim/src/streams.c ptag_make) anywhere on a luma line, as pause and cue keep only pieces.
// Cells of 7 DAC samples (6.6 Y); a sliding 32-cell window matches fixed cells (1 0 at 0, 10, 20; 0 0 at 30) and CRC-8.
module ptag_slicer #(
    parameter [7:0]  THR  = 8'd100,     // between blanking and 80 IRE
    parameter [17:0] CELL = 18'd845     // Y samples per cell, in 1/128
) (
    input  wire        clk,
    input  wire        start,           // SAV
    input  wire        y_en,
    input  wire [7:0]  y,
    output reg         hit,             // pulse per window that matches
    output reg  [15:0] pos,             // with hit
    output reg  [6:0]  at               // with hit: cells into the line where the window ends
);
    function [7:0] crc8(input [15:0] d);   // CRC-8/SMBUS over d[7:0] then d[15:8], each MSB first
        integer i;
        reg [7:0] c;
        reg       b;
        begin
            c = 8'h00;
            for (i = 0; i < 16; i = i + 1) begin
                b = (i < 8 ? d[7 - i] : d[23 - i]) ^ c[7];
                c = {c[6:0], 1'b0} ^ (b ? 8'h07 : 8'h00);
            end
            crc8 = c;
        end
    endfunction

    reg         hi_d, chk;
    reg  [10:0] cur;                    // index of the last sample
    reg  [17:0] nxt;                    // next cell centre, 1/128 sample
    reg  [31:0] s;                      // cells, newest in s[0]: window cell i is s[31 - i]
    reg  [6:0]  n;
    wire        hi   = y > THR;
    wire [17:0] half = {1'b0, CELL[17:1]};
    wire [17:0] now  = {cur + 1'b1, 7'd0};
    wire [17:0] due  = hi != hi_d ? now - 18'd64 + half : nxt;
    wire [7:0]  lo8  = {s[22], s[23], s[24], s[25], s[26], s[27], s[28], s[29]};
    wire [7:0]  hi8  = {s[12], s[13], s[14], s[15], s[16], s[17], s[18], s[19]};
    wire [7:0]  c8   = {s[2], s[3], s[4], s[5], s[6], s[7], s[8], s[9]};
    wire        frame_ok = s[31] && !s[30] && s[21] && !s[20] && s[11] && !s[10] && !s[1] && !s[0];

    always @(posedge clk) begin
        hit <= 1'b0;
        chk <= 1'b0;
        if (start) begin
            cur  <= 11'h7FF;
            nxt  <= half;
            hi_d <= 1'b0;
            s    <= 32'd0;
            n    <= 7'd0;
        end else if (y_en) begin
            cur  <= cur + 1'b1;
            hi_d <= hi;
            nxt  <= due;
            if (now + 18'd64 > due) begin
                nxt <= due + CELL;
                s   <= {s[30:0], hi};
                n   <= n + (n != 7'h7F);
                chk <= 1'b1;
            end
        end
        if (chk && frame_ok && crc8({hi8, lo8}) == c8) begin
            hit <= 1'b1;
            pos <= {hi8, lo8};
            at  <= n;
        end
    end
endmodule
