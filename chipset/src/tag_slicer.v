// GCV2 stream tag (vcr_sim/src/streams.c): 19 cells of 33.9 Y samples, run-in 1 0, {scheme, count, check}
// MSB first, stop 1; timed from the first white sample, so no absolute horizontal position is needed.
module tag_slicer #(
    parameter [7:0]  THR   = 8'd126,    // midway between studio black and white
    parameter [10:0] FIRST = 11'd17,    // samples from the leading edge to the middle of cell 0
    parameter [10:0] CELL  = 11'd34,
    parameter [5:0]  RUN_MIN = 6'd20    // first white run: a whole cell, where VITC's first bit is 7.5 samples
) (
    input  wire       clk,
    input  wire       start,            // SAV of the tag line
    input  wire       y_en,
    input  wire [7:0] y,
    input  wire       done,             // EAV of the tag line
    output reg        ok,               // valid with done's next cycle
    output reg  [3:0] scheme,           // [3] set for 240p
    output reg  [7:0] count
);
    reg        seen, in_run;
    reg [5:0]  run;
    reg [10:0] n, next;
    reg [4:0]  cells;
    reg [18:0] bits;
    wire [3:0] s = bits[16:13], chk = bits[4:1];
    wire [7:0] c = bits[12:5];

    always @(posedge clk) begin
        if (start) begin
            seen  <= 1'b0;
            cells <= 5'd0;
        end else if (y_en) begin
            if (!seen) begin
                seen   <= y > THR;
                in_run <= 1'b1;
                run    <= 6'd1;
                n      <= 11'd1;
                next   <= FIRST;
            end else begin
                n <= n + 1'b1;
                if (in_run && y > THR) run <= run + (run != 6'h3F);
                else in_run <= 1'b0;
                if (n == next && cells < 5'd19) begin
                    bits  <= {bits[17:0], y > THR};
                    cells <= cells + 1'b1;
                    next  <= next + CELL;
                end
            end
        end
        if (done) begin
            ok     <= cells == 5'd19 && run >= RUN_MIN && bits[18] && !bits[17] && bits[0] && chk == (s ^ c[7:4] ^ c[3:0] ^ 4'hA);
            scheme <= s;
            count  <= c;
        end
    end
endmodule
