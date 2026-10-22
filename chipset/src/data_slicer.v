// Slices one data-tape line off a stored line's luma. The run-in gives the cell width
// and threshold; cells are timed from its falling edge, nudged a quarter of the error at each transition.
module data_slicer #(
    parameter [7:0] THR0 = 8'd100       // finds the run-in; bits use the measured threshold
) (
    input  wire         clk,
    input  wire         start,          // SAV of a stored line
    input  wire [7:0]   row,            // with start
    input  wire         y_en,
    input  wire [7:0]   y,
    input  wire         done,           // EAV of the line
    output reg          valid,          // pulse after every done
    output reg          ok,             // with valid: all bits in and the CRC held
    output reg          runin,          // with valid: a run-in was found
    output reg  [1:0]   cls,            // cell width in DAC samples - 4
    output reg  [7:0]   line_row,
    output reg  [167:0] bytes           // byte 0 in [167:160], left-aligned: B = 21, 17, 14, 12 by cls
);
    localparam [2:0] S_SEEK = 3'd0, S_WHITE = 3'd1, S_BITS = 3'd2, S_DONE = 3'd3, S_FAIL = 3'd4;
    reg  [2:0]  state;
    reg  [9:0]  n, n0;
    reg  [7:0]  hist [0:7];             // hist[k] = sample n - 1 - k
    reg  [9:0]  bsum, wsum;
    reg  [7:0]  thr;
    reg  signed [19:0] pos;             // next cell centre, Q8 samples
    reg  [10:0] s_q8;
    reg  [7:0]  nbits;                  // bits in so far
    reg  [7:0]  total;
    reg         prev_hi, crossed;
    reg  [15:0] lfsr, crc;
    reg  [167:0] sr;
    integer     i;

    wire [9:0]  k = n - n0;
    wire        hi = y > thr;
    wire signed [19:0] e_q8 = $signed({2'b0, n, 8'h00}) - 20'sd128;
    wire signed [19:0] err  = e_q8 - (pos - $signed({9'b0, s_q8[10:1]}));
    wire signed [19:0] pos_c = (hi != prev_hi && !crossed) ? pos + (err >>> 2) : pos;
    wire [11:0] at = pos[19:8] + pos[7];   // nearest sample to the centre
    wire        bit_in = hi ^ lfsr[0];
    wire [10:0] tsum = {1'b0, bsum} + wsum + y;
    wire [1:0]  kcls = k < 10'd17 ? 2'd0 : k < 10'd21 ? 2'd1 : k < 10'd25 ? 2'd2 : 2'd3;   // run-in 4 cells long
    wire [10:0] kstep = step(kcls);

    function [10:0] step(input [1:0] c);     // w DAC samples * 13.5 / 14.318 Y samples, Q8
        case (c)
            2'd0: step = 11'd965;
            2'd1: step = 11'd1207;
            2'd2: step = 11'd1448;
            default: step = 11'd1690;
        endcase
    endfunction
    `include "datatape.vh"

    always @(posedge clk) begin
        valid <= 1'b0;
        if (start) begin
            state    <= S_SEEK;
            n        <= 10'd0;
            line_row <= row;
        end else if (y_en) begin
            n <= n + 1'b1;
            hist[0] <= y;
            for (i = 1; i < 8; i = i + 1) hist[i] <= hist[i - 1];
            case (state)
                S_SEEK:
                    if (n >= 10'd8 && y > THR0) begin
                        n0    <= n;
                        bsum  <= hist[4] + hist[5] + hist[6] + hist[7];
                        wsum  <= 10'd0;
                        state <= S_WHITE;
                    end
                S_WHITE: begin
                    if (k >= 10'd3 && k <= 10'd5) wsum <= wsum + y;
                    if (k == 10'd6) thr <= tsum[10:3];
                    if (k >= 10'd8 && !hi) begin
                        if (k < 10'd13 || k > 10'd28) state <= S_FAIL;
                        else begin
                            cls     <= kcls;
                            s_q8    <= kstep;
                            total   <= {dt_line_bytes(kcls), 3'b000};
                            pos     <= e_q8 + $signed({9'b0, kstep}) + $signed({10'b0, kstep[10:1]});   // past the black cell, to mid-cell
                            prev_hi <= 1'b0;
                            crossed <= 1'b0;
                            nbits   <= 8'd0;
                            lfsr    <= {line_row, ~line_row};
                            crc     <= 16'hFFFF;
                            state   <= S_BITS;
                        end
                    end else if (k > 10'd28) state <= S_FAIL;
                end
                S_BITS: begin
                    prev_hi <= hi;
                    if (hi != prev_hi) crossed <= 1'b1;
                    pos <= pos_c;
                    if ({2'b0, n} >= at) begin
                        pos     <= pos_c + $signed({9'b0, s_q8});
                        crossed <= 1'b0;
                        sr      <= {sr[166:0], bit_in};
                        lfsr    <= {1'b0, lfsr[15:1]} ^ (lfsr[0] ? 16'hB400 : 16'h0000);
                        if (nbits < total - 8'd16)
                            crc <= {crc[14:0], 1'b0} ^ ((crc[15] ^ bit_in) ? 16'h1021 : 16'h0000);
                        nbits   <= nbits + 1'b1;
                        if (nbits + 1'b1 == total) state <= S_DONE;
                    end
                end
                default: ;
            endcase
        end
        if (done) begin
            valid <= 1'b1;
            runin <= state == S_BITS || state == S_DONE;
            ok    <= state == S_DONE && crc == sr[15:0];
            case (cls)                  // left-align the B bytes
                2'd0: bytes <= sr;
                2'd1: bytes <= {sr[135:0], 32'd0};
                2'd2: bytes <= {sr[111:0], 56'd0};
                default: bytes <= {sr[95:0], 72'd0};
            endcase
        end
    end
endmodule
