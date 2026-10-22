// BT.656 field/line statistics, measured from the FF 00 00 XY codes, never by counting clocks:
// the ADV7180's clock is fixed and it varies samples per line to absorb VCR timebase error.
module bt656_rx #(
    parameter integer CLK_HZ = 27_000_000
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [7:0]  data,
    output reg  [7:0]  fields_per_sec,
    output reg  [9:0]  lines_f0,        // total lines in the last F=0 / F=1 field
    output reg  [9:0]  lines_f1,
    output reg  [9:0]  active_f0,       // lines with V=0 in the last F=0 / F=1 field
    output reg  [9:0]  active_f1,
    output reg  [11:0] active_bytes,    // SAV to EAV, last line (1440 = 720 pixels)
    output reg  [11:0] line_min,        // EAV to EAV over the last second
    output reg  [11:0] line_max,
    output reg  [15:0] parity_errs,     // since reset: fields breaking the 480i sequence (see below)
    output reg  [15:0] bad_codes,       // last second: codes failing the XY protection bits
    output reg         codes_seen,      // last second had valid codes
    output reg         parity_ok        // last second had fields, none breaking the sequence
);
    // A field breaks the 480i sequence if it repeats the previous F or line count: the ADV7180 toggles F
    // on 240p too (all fields 262 lines); real interlace alternates 262 and 263.
    reg  [7:0] b0, b1, b2, b3;
    wire       is_code = b3 == 8'hFF && b2 == 8'h00 && b1 == 8'h00;
    wire       f = b0[6], v = b0[5], h = b0[4];
    wire       prot_ok = b0[7] && b0[3:0] == {v ^ h, f ^ h, f ^ v, f ^ v ^ h};
    wire       eav = is_code && prot_ok && h;
    wire       sav = is_code && prot_ok && !h;

    reg  [11:0] since_eav, since_sav;
    reg  [9:0]  line_n, active_n, last_lines;
    reg         prev_v, cur_f, last_f, have_last;
    reg  [24:0] sec;
    reg  [7:0]  fields_w;
    reg  [11:0] min_w, max_w;
    reg  [15:0] bad_w;
    reg         good_w, repeat_w;

    always @(posedge clk) begin
        {b3, b2, b1, b0} <= {b2, b1, b0, data};
        if (rst) begin
            since_eav      <= 12'd0;
            since_sav      <= 12'd0;
            line_n         <= 10'd0;
            active_n       <= 10'd0;
            prev_v         <= 1'b0;
            have_last      <= 1'b0;
            parity_errs    <= 16'd0;
            sec            <= 25'd0;
            fields_w       <= 8'd0;
            min_w          <= 12'hFFF;
            max_w          <= 12'd0;
            bad_w          <= 16'd0;
            good_w         <= 1'b0;
            repeat_w       <= 1'b0;
            fields_per_sec <= 8'd0;
            codes_seen     <= 1'b0;
            parity_ok      <= 1'b0;
        end else begin
            since_eav <= eav ? 12'd1 : since_eav + (since_eav != 12'hFFF);
            since_sav <= sav ? 12'd1 : since_sav + (since_sav != 12'hFFF);
            if (is_code && !prot_ok) bad_w <= bad_w + (bad_w != 16'hFFFF);
            if (is_code && prot_ok) good_w <= 1'b1;

            if (eav) begin
                active_bytes <= since_sav - 12'd4;
                if (since_eav < min_w) min_w <= since_eav;
                if (since_eav > max_w) max_w <= since_eav;
                prev_v <= v;
                if (v && !prev_v) begin         // entering vertical blanking: the previous field ended
                    if (cur_f) begin
                        lines_f1  <= line_n;
                        active_f1 <= active_n;
                    end else begin
                        lines_f0  <= line_n;
                        active_f0 <= active_n;
                    end
                    if (have_last && (cur_f == last_f || line_n == last_lines)) begin
                        parity_errs <= parity_errs + (parity_errs != 16'hFFFF);
                        repeat_w    <= 1'b1;
                    end
                    last_f     <= cur_f;
                    last_lines <= line_n;
                    have_last <= 1'b1;
                    fields_w  <= fields_w + 1'b1;
                    line_n    <= 10'd1;
                    active_n  <= 10'd0;
                end else begin
                    line_n <= line_n + (line_n != 10'h3FF);
                    if (!v) begin
                        active_n <= active_n + (active_n != 10'h3FF);
                        cur_f    <= f;
                    end
                end
            end

            // An event landing on the window rollover cycle is dropped; harmless for a readout.
            if (sec == CLK_HZ - 1) begin
                sec            <= 25'd0;
                fields_per_sec <= fields_w;
                line_min       <= min_w;
                line_max       <= max_w;
                bad_codes      <= bad_w;
                codes_seen     <= good_w;
                parity_ok      <= fields_w != 8'd0 && !repeat_w;
                fields_w       <= 8'd0;
                min_w          <= 12'hFFF;
                max_w          <= 12'd0;
                bad_w          <= 16'd0;
                good_w         <= 1'b0;
                repeat_w       <= 1'b0;
            end else sec <= sec + 1'b1;
        end
    end
endmodule
