// Dual-clock FIFO with Gray-coded pointers. rdata is valid while !empty (show-ahead via a registered read).
module async_fifo #(
    parameter integer AW = 9,
    parameter integer DW = 8
) (
    input  wire          wclk,
    input  wire          wrst,
    input  wire          we,
    input  wire [DW-1:0] wdata,
    output wire          full,
    output wire [AW:0]   wfill,             // write side's view, lags reads by a few wclk
    input  wire          rclk,
    input  wire          rrst,
    input  wire          re,                // pop: only while !empty
    output wire [DW-1:0] rdata,
    output wire          empty,
    output wire [AW:0]   rfill
);
    reg [DW-1:0] mem [0:(1 << AW) - 1];

    function [AW:0] g2b(input [AW:0] g);
        integer k;
        begin
            g2b[AW] = g[AW];
            for (k = AW - 1; k >= 0; k = k - 1) g2b[k] = g2b[k + 1] ^ g[k];
        end
    endfunction

    // rb_w: the read pointer in binary, two clocks late (a late read pointer only makes full cautious).
    // full is registered from both possible next write pointers, so the write enable is one gate off a flop.
    reg  [AW:0] wp, wp1, wg, rg_m, rg_s, rb_b, rb_w;
    reg  [AW:0] rp, rg, wg_m, wg_s;
    reg         full_r;
    wire        wr = we && !full_r;
    wire [AW:0] fill0 = wp - rb_w, fill1 = wp1 - rb_w;
    always @(posedge wclk) begin
        {rg_s, rg_m} <= {rg_m, rg};
        rb_b         <= g2b(rg_s);
        rb_w         <= rb_b;
        if (wrst) begin
            wp     <= 0;
            wp1    <= 1;
            wg     <= 0;
            full_r <= 1'b0;
        end else begin
            full_r <= wr ? fill1[AW] : fill0[AW];
            if (wr) begin
                mem[wp[AW-1:0]] <= wdata;
                wp  <= wp1;
                wp1 <= wp1 + 1'b1;
                wg  <= wp1 ^ (wp1 >> 1);
            end
        end
    end
    assign wfill = fill0;
    assign full  = full_r;

    // rp addresses the head; the RAM output register holds mem[rp] one cycle after rp moves, so empty
    // stays up for that cycle.
    reg  [DW-1:0] q;
    reg         fresh;
    wire [AW:0] rp_n = rp + (re && !empty);
    always @(posedge rclk) begin
        {wg_s, wg_m} <= {wg_m, wg};
        q <= mem[rp_n[AW-1:0]];
        if (rrst) begin
            rp    <= 0;
            rg    <= 0;
            fresh <= 1'b0;
        end else begin
            rp    <= rp_n;
            rg    <= rp_n ^ (rp_n >> 1);
            fresh <= rp_n != g2b(wg_s);
        end
    end
    assign rfill = g2b(wg_s) - rp;
    assign empty = !fresh || rfill == 0;
    assign rdata = q;
endmodule
