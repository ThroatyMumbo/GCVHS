// IS42S16320 at 100 MHz, CL3, burst 1: a video line is one row, a READ or WRITE every cycle (~7.3 us); a start-up test
// row (SPARE_ROW) measures the read latency. Data-tape packets: 16-word slots in bank 1, 64 per row, after video lines and refreshes.
module sdram_ctrl (
    input  wire        clk,             // 100 MHz; DRAM_CLK runs 4 ns ahead of it
    input  wire        rst,
    input  wire        wr_req,          // pulse: copy write line buffer half wr_half to row wr_row
    input  wire [12:0] wr_row,
    input  wire        wr_half,
    output wire [10:0] wl_addr,         // write line buffer read port, data one cycle later
    input  wire [15:0] wl_data,
    input  wire        rd_req,          // pulse: copy row rd_row to read line buffer slot rd_slot
    input  wire [12:0] rd_row,
    input  wire [1:0]  rd_slot,
    output reg         rl_we,
    output reg  [11:0] rl_addr,
    output reg  [15:0] rl_data,
    input  wire        pk_wr_req,       // pulse: copy packet ring entry pk_wr_entry to slot pk_wr_idx
    input  wire [1:0]  pk_wr_entry,
    input  wire [14:0] pk_wr_idx,
    output wire [5:0]  pk_addr,         // packet ring read port, data one cycle later
    input  wire [15:0] pk_data,
    input  wire        pk_rd_req,       // pulse: read slot pk_rd_idx
    input  wire [14:0] pk_rd_idx,
    output reg         pk_rv,
    output reg  [3:0]  pk_rcol,
    output reg  [15:0] pk_rdata,
    output reg         pk_wr_done,      // pulse: the packet write has landed
    output reg         ready,           // init and self-test done
    output reg         bist_ok,
    output reg  [2:0]  lat,
    output wire [49:0] bist_counts,     // test-row matches out of 720 for latencies 3-7
    output reg  [12:0] sd_addr,
    output reg  [1:0]  sd_ba,
    output reg         sd_cs_n,
    output reg         sd_ras_n,
    output reg         sd_cas_n,
    output reg         sd_we_n,
    output reg         sd_dqm,
    inout  wire [15:0] sd_dq
);
    `include "sdram_map.vh"
    localparam [3:0] C_NOP = 4'b0111, C_ACT = 4'b0011, C_READ = 4'b0101, C_WRITE = 4'b0100,
                     C_PRE = 4'b0010, C_REF = 4'b0001, C_MRS = 4'b0000;
    localparam [3:0] S_PRE_ALL = 4'd0, S_INIT_REF = 4'd1, S_MRS = 4'd2, S_IDLE = 4'd3, S_ACT = 4'd4,
                     S_XFER = 4'd5, S_PRE = 4'd6, S_REF = 4'd7, S_WAIT = 4'd8;
    localparam [12:0] MODE = 13'h030;   // burst length 1, sequential, CAS latency 3
    localparam [9:0]  COLS = 10'd720;
    localparam integer REF_EVERY = 700; // cycles; 8192 rows per 64 ms needs one per 781

    reg  [3:0]  state, after;
    reg  [14:0] wait_n;
    reg  [2:0]  init_refs;
    reg  [9:0]  ref_timer;
    reg  [3:0]  ref_pend;
    reg         wpend, rpend, whalf;
    reg  [1:0]  rslot, op_line;         // read line buffer slot
    reg  [12:0] wrow, rrow;
    reg         op_write, op_bist, op_half, op_pk;
    reg  [12:0] op_row;
    reg  [1:0]  op_entry;
    reg  [5:0]  op_slot;                // slot within the row
    reg         pk_wpend, pk_rpend;
    reg  [1:0]  pk_went;
    reg  [14:0] pk_widx, pk_ridx;
    wire [9:0]  op_last = op_pk ? 10'd16 : COLS;
    reg  [9:0]  col, xc1;               // col addresses the line buffer; xc1 is issued a cycle later
    reg         xv1;
    reg  [1:0]  bist_phase;
    reg  [15:0] dq_out, dq_in;
    reg         dq_oe;
    reg  [7:0]  tok_v;                  // tok_*[j]: the READ decided j + 1 cycles ago
    reg  [9:0]  tok_col [0:7];
    reg  [9:0]  cnt [3:7];
    integer     i;
    // S_IDLE goes to S_REF: a backlog of 8 beats video, otherwise a refresh waits for video and goes before packets.
    wire        take = state == S_IDLE && ready && (ref_pend >= 4'd8 || (!wpend && !rpend && ref_pend != 4'd0));

    assign sd_dq       = dq_oe ? dq_out : 16'bz;
    assign wl_addr     = {op_half, col};
    assign pk_addr     = {op_entry, col[3:0]};
    assign bist_counts = {cnt[3], cnt[4], cnt[5], cnt[6], cnt[7]};

    function [15:0] pattern(input [9:0] c);
        pattern = {c[5:0], ~c};
    endfunction

    always @(posedge clk) begin
        {sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n} <= C_NOP;
        dq_in <= sd_dq;
        rl_we <= 1'b0;
        pk_rv <= 1'b0;
        pk_wr_done <= 1'b0;

        tok_v      <= {tok_v[6:0], state == S_XFER && xv1 && !op_write};
        tok_col[0] <= xc1;
        for (i = 1; i < 8; i = i + 1) tok_col[i] <= tok_col[i - 1];

        if (op_bist && !op_write) begin
            for (i = 3; i <= 7; i = i + 1)
                if (tok_v[i - 1] && dq_in == pattern(tok_col[i - 1])) cnt[i] <= cnt[i] + 1'b1;
        end else if (!op_bist && !op_write && tok_v[lat - 1'b1]) begin
            if (op_pk) begin
                pk_rv    <= 1'b1;
                pk_rcol  <= tok_col[lat - 1'b1][3:0];
                pk_rdata <= dq_in;
            end else begin
                rl_we   <= 1'b1;
                rl_addr <= {op_line, tok_col[lat - 1'b1]};
                rl_data <= dq_in;
            end
        end

        if (rst) begin
            state      <= S_WAIT;       // 250 us of NOPs after power-up
            wait_n     <= 15'd25000;
            after      <= S_PRE_ALL;
            ready      <= 1'b0;
            bist_ok    <= 1'b0;
            lat        <= 3'd5;
            bist_phase <= 2'd0;
            wpend      <= 1'b0;
            rpend      <= 1'b0;
            pk_wpend   <= 1'b0;
            pk_rpend   <= 1'b0;
            op_pk      <= 1'b0;
            ref_timer  <= 10'd0;
            ref_pend   <= 4'd0;
            op_write   <= 1'b0;
            op_bist    <= 1'b0;
            dq_oe      <= 1'b0;
            sd_dqm     <= 1'b1;
            for (i = 3; i <= 7; i = i + 1) cnt[i] <= 10'd0;
        end else begin
            case (state)
                S_WAIT:
                    if (wait_n <= 15'd1) state <= after;
                    else wait_n <= wait_n - 1'b1;
                S_PRE_ALL: begin
                    {sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n} <= C_PRE;
                    sd_addr[10] <= 1'b1;        // all banks
                    init_refs   <= 3'd0;
                    {state, wait_n, after} <= {S_WAIT, 15'd3, S_INIT_REF};
                end
                S_INIT_REF: begin
                    {sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n} <= C_REF;
                    init_refs <= init_refs + 1'b1;
                    {state, wait_n, after} <= {S_WAIT, 15'd8, init_refs == 3'd7 ? S_MRS : S_INIT_REF};
                end
                S_MRS: begin
                    {sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n} <= C_MRS;
                    sd_addr <= MODE;
                    sd_ba   <= 2'd0;
                    {state, wait_n, after} <= {S_WAIT, 15'd3, S_IDLE};
                end
                S_IDLE: begin
                    sd_dqm <= 1'b0;
                    if (!ready) begin
                        op_bist <= 1'b1;
                        op_row  <= SPARE_ROW;
                        op_half <= 1'b0;
                        case (bist_phase)
                            2'd0: begin op_write <= 1'b1; bist_phase <= 2'd1; state <= S_ACT; end
                            2'd1: begin op_write <= 1'b0; bist_phase <= 2'd2; state <= S_ACT; end
                            default: begin
                                if      (cnt[5] == COLS) lat <= 3'd5;
                                else if (cnt[4] == COLS) lat <= 3'd4;
                                else if (cnt[6] == COLS) lat <= 3'd6;
                                else if (cnt[3] == COLS) lat <= 3'd3;
                                else if (cnt[7] == COLS) lat <= 3'd7;
                                bist_ok <= cnt[3] == COLS || cnt[4] == COLS || cnt[5] == COLS ||
                                           cnt[6] == COLS || cnt[7] == COLS;
                                op_bist <= 1'b0;
                                ready   <= 1'b1;
                            end
                        endcase
                    end else if (ref_pend >= 4'd8) begin
                        state <= S_REF;
                    end else if (wpend) begin
                        wpend    <= 1'b0;
                        op_write <= 1'b1;
                        op_pk    <= 1'b0;
                        op_row   <= wrow;
                        op_half  <= whalf;
                        state    <= S_ACT;
                    end else if (rpend) begin
                        rpend    <= 1'b0;
                        op_write <= 1'b0;
                        op_pk    <= 1'b0;
                        op_row   <= rrow;
                        op_line  <= rslot;
                        state    <= S_ACT;
                    end else if (ref_pend != 4'd0) begin
                        state <= S_REF;
                    end else if (pk_wpend) begin
                        pk_wpend <= 1'b0;
                        op_write <= 1'b1;
                        op_pk    <= 1'b1;
                        op_row   <= {4'd0, pk_widx[14:6]};
                        op_slot  <= pk_widx[5:0];
                        op_entry <= pk_went;
                        state    <= S_ACT;
                    end else if (pk_rpend) begin
                        pk_rpend <= 1'b0;
                        op_write <= 1'b0;
                        op_pk    <= 1'b1;
                        op_row   <= {4'd0, pk_ridx[14:6]};
                        op_slot  <= pk_ridx[5:0];
                        state    <= S_ACT;
                    end
                end
                S_ACT: begin
                    {sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n} <= C_ACT;
                    sd_ba   <= {1'b0, op_pk};
                    sd_addr <= op_row;
                    col     <= 10'd0;
                    xv1     <= 1'b0;
                    {state, wait_n, after} <= {S_WAIT, 15'd2, S_XFER};
                end
                S_XFER: begin
                    if (col != op_last) col <= col + 1'b1;
                    xv1 <= col != op_last;
                    xc1 <= col;
                    if (xv1) begin
                        {sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n} <= op_write ? C_WRITE : C_READ;
                        // A10 low: no auto precharge
                        sd_addr <= op_pk ? {3'b000, op_slot, xc1[3:0]} : {3'b000, xc1};
                        if (op_write) begin
                            dq_out <= op_bist ? pattern(xc1) : op_pk ? pk_data : wl_data;
                            dq_oe  <= 1'b1;
                        end
                    end else if (col == op_last) begin  // last column issued: let writes land and reads drain
                        dq_oe <= 1'b0;
                        {state, wait_n, after} <= {S_WAIT, 15'd8, S_PRE};
                    end
                end
                S_PRE: begin
                    {sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n} <= C_PRE;
                    sd_addr[10] <= 1'b1;
                    pk_wr_done  <= op_pk && op_write;
                    {state, wait_n, after} <= {S_WAIT, 15'd3, S_IDLE};
                end
                S_REF: begin
                    {sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n} <= C_REF;
                    {state, wait_n, after} <= {S_WAIT, 15'd8, S_IDLE};
                end
                default: state <= S_IDLE;
            endcase

            if (ref_timer == REF_EVERY - 1) ref_timer <= 10'd0;
            else ref_timer <= ref_timer + 1'b1;
            ref_pend <= ref_pend + (ref_timer == REF_EVERY - 1 && ref_pend != 4'd15) - take;

            if (wr_req) begin
                wpend <= 1'b1;
                wrow  <= wr_row;
                whalf <= wr_half;
            end
            if (rd_req) begin
                rpend <= 1'b1;
                rrow  <= rd_row;
                rslot <= rd_slot;
            end
            if (pk_wr_req) begin
                pk_wpend <= 1'b1;
                pk_went  <= pk_wr_entry;
                pk_widx  <= pk_wr_idx;
            end
            if (pk_rd_req) begin
                pk_rpend <= 1'b1;
                pk_ridx  <= pk_rd_idx;
            end
        end
    end
endmodule
