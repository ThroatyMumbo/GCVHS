// Assembles the data-tape image into SDRAM slots, one packet write per pk_ack; a lost
// row per parity group is rebuilt at the end of the field, anything else waits for the next pass.
module data_loader (
    input  wire         clk,
    input  wire         rst,
    input  wire         field_start,    // SAV of stored row 0
    input  wire         valid,          // data_slicer
    input  wire         ok,
    input  wire         runin,
    input  wire [1:0]   cls,
    input  wire [7:0]   row,
    input  wire [167:0] bytes,
    output reg          ring_we,        // packet ring: entry e is words {e, 0..8}, byte 2w in the low half
    output reg  [5:0]   ring_addr,
    output reg  [15:0]  ring_data,
    output reg          pk_req,         // pulse: write ring entry pk_req_data[16:15] to slot pk_req_data[14:0]
    output reg  [16:0]  pk_req_data,
    input  wire         pk_ack,         // pulse: that write is done
    output reg          have_session,
    output reg          complete,
    output reg  [7:0]   session,
    output reg  [7:0]   rows,
    output reg  [3:0]   groups,
    output reg  [1:0]   s_cls,
    output reg  [15:0]  n,
    output reg  [15:0]  have,
    output reg  [31:0]  crc_exp,
    output reg  [4:0]   d,              // data bytes per packet
    output reg  [23:0]  lines_ok,       // rows < R while data is arriving: CRC good,
    output reg  [23:0]  lines_bad,      // ... a run-in but a bad CRC,
    output reg  [23:0]  lines_none,     // ... no run-in
    output reg  [15:0]  recovered,      // packets rebuilt from parity
    output reg  [15:0]  dupes,          // packets already held
    output reg  [15:0]  data_fields,    // fields with a good line this session
    output reg  [15:0]  load_fields,    // data_fields when the image completed
    output reg  [7:0]   lost,           // SDRAM writes never acknowledged (should stay 0)
    output reg  [7:0]   drops,          // lines that arrived with one already waiting (should stay 0)
    output reg  [15:0]  misseq,         // CRC-good data lines whose idx breaks the field's row sequence
    output wire         data_active     // a good line in one of the last two fields
);
    localparam [3:0] P_IDLE = 4'd0, P_HDR = 4'd1, P_CLEAR = 4'd2, P_END = 4'd3, P_WRAP = 4'd4,
                     P_BM = 4'd5, P_TEST = 4'd6, P_COPY = 4'd7, P_REQ = 4'd8, P_ACK = 4'd9, P_NEXT = 4'd10,
                     P_SEQ = 4'd11;
    localparam [1:0] S_LINE = 2'd0, S_CAND = 2'd1, S_ACC = 2'd2;  // packet source: pb, cand, acc[g]

    `include "datatape.vh"

    reg  [3:0]   state;
    reg  [31:0]  bm [0:1023];           // one bit per packet held
    reg  [31:0]  bm_q;
    reg  [9:0]   clr;
    reg  [135:0] pb;                    // the line's 17 data bytes, byte 0 on top
    reg  [135:0] acc [0:7];             // per group: XOR of its data rows and parity row
    reg  [1:0]   mcnt [0:7];            // per group: rows lost, saturating at 2
    reg  [7:0]   mrow [0:7];
    reg  [7:0]   par_ok;
    reg  [3:0]   g;
    reg  [7:0]   seen;                  // data rows seen this field, good or not
    reg  [1:0]   lcls;
    reg  [2:0]   pb_k;                  // header idx 0xFFF0 + K - 1
    reg          q_v, q_ok, q_runin;    // the line waiting for P_IDLE
    reg  [1:0]   q_cls;
    reg  [7:0]   q_row;
    reg  [2:0]   q_grp;                 // (row - 1) mod K
    reg  [167:0] q_bytes;
    reg  [135:0] cand;                  // the field's first good line, written once a second agrees with it
    reg          cand_v;
    reg  [7:0]   cand_row;
    reg  [14:0]  cand_idx;
    reg  [2:0]   cand_grp;
    reg  [14:0]  pk_idx;                // the line in pb
    reg  [7:0]   prow;
    reg  [2:0]   pgrp;
    reg  [1:0]   src_sel;
    reg  [14:0]  st_idx;
    reg  [3:0]   w;
    reg  [11:0]  wait_n;
    reg  [1:0]   entry;
    reg  [7:0]   ref_row;
    reg  [14:0]  ref_idx;
    reg          ref_ok, field_good;    // ref_ok: two good lines agreed on the field's row sequence
    reg  [1:0]   quiet;
    reg  signed [17:0] t;
    integer      j;

    wire [15:0]  idx     = q_bytes[167:152];
    wire [135:0] line    = q_bytes[151:16];
    wire [15:0]  h_n     = {pb[111:104], pb[119:112]};
    wire [31:0]  h_crc   = {pb[79:72], pb[87:80], pb[95:88], pb[103:96]};
    wire [7:0]   h_rows  = pb[127:120];
    wire [3:0]   h_k     = {1'b0, pb_k} + 4'd1;
    wire         h_sane  = h_n != 16'd0 && h_n <= 16'd32768 && h_rows <= 8'd240 && h_rows >= {4'd0, h_k} + 8'd2;
    wire         h_new   = !have_session || pb[135:128] != session || h_n != n || h_crc != crc_exp ||
                           lcls != s_cls || h_k != groups;
    wire         same    = have_session && q_cls == s_cls;
    wire         in_rows = have_session && q_row < rows;
    wire [7:0]   m_last  = rows - {4'd0, groups} - 8'd1;       // last data row
    wire [7:0]   par_p   = q_row - m_last - 8'd1;              // the parity group a row past m_last carries
    wire [7:0]   grp_in  = groups == 4'd0 ? 8'd0 : (row - 8'd1) % {4'd0, groups};
    wire         take    = state == P_IDLE && q_v;
    wire [135:0] dmask   = ~(136'd0) << (8 * (17 - d));        // the first d bytes
    wire [143:0] src     = {src_sel == S_ACC ? acc[g[2:0]] : src_sel == S_CAND ? cand : pb, 8'd0};
    wire [7:0]   src_lo  = src[143 - 16 * w -: 8];
    wire [7:0]   src_hi  = src[135 - 16 * w -: 8];
    wire [14:0]  base_idx = ref_ok ? ref_idx : cand_idx;
    wire [7:0]   base_row = ref_ok ? ref_row : cand_row;
    wire signed [17:0] sn = $signed({2'b0, n});
    assign data_active = quiet < 2'd2;

    always @(posedge clk) begin
        if (state == P_CLEAR) bm[clr] <= 32'd0;
        else if (state == P_ACK && pk_ack) bm[st_idx[14:5]] <= bm_q | (32'd1 << st_idx[4:0]);
        bm_q <= bm[st_idx[14:5]];
    end

    always @(posedge clk) begin
        ring_we <= 1'b0;
        pk_req  <= 1'b0;
        if (rst) begin
            state        <= P_IDLE;
            have_session <= 1'b0;
            complete     <= 1'b0;
            have         <= 16'd0;
            lines_ok     <= 24'd0;
            lines_bad    <= 24'd0;
            lines_none   <= 24'd0;
            recovered    <= 16'd0;
            dupes        <= 16'd0;
            data_fields  <= 16'd0;
            load_fields  <= 16'd0;
            lost         <= 8'd0;
            drops        <= 8'd0;
            misseq       <= 16'd0;
            quiet        <= 2'd3;
            entry        <= 2'd0;
            field_good   <= 1'b0;
            q_v          <= 1'b0;
            cand_v       <= 1'b0;
            ref_ok       <= 1'b0;
        end else begin
            if (field_start) begin
                for (j = 0; j < 8; j = j + 1) begin
                    acc[j]  <= 136'd0;
                    mcnt[j] <= 2'd0;
                end
                par_ok     <= 8'd0;
                seen       <= 8'd0;
                ref_ok     <= 1'b0;
                cand_v     <= 1'b0;
                field_good <= 1'b0;
                quiet      <= field_good ? 2'd0 : quiet + (quiet != 2'd3);
                if (field_good && have_session) data_fields <= data_fields + (data_fields != 16'hFFFF);
            end

            if (valid) begin                // held until P_IDLE takes it; a second one while it waits is lost
                if (q_v && !take) drops <= drops + (drops != 8'hFF);
                else begin
                    q_v     <= 1'b1;
                    q_ok    <= ok;
                    q_runin <= runin;
                    q_cls   <= cls;
                    q_row   <= row;
                    q_grp   <= grp_in[2:0];
                    q_bytes <= bytes;
                end
            end else if (take) q_v <= 1'b0;

            case (state)
                P_IDLE: if (q_v) begin
                    pb    <= line;
                    pb_k  <= idx[2:0];
                    lcls  <= q_cls;
                    if (q_ok) field_good <= 1'b1;
                    if (in_rows && data_active) begin
                        if (q_ok) lines_ok <= lines_ok + (lines_ok != 24'hFFFFFF);
                        else if (q_runin) lines_bad <= lines_bad + (lines_bad != 24'hFFFFFF);
                        else lines_none <= lines_none + (lines_none != 24'hFFFFFF);
                    end
                    if (q_ok && q_row == 8'd0 && idx[15:3] == 13'h1FFE) state <= P_HDR;
                    else if (in_rows && q_row >= 8'd1 && q_row <= m_last) begin
                        seen   <= seen + 1'b1;
                        pk_idx <= idx[14:0];
                        prow   <= q_row;
                        pgrp   <= q_grp;
                        if (!(q_ok && same && idx < n)) begin
                            mcnt[q_grp] <= mcnt[q_grp] + (mcnt[q_grp] != 2'd2);
                            mrow[q_grp] <= q_row;
                        end else if (ref_ok || cand_v) begin
                            t     <= $signed({3'b0, base_idx}) + $signed({10'b0, q_row}) - $signed({10'b0, base_row});
                            state <= P_SEQ;
                        end else begin
                            cand_v   <= 1'b1;
                            cand     <= line;
                            cand_idx <= idx[14:0];
                            cand_row <= q_row;
                            cand_grp <= q_grp;
                        end
                    end else if (in_rows && q_row > m_last) begin
                        if (q_ok && same && idx[15:3] == 13'h1FE0 && idx[2:0] < groups && {5'd0, idx[2:0]} == par_p) begin
                            acc[idx[2:0]]    <= acc[idx[2:0]] ^ (line & dmask);
                            par_ok[idx[2:0]] <= 1'b1;
                        end
                        if (q_row == rows - 8'd1) begin
                            g     <= 3'd0;
                            state <= P_END;
                        end
                    end
                end
                P_SEQ:                      // t: the idx this row should carry, mod n
                    if (t < 0) t <= t + sn;
                    else if (t >= sn) t <= t - sn;
                    else if (t[14:0] != pk_idx) begin
                        misseq <= misseq + (misseq != 16'hFFFF);
                        state  <= P_IDLE;
                        if (ref_ok) begin
                            mcnt[pgrp] <= mcnt[pgrp] + (mcnt[pgrp] != 2'd2);
                            mrow[pgrp] <= prow;
                        end else begin      // one of the two is wrong: drop the older, this line waits instead
                            mcnt[cand_grp] <= mcnt[cand_grp] + (mcnt[cand_grp] != 2'd2);
                            mrow[cand_grp] <= cand_row;
                            cand           <= pb;
                            cand_idx       <= pk_idx;
                            cand_row       <= prow;
                            cand_grp       <= pgrp;
                        end
                    end else if (ref_ok) begin
                        acc[pgrp] <= acc[pgrp] ^ (pb & dmask);
                        st_idx    <= pk_idx;
                        src_sel   <= S_LINE;
                        state     <= P_BM;
                    end else begin          // they agree: write the candidate, then this line (P_NEXT)
                        ref_ok        <= 1'b1;
                        ref_row       <= prow;
                        ref_idx       <= pk_idx;
                        cand_v        <= 1'b0;
                        acc[cand_grp] <= acc[cand_grp] ^ (cand & dmask);
                        st_idx        <= cand_idx;
                        src_sel       <= S_CAND;
                        state         <= P_BM;
                    end
                P_HDR: begin
                    state <= P_IDLE;
                    if (h_sane) begin
                        rows <= h_rows;
                        if (h_new) begin
                            have_session <= 1'b0;
                            complete     <= 1'b0;
                            session      <= pb[135:128];
                            groups       <= h_k;
                            n            <= h_n;
                            crc_exp      <= h_crc;
                            s_cls        <= lcls;
                            d            <= dt_data_bytes(lcls);
                            have         <= 16'd0;
                            data_fields  <= 16'd0;
                            load_fields  <= 16'd0;
                            clr          <= 10'd0;
                            state        <= P_CLEAR;
                        end
                    end
                end
                P_CLEAR: begin
                    clr <= clr + 1'b1;
                    if (&clr) begin
                        have_session <= 1'b1;
                        state        <= P_IDLE;
                    end
                end
                P_END:                      // group g lost one row and its parity is good: acc[g] is that row
                    if (g == groups || seen != m_last) state <= P_IDLE;
                    else if (mcnt[g[2:0]] == 2'd1 && par_ok[g[2:0]] && ref_ok) begin
                        t     <= $signed({3'b0, ref_idx}) + $signed({10'b0, mrow[g[2:0]]}) - $signed({10'b0, ref_row});
                        state <= P_WRAP;
                    end else g <= g + 1'b1;
                P_WRAP:
                    if (t < 0) t <= t + sn;
                    else if (t >= sn) t <= t - sn;
                    else begin
                        st_idx  <= t[14:0];
                        src_sel <= S_ACC;
                        state   <= P_BM;
                    end
                P_BM: state <= P_TEST;      // bm_q follows st_idx
                P_TEST:
                    if (bm_q[st_idx[4:0]]) begin
                        dupes <= dupes + (dupes != 16'hFFFF);
                        state <= P_NEXT;
                    end else begin
                        w     <= 4'd0;
                        state <= P_COPY;
                    end
                P_COPY: begin
                    ring_we   <= 1'b1;
                    ring_addr <= {entry, w};
                    ring_data <= {src_hi, src_lo};
                    w         <= w + 1'b1;
                    if (w == 4'd8) state <= P_REQ;
                end
                P_REQ: begin
                    pk_req      <= 1'b1;
                    pk_req_data <= {entry, st_idx};
                    entry       <= entry + 1'b1;
                    wait_n      <= 12'd0;
                    state       <= P_ACK;
                end
                P_ACK:                      // the bitmap bit is set with the ack (bm block above)
                    if (pk_ack) begin
                        have <= have + 1'b1;
                        if (have + 1'b1 == n) begin
                            complete    <= 1'b1;
                            load_fields <= data_fields + 1'b1;
                        end
                        if (src_sel == S_ACC) recovered <= recovered + (recovered != 16'hFFFF);
                        state <= P_NEXT;
                    end else if (&wait_n) begin
                        lost  <= lost + (lost != 8'hFF);
                        state <= P_NEXT;
                    end else wait_n <= wait_n + 1'b1;
                P_NEXT:                     // back to the parity groups, on to the line after a candidate, or idle
                    if (src_sel == S_ACC) begin
                        g     <= g + 1'b1;
                        state <= P_END;
                    end else if (src_sel == S_CAND) begin
                        acc[pgrp] <= acc[pgrp] ^ (pb & dmask);
                        st_idx    <= pk_idx;
                        src_sel   <= S_LINE;
                        state     <= P_BM;
                    end else state <= P_IDLE;
                default: state <= P_IDLE;
            endcase
        end
    end
endmodule
