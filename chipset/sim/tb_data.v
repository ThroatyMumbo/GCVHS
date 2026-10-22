// Data-tape path from luma to verified image: data_slicer, data_loader, the packet ring and CDC,
// sdram_ctrl against a behavioural SDRAM, data_verify. Stimulus from tools/datatape.py stim.
`timescale 1ns / 1ps
module tb_data;
    `include "out/params.vh"
    localparam integer LINE = 720;

    reg clk27 = 0, clk100 = 0;
    always #18.518 clk27 = ~clk27;
    always #5 clk100 = ~clk100;

    reg rst27 = 1, rst100 = 1;
    reg [7:0] stim [0:FIELDS * 240 * LINE - 1];
    reg [7:0] image [0:N * D - 1];
    initial begin
        $readmemh("out/stim.hex", stim);
        $readmemh("out/image.hex", image);
    end

    reg        st_sav = 0, st_eav = 0, y_en = 0;
    reg [7:0]  st_row = 0, y = 0;

    wire         valid, ok, runin;
    wire [1:0]   cls;
    wire [7:0]   line_row;
    wire [167:0] bytes;
    data_slicer slicer (
        .clk(clk27), .start(st_sav), .row(st_row), .y_en(y_en), .y(y), .done(st_eav),
        .valid(valid), .ok(ok), .runin(runin), .cls(cls), .line_row(line_row), .bytes(bytes)
    );

    wire        ring_we, pk_req, have_session, complete, data_active;
    wire [5:0]  ring_addr;
    wire [15:0] ring_data;
    wire [16:0] pk_req_data;
    wire [7:0]  session, rows, lost;
    wire [3:0]  groups;
    wire [1:0]  s_cls;
    wire [15:0] n, have, recovered, dupes, data_fields, load_fields;
    wire [31:0] crc_exp;
    wire [4:0]  d;
    wire [23:0] lines_ok, lines_bad, lines_none;
    wire [7:0]  drops;
    wire [15:0] misseq;
    wire        pk_ack, pk_wr_done;
    // A forged line passes its CRC with a wrong idx: it replaces the sliced line when inj is set.
    reg         inj = 0;
    reg  [15:0] inj_idx = 0;
    wire [167:0] inj_bytes = {inj_idx, {17{8'hC3}}, 16'h0000};
    data_loader loader (
        .clk(clk27), .rst(rst27), .field_start(st_sav && st_row == 8'd0),
        .valid(valid), .ok(inj | ok), .runin(inj | runin), .cls(cls), .row(line_row),
        .bytes(inj ? inj_bytes : bytes),
        .ring_we(ring_we), .ring_addr(ring_addr), .ring_data(ring_data), .pk_req(pk_req), .pk_req_data(pk_req_data),
        .pk_ack(pk_ack),
        .have_session(have_session), .complete(complete), .session(session), .rows(rows), .groups(groups),
        .s_cls(s_cls), .n(n), .have(have), .crc_exp(crc_exp), .d(d),
        .lines_ok(lines_ok), .lines_bad(lines_bad), .lines_none(lines_none), .recovered(recovered),
        .dupes(dupes), .data_fields(data_fields), .load_fields(load_fields), .lost(lost), .drops(drops), .misseq(misseq),
        .data_active(data_active)
    );
    req_cdc #(.W(1)) ack_cdc (
        .sclk(clk100), .sreq(pk_wr_done), .sdata(1'b0), .dclk(clk27), .dreq(pk_ack), .ddata()
    );

    wire [5:0]  pk_addr;
    wire [15:0] pk_data;
    dc_ram #(.AW(6)) ring (
        .wclk(clk27), .we(ring_we), .waddr(ring_addr), .wdata(ring_data),
        .rclk(clk100), .raddr(pk_addr), .rdata(pk_data)
    );
    wire        sd_pk_req;
    wire [16:0] sd_pk_data;
    req_cdc #(.W(17)) pk_cdc (
        .sclk(clk27), .sreq(pk_req), .sdata(pk_req_data),
        .dclk(clk100), .dreq(sd_pk_req), .ddata(sd_pk_data)
    );

    wire        rd_req, pk_rv, ready, bist_ok, sd_dqm;
    wire [14:0] rd_idx;
    wire [3:0]  pk_rcol;
    wire [15:0] pk_rdata;
    wire [2:0]  lat;
    wire [12:0] sd_addr;
    wire [1:0]  sd_ba;
    wire        sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n;
    wire [15:0] sd_dq;
    sdram_ctrl sdram (
        .clk(clk100), .rst(rst100),
        .wr_req(1'b0), .wr_row(13'd0), .wr_half(1'b0), .wl_addr(), .wl_data(16'd0),
        .rd_req(1'b0), .rd_row(13'd0), .rd_slot(2'd0), .rl_we(), .rl_addr(), .rl_data(),
        .pk_wr_req(sd_pk_req), .pk_wr_entry(sd_pk_data[16:15]), .pk_wr_idx(sd_pk_data[14:0]),
        .pk_addr(pk_addr), .pk_data(pk_data),
        .pk_rd_req(rd_req), .pk_rd_idx(rd_idx), .pk_rv(pk_rv), .pk_rcol(pk_rcol), .pk_rdata(pk_rdata),
        .pk_wr_done(pk_wr_done), .ready(ready), .bist_ok(bist_ok), .lat(lat), .bist_counts(),
        .sd_addr(sd_addr), .sd_ba(sd_ba), .sd_cs_n(sd_cs_n), .sd_ras_n(sd_ras_n),
        .sd_cas_n(sd_cas_n), .sd_we_n(sd_we_n), .sd_dqm(sd_dqm), .sd_dq(sd_dq)
    );

    // SDRAM: CAS latency 3, bank 0 row 1000 (the self-test) and bank 1 rows 0-1023 are enough here.
    reg  [15:0] mem [0:(1 << 21) - 1];
    reg  [12:0] open_row [0:3];
    reg  [15:0] rq [0:3];
    reg  [3:0]  rq_v;
    reg  [15:0] dq_drv;
    reg         dq_en = 0;
    assign sd_dq = dq_en ? dq_drv : 16'bz;
    wire [3:0]  cmd = {sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n};
    wire [20:0] maddr = {sd_ba[0], open_row[sd_ba][9:0], sd_addr[9:0]};
    always @(posedge clk100) begin
        rq_v <= {rq_v[2:0], 1'b0};
        rq[1] <= rq[0];
        rq[2] <= rq[1];
        if (cmd == 4'b0011) open_row[sd_ba] <= sd_addr;
        if (cmd == 4'b0100) mem[maddr] <= sd_dq;
        if (cmd == 4'b0101) begin
            rq_v[0] <= 1'b1;
            rq[0]   <= mem[maddr];
        end
        dq_en  <= #1 rq_v[2];
        dq_drv <= #1 rq[2];
    end

    reg  [14:0] peek_idx = 0;
    wire        verified, verify_ok;
    wire [31:0] crc_got;
    wire [14:0] peek_at;
    wire [135:0] peek;
    reg         img_req = 0, img_done = 0;
    reg  [15:0] img_first = 0, img_count = 0;
    reg  [4:0]  img_sel = 0;
    wire        img_ready;
    wire [14:0] img_idx;
    wire [7:0]  img_byte;
    data_verify verify (
        .clk(clk100), .rst(rst100), .complete(complete), .n(n), .d(d), .crc_exp(crc_exp),
        .peek_idx(peek_idx), .rd_req(rd_req), .rd_idx(rd_idx), .rv(pk_rv), .rcol(pk_rcol), .rdata(pk_rdata),
        .verified(verified), .verify_ok(verify_ok), .crc_got(crc_got), .peek_at(peek_at), .peek(peek),
        .img_req(img_req), .img_first(img_first), .img_count(img_count), .img_ready(img_ready), .img_idx(img_idx),
        .img_sel(img_sel), .img_byte(img_byte), .img_done(img_done)
    );

    // Game-CPU side of img_*, as cpu_up: takes a slot on img_ready, reads its D bytes, then pulses img_done.
    // A queued request goes out at once, or (q_on_done) in the same cycle as the next img_done.
    reg         q_req = 0, q_on_done = 0, c_busy = 0;
    reg  [15:0] q_first, q_count;
    reg  [7:0]  c_t = 0;
    reg  [14:0] img_log [0:255];
    integer     img_n = 0, img_bad = 0, p_done = -1;
    always @(posedge clk100) begin
        img_req  <= 1'b0;
        img_done <= 1'b0;
        if (q_req && !q_on_done) begin
            img_req   <= 1'b1;
            img_first <= q_first;
            img_count <= q_count;
            q_req     <= 1'b0;
        end
        if (c_busy) begin
            c_t <= c_t + 1'b1;
            if (c_t < D) img_sel <= c_t[4:0];
            if (c_t >= 1 && c_t <= D && img_byte !== image[img_idx * D + c_t - 1]) img_bad = img_bad + 1;
            if (c_t == 8'd40) begin
                img_done <= 1'b1;
                c_busy   <= 1'b0;
                img_log[img_n & 255] = img_idx;
                img_n = img_n + 1;
                if (q_req && q_on_done) begin
                    img_req   <= 1'b1;
                    img_first <= q_first;
                    img_count <= q_count;
                    q_req     <= 1'b0;
                    q_on_done <= 1'b0;
                    p_done    = img_n;
                end
            end
        end else if (img_ready && !img_done) begin
            c_busy <= 1'b1;
            c_t    <= 8'd0;
        end
    end

    task img_queue(input [15:0] first, input [15:0] count, input on_done);
        begin
            @(negedge clk100);
            q_first   = first;
            q_count   = count;
            q_on_done = on_done;
            q_req     = 1'b1;
            if (!on_done) @(posedge clk100);
        end
    endtask

    initial begin : watchdog
        integer m;
        #((FIELDS * 17 + 50) * 1_000_000.0);
        $write("img slots:");
        for (m = 0; m < img_n; m = m + 1) $write(" %0d", img_log[m]);
        $display("\nFAIL: timeout");
        $finish;
    end

    task send_line(input integer base, input [7:0] r);
        integer i;
        begin
            @(posedge clk27) begin st_sav <= 1; st_row <= r; end
            @(posedge clk27) st_sav <= 0;
            for (i = 0; i < LINE; i = i + 1) begin
                @(posedge clk27) begin y_en <= 1; y <= stim[base + i]; end
                @(posedge clk27) y_en <= 0;
            end
            @(posedge clk27) st_eav <= 1;
            @(posedge clk27) st_eav <= 0;
            repeat (274) @(posedge clk27);
        end
    endtask

    integer f, r, i, k, bad, words, p1, p2, img_fail;
    reg [15:0] wv, want;
    initial begin
        repeat (20) @(posedge clk27);
        rst27 = 0;
        rst100 = 0;
        wait (ready);
        $display("sdram ready, bist_ok %0d lat %0d", bist_ok, lat);
        for (f = 0; f < FIELDS; f = f + 1) begin
            for (r = 0; r < 240; r = r + 1) begin
                // forged: a field's first line, one after the sequence is set, one after a good first
                // line, and a parity line under the wrong group
                inj = 1'b1;
                if (f == 0 && r == 1) inj_idx = N * 2 / 3;
                else if (f == 0 && r == 100) inj_idx = N * 3 / 4;
                else if (f == 1 && r == 2) inj_idx = N / 2;
                else if (f == 2 && r == 6) inj_idx = N * 9 / 10;
                else if (f == 2 && r == ROWS_R - GROUPS + 2 && GROUPS >= 4) inj_idx = 16'hFF00 + GROUPS - 1;
                else inj = 1'b0;
                send_line((f * 240 + r) * LINE, r[7:0]);
            end
            inj = 1'b0;
            repeat (20 * 1716) @(posedge clk27);
            $display("field %0d: session %0d have %0d/%0d ok %0d bad %0d none %0d recovered %0d dupes %0d misseq %0d drops %0d",
                     f, have_session, have, n, lines_ok, lines_bad, lines_none, recovered, dupes, misseq, drops);
        end
        for (i = 0; i < 2000 && !verified; i = i + 1) #10_000;
        $display("session %02x (want %02x) cls %0d rows %0d groups %0d n %0d d %0d crc_exp %08x (want %08x) lost %0d",
                 session, SESSION, s_cls, rows, groups, n, d, crc_exp, CRC, lost);
        $display("complete %0d after %0d data fields; verified %0d ok %0d crc %08x",
                 complete, load_fields, verified, verify_ok, crc_got);
        bad = 0;
        for (k = 0; k < N; k = k + 1)
            for (i = 0; i < (D + 1) / 2; i = i + 1) begin
                wv   = mem[{1'b1, k[15:6], k[5:0], i[3:0]}];
                want = {2 * i + 1 < D ? image[k * D + 2 * i + 1] : wv[15:8], image[k * D + 2 * i]};
                if (wv !== want) begin
                    if (bad < 8) $display("slot %0d word %0d: %04x want %04x", k, i, wv, want);
                    bad = bad + 1;
                end
            end
        $display("sdram slots: %0d words differ", bad);
        if (!complete)
            for (k = 0; k < N; k = k + 1)
                if (!loader.bm[k >> 5][k & 31]) $display("  missing packet %0d", k);
        peek_idx = N - 1;
        wait (peek_at == N - 1);
        repeat (2) @(posedge clk100);
        $display("peek %0d: %h", peek_at, peek);

        // img_*: a request mid-send, one while a slot is being read, one in the same cycle as img_done
        img_queue(0, 20, 0);
        while (!(img_n >= 3 && c_busy && c_t < 8'd20 && verify.state == 3'd6)) @(negedge clk100);
        p1 = img_n;
        img_queue(N / 6, 6, 0);
        while (!(img_n >= p1 + 3 && verify.state == 3'd2 && verify.imaging)) @(negedge clk100);
        p2 = img_n;
        img_queue(N / 3, 4, 0);
        wait (img_n >= p2 + 2);
        img_queue(N / 2, 5, 1);
        wait (p_done >= 0 && img_n == p_done + 5);
        repeat (2000) @(posedge clk100);
        img_fail = img_bad != 0 || img_n != p_done + 5 || verify.img_left != 0 ||
                   img_log[p1 + 1] != N / 6 || img_log[p2] != N / 3;
        for (k = 0; k < 5; k = k + 1) if (img_log[p_done + k] != N / 2 + k) img_fail = 1;
        $write("img slots:");
        for (k = 0; k < img_n; k = k + 1) $write(" %0d", img_log[k]);
        $display("; %0d bad bytes, %0s", img_bad, img_fail ? "FAIL" : "ok");

        if (complete && verified && verify_ok && crc_got == CRC && bad == 0 && session == SESSION && lost == 0 &&
            drops == 0 && !img_fail)
            $display("PASS");
        else
            $display("FAIL");
        $finish;
    end
endmodule
