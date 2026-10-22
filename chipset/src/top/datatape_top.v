// Data tape (tools/datatape.py): packets sliced off the stored lines, assembled in SDRAM bank 1 and checked against
// the header's CRC-32. JTAG instance DTAP: source {reset loader, peek slot[14:0]}; probe: tools/datatape.py PROBE.
module datatape_top (
    input  wire        clk27,           // TD_CLK27
    input  wire        rst27,
    input  wire        sd_clk,
    input  wire        rst_sd,
    input  wire        st_sav,
    input  wire [7:0]  st_row,
    input  wire        st_y_en,
    input  wire [7:0]  st_y,
    input  wire        st_eav,
    input  wire        field_start,
    input  wire        wr_req,
    input  wire [12:0] wr_row,
    input  wire        loader_rst,      // clk27, from the game CPU
    output wire        pk_wr_req,       // sd_clk: sdram_ctrl's packet port
    output wire [1:0]  pk_wr_entry,
    output wire [14:0] pk_wr_idx,
    input  wire [5:0]  pk_addr,
    output wire [15:0] pk_data,
    input  wire        pk_wr_done,
    output wire        pk_rd_req,
    output wire [14:0] pk_rd_idx,
    input  wire        pk_rv,
    input  wire [3:0]  pk_rcol,
    input  wire [15:0] pk_rdata,
    input  wire        img_req,         // sd_clk: game CPU slot reads (cpu_link)
    input  wire [15:0] img_first,
    input  wire [15:0] img_count,
    output wire        img_ready,
    output wire [14:0] img_idx,
    input  wire [4:0]  img_sel,
    output wire [7:0]  img_byte,
    input  wire        img_done,
    output reg  [3:0]  slot_data = 4'hF,    // per slot: its last field carried data; nothing filed yet shows nothing
    output wire        data_active,
    output wire        have_session,
    output wire        complete,
    output wire [7:0]  session,
    output wire [4:0]  d,
    output wire [15:0] n,
    output wire [15:0] have,
    output wire [31:0] crc_exp,
    output wire        verified,        // sd_clk
    output wire        verify_ok
);
    `include "sdram_map.vh"
    wire [15:0] dtap_src;
    wire        dt_rst;
    sync2 dt_rst_sync (.clk(clk27), .d(dtap_src[15] | loader_rst), .q(dt_rst));
    wire         valid, ok, runin;
    wire [1:0]   cls;
    wire [7:0]   row;
    wire [167:0] bytes;
    data_slicer dslicer (
        .clk(clk27), .start(st_sav), .row(st_row), .y_en(st_y_en), .y(st_y), .done(st_eav),
        .valid(valid), .ok(ok), .runin(runin), .cls(cls), .line_row(row), .bytes(bytes)
    );
    wire         ring_we, pk_req, pk_ack;
    wire [5:0]   ring_waddr;
    wire [15:0]  ring_wdata;
    wire [16:0]  pk_req_data;
    wire [7:0]   rows, lost, drops;
    wire [3:0]   groups;
    wire [1:0]   s_cls;
    wire [15:0]  recovered, dupes, data_fields, load_fields, misseq;
    wire [23:0]  lines_ok, lines_bad, lines_none;
    data_loader dloader (
        .clk(clk27), .rst(rst27 | dt_rst), .field_start(field_start),
        .valid(valid), .ok(ok), .runin(runin), .cls(cls), .row(row), .bytes(bytes),
        .ring_we(ring_we), .ring_addr(ring_waddr), .ring_data(ring_wdata),
        .pk_req(pk_req), .pk_req_data(pk_req_data), .pk_ack(pk_ack),
        .have_session(have_session), .complete(complete), .session(session), .rows(rows),
        .groups(groups), .s_cls(s_cls), .n(n), .have(have), .crc_exp(crc_exp), .d(d),
        .lines_ok(lines_ok), .lines_bad(lines_bad), .lines_none(lines_none),
        .recovered(recovered), .dupes(dupes), .data_fields(data_fields),
        .load_fields(load_fields), .lost(lost), .drops(drops), .misseq(misseq), .data_active(data_active)
    );
    // Slots whose last written field carried data lines show black (except raw): a leader field is no picture.
    // A leader field has ~190 good lines, and a picture line passes the CRC now and then.
    localparam [3:0] DATA_FIELD_LINES = 4'd8;
    reg  [3:0]  fd_ok;
    reg         fd_written;
    reg  [1:0]  fd_slot;
    always @(posedge clk27) begin
        if (valid && ok && fd_ok != 4'd15) fd_ok <= fd_ok + 1'b1;
        if (wr_req) {fd_written, fd_slot} <= {1'b1, video_row_slot(wr_row)};
        if (field_start) begin
            if (fd_written) slot_data[fd_slot] <= fd_ok >= DATA_FIELD_LINES;
            fd_ok      <= 4'd0;
            fd_written <= 1'b0;
        end
    end

    req_cdc #(.W(1)) pk_ack_cdc (
        .sclk(sd_clk), .sreq(pk_wr_done), .sdata(1'b0), .dclk(clk27), .dreq(pk_ack), .ddata()
    );
    dc_ram #(.AW(6)) pk_ring (
        .wclk(clk27), .we(ring_we), .waddr(ring_waddr), .wdata(ring_wdata),
        .rclk(sd_clk), .raddr(pk_addr), .rdata(pk_data)
    );
    req_cdc #(.W(17)) pk_cdc (
        .sclk(clk27), .sreq(pk_req), .sdata(pk_req_data),
        .dclk(sd_clk), .dreq(pk_wr_req), .ddata({pk_wr_entry, pk_wr_idx})
    );

    wire [14:0]  peek_s, peek_at;
    wire [31:0]  crc_got;
    wire [135:0] peek;
    sync2 #(.W(15)) peek_sync (.clk(sd_clk), .d(dtap_src[14:0]), .q(peek_s));
    data_verify dverify (
        .clk(sd_clk), .rst(rst_sd), .complete(complete), .n(n), .d(d), .crc_exp(crc_exp),
        .peek_idx(peek_s), .rd_req(pk_rd_req), .rd_idx(pk_rd_idx),
        .rv(pk_rv), .rcol(pk_rcol), .rdata(pk_rdata),
        .verified(verified), .verify_ok(verify_ok), .crc_got(crc_got), .peek_at(peek_at), .peek(peek),
        .img_req(img_req), .img_first(img_first), .img_count(img_count), .img_ready(img_ready), .img_idx(img_idx),
        .img_sel(img_sel), .img_byte(img_byte), .img_done(img_done)
    );

    // Counters may read torn.
    jtag_probe #(.IDX(1), .ID("DTAP"), .PW(441), .SW(16)) dtap (
        .probe({peek_at, peek, session, s_cls, rows, groups, n, have, crc_exp,
                crc_got, have_session, complete, verified, verify_ok, lines_ok, lines_bad,
                lines_none, recovered, dupes, data_fields, load_fields, lost, drops, misseq}),
        .source(dtap_src)
    );
endmodule
