// SDRAM (100 MHz): stored lines in from a two-line buffer at 27 MHz, display rows out to a four-line buffer at vclk,
// and the data tape's packet port.
module frame_store (
    input  wire        clk27,           // TD_CLK27
    input  wire        vclk,
    input  wire        sd_clk,
    input  wire        rst_sd,
    input  wire        wl_we,           // clk27: field_writer's line buffer and row requests
    input  wire [10:0] wl_waddr,
    input  wire [15:0] wl_wdata,
    input  wire        wr_req,
    input  wire [12:0] wr_row,
    input  wire        wr_half,
    input  wire        fetch,           // vclk: vga_out's row fetches and line buffer reads
    input  wire [1:0]  fetch_slot,
    input  wire [12:0] fetch_row,
    input  wire [11:0] rl_raddr,
    output wire [15:0] rl_rdata,
    output wire [7:0]  rl_rdata_p,
    input  wire        pk_wr_req,       // sd_clk: datatape_top's packet port
    input  wire [1:0]  pk_wr_entry,
    input  wire [14:0] pk_wr_idx,
    output wire [5:0]  pk_addr,
    input  wire [15:0] pk_data,
    output wire        pk_wr_done,
    input  wire        pk_rd_req,
    input  wire [14:0] pk_rd_idx,
    output wire        pk_rv,
    output wire [3:0]  pk_rcol,
    output wire [15:0] pk_rdata,
    output wire        bist_ok,
    output wire [2:0]  lat,
    output wire [49:0] bist_counts,
    output wire [12:0] sd_addr,
    output wire [1:0]  sd_ba,
    output wire        sd_cs_n,
    output wire        sd_ras_n,
    output wire        sd_cas_n,
    output wire        sd_we_n,
    output wire        sd_dqm,
    inout  wire [15:0] sd_dq
);
    wire        sd_wr_req, sd_wr_half, sd_rd_req;
    wire [1:0]  sd_rd_slot;
    wire [12:0] sd_wr_row, sd_rd_row;
    req_cdc wr_cdc (
        .sclk(clk27), .sreq(wr_req), .sdata({wr_half, wr_row}),
        .dclk(sd_clk), .dreq(sd_wr_req), .ddata({sd_wr_half, sd_wr_row})
    );
    req_cdc #(.W(15)) rd_cdc (
        .sclk(vclk), .sreq(fetch), .sdata({fetch_slot, fetch_row}),
        .dclk(sd_clk), .dreq(sd_rd_req), .ddata({sd_rd_slot, sd_rd_row})
    );

    wire [10:0] wl_raddr;
    wire [11:0] rl_waddr;
    wire [15:0] wl_rdata, rl_wdata, rl_rdata_pw;
    wire        rl_we;
    dc_ram write_lines (
        .wclk(clk27), .we(wl_we), .waddr(wl_waddr), .wdata(wl_wdata),
        .rclk(sd_clk), .raddr(wl_raddr), .rdata(wl_rdata)
    );
    dc_ram #(.AW(12)) read_lines (
        .wclk(sd_clk), .we(rl_we), .waddr(rl_waddr), .wdata(rl_wdata),
        .rclk(vclk), .raddr(rl_raddr), .rdata(rl_rdata)
    );
    dc_ram #(.AW(12)) read_lines_p (    // second read port: the other pixel of each Cb/Cr pair
        .wclk(sd_clk), .we(rl_we), .waddr(rl_waddr), .wdata(rl_wdata),
        .rclk(vclk), .raddr(rl_raddr ^ 12'd1), .rdata(rl_rdata_pw)
    );
    assign rl_rdata_p = rl_rdata_pw[7:0];

    sdram_ctrl sdram (
        .clk(sd_clk), .rst(rst_sd),
        .wr_req(sd_wr_req), .wr_row(sd_wr_row), .wr_half(sd_wr_half),
        .wl_addr(wl_raddr), .wl_data(wl_rdata),
        .rd_req(sd_rd_req), .rd_row(sd_rd_row), .rd_slot(sd_rd_slot),
        .rl_we(rl_we), .rl_addr(rl_waddr), .rl_data(rl_wdata),
        .pk_wr_req(pk_wr_req), .pk_wr_entry(pk_wr_entry), .pk_wr_idx(pk_wr_idx),
        .pk_addr(pk_addr), .pk_data(pk_data),
        .pk_rd_req(pk_rd_req), .pk_rd_idx(pk_rd_idx), .pk_rv(pk_rv), .pk_rcol(pk_rcol), .pk_rdata(pk_rdata),
        .pk_wr_done(pk_wr_done),
        .ready(), .bist_ok(bist_ok), .lat(lat), .bist_counts(bist_counts),
        .sd_addr(sd_addr), .sd_ba(sd_ba), .sd_cs_n(sd_cs_n), .sd_ras_n(sd_ras_n),
        .sd_cas_n(sd_cas_n), .sd_we_n(sd_we_n), .sd_dqm(sd_dqm), .sd_dq(sd_dq)
    );
endmodule
