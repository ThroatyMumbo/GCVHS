// Game CPU link: DN front end at 250 MHz, parser, register file and UP packets on
// sd_clk, CPU audio handed to the codec's 50 MHz domain.
module cpu_link #(
    parameter integer SB = 64
) (
    input  wire          lk_clk,
    input  wire          clk,               // sd_clk
    input  wire          rst,               // sd_clk domain
    input  wire          aclk,              // CLOCK_50: audio side
    input  wire          arst,
    input  wire [8:0]    dn_pins,           // {data[7:0], strobe}
    output wire          up_strobe,
    output wire [3:0]    up_data,
    output wire          rdy,

    output wire          own,
    output wire [8:0]    r_video,
    output wire [7:0]    r_audio,
    output wire [9:0]    r_fx,
    output wire          r_comb,
    output wire [25:0]   r_hud,
    output wire          r_ov_en,
    output wire          r_loader_rst,
    output wire [18:0]   r_win_pos,
    output wire [18:0]   r_win_size,
    output wire [19:0]   r_win_hstep,
    output wire [19:0]   r_win_vstep,
    output wire [24:0]   r_win_pan,
    output wire [19:0]   r_ov_hwin,
    output wire [24:0]   r_ov_hstep,
    output wire [47:0]   ir_payload,
    output wire          ir_send,
    output wire          reg_wr,

    output wire          fb_we,
    output wire [16:0]   fb_addr,
    output wire [7:0]    fb_data,
    output wire          pal_we,
    output wire [7:0]    pal_addr,
    output wire [24:0]   pal_data,
    output wire          flip,

    output wire          img_req,
    output wire [15:0]   img_first,
    output wire [15:0]   img_count,
    input  wire          img_ready,
    input  wire [14:0]   img_idx,
    input  wire [4:0]    img_d,
    output wire [4:0]    img_sel,
    input  wire [7:0]    img_byte,
    output wire          img_done,

    input  wire          status_req,
    input  wire [8*SB-1:0] status,
    output wire          r_tap,
    input  wire          tap_req,           // pulse: send tap_line (TAPE_AUD)
    input  wire [175:0]  tap_line,
    output wire [15:0]   tap_drops,

    input  wire          a_re,              // aclk: pop one stereo frame
    output wire          a_empty,
    output wire [31:0]   a_data,            // {right, left}

    output wire [15:0]   dn_packets,
    output wire [15:0]   dn_crc_errs,
    output wire [15:0]   dn_len_errs,
    output wire [15:0]   up_packets
);
    wire       rx_v;
    wire [7:0] rx_d;
    cpu_rx rx (.clk(lk_clk), .pins(dn_pins), .valid(rx_v), .data(rx_d));

    reg  [2:0] lk_rst;
    always @(posedge lk_clk) lk_rst <= {lk_rst[1:0], rst};
    wire       dn_empty, dn_re;
    wire [7:0] dn_d;
    wire [9:0] dn_wfill;
    async_fifo #(.AW(9), .DW(8)) dn_fifo (
        .wclk(lk_clk), .wrst(lk_rst[2]), .we(rx_v), .wdata(rx_d), .full(), .wfill(dn_wfill),
        .rclk(clk), .rrst(rst), .re(dn_re), .rdata(dn_d), .empty(dn_empty), .rfill()
    );
    assign rdy = dn_wfill < 10'd256;

    wire        aud_we, aud_full;
    wire [31:0] aud_data;
    wire        echo_we, echo_done;
    wire [7:0]  echo_addr, echo_data;
    wire [8:0]  echo_len;
    cpu_cmd cmd (
        .clk(clk), .rst(rst), .in_data(dn_d), .in_empty(dn_empty), .in_re(dn_re),
        .own(own), .r_video(r_video), .r_audio(r_audio), .r_fx(r_fx), .r_comb(r_comb), .r_hud(r_hud),
        .r_ov_en(r_ov_en), .r_loader_rst(r_loader_rst), .r_win_pos(r_win_pos), .r_win_size(r_win_size),
        .r_win_hstep(r_win_hstep), .r_win_vstep(r_win_vstep), .r_win_pan(r_win_pan), .r_ov_hwin(r_ov_hwin),
        .r_ov_hstep(r_ov_hstep), .r_tap(r_tap), .ir_payload(ir_payload), .ir_send(ir_send),
        .reg_wr(reg_wr),
        .fb_we(fb_we), .fb_addr(fb_addr), .fb_data(fb_data),
        .pal_we(pal_we), .pal_addr(pal_addr), .pal_data(pal_data),
        .aud_we(aud_we), .aud_data(aud_data), .aud_full(aud_full), .flip(flip),
        .img_req(img_req), .img_first(img_first), .img_count(img_count),
        .echo_we(echo_we), .echo_addr(echo_addr), .echo_data(echo_data), .echo_done(echo_done),
        .echo_len(echo_len),
        .packets(dn_packets), .crc_errs(dn_crc_errs), .len_errs(dn_len_errs)
    );

    // CPU audio: 2048 frames. Credits: grant 512 frames whenever what is queued plus what is owed is under
    // 1024, at most every 100 us; forget unmet credit after 100 ms without audio, so an idle CPU can start later.
    // AUD_REQ carries the running total granted, repeated every 1 ms while any is unmet, so a lost one costs nothing.
    reg  [2:0]  a_rst;
    always @(posedge aclk) a_rst <= {a_rst[1:0], arst};
    wire [11:0] a_wfill;
    async_fifo #(.AW(11), .DW(32)) aud_fifo (
        .wclk(clk), .wrst(rst), .we(aud_we), .wdata(aud_data), .full(aud_full), .wfill(a_wfill),
        .rclk(aclk), .rrst(a_rst[2]), .re(a_re), .rdata(a_data), .empty(a_empty), .rfill()
    );
    reg  [12:0] owed;
    reg  [13:0] gap;
    reg  [16:0] since;
    reg  [23:0] quiet;
    reg  [15:0] granted;
    reg         aud_req;
    always @(posedge clk) begin
        aud_req <= 1'b0;
        gap     <= gap + (gap != 14'h3FFF);
        since   <= since + (since != 17'h1FFFF);
        quiet   <= aud_we ? 24'd0 : quiet + 1'b1;
        if (rst) begin
            owed    <= 13'd0;
            gap     <= 14'd0;
            since   <= 17'd0;
            quiet   <= 24'd0;
            granted <= 16'd0;
        end else if (quiet == 24'd10_000_000) begin
            owed  <= 13'd0;
            quiet <= 24'd0;
        end else if (gap >= 14'd10000 && {1'b0, a_wfill} + owed < 13'd1024) begin
            aud_req <= 1'b1;
            gap     <= 14'd0;
            since   <= 17'd0;
            granted <= granted + 16'd512;
            owed    <= owed + 13'd512 - aud_we;
        end else begin
            if (owed != 13'd0 && since >= 17'd100_000) begin
                aud_req <= 1'b1;
                since   <= 17'd0;
            end
            if (aud_we && owed != 13'd0) owed <= owed - 1'b1;
        end
    end

    cpu_up #(.SB(SB)) up (
        .clk(clk), .rst(rst), .status_req(status_req), .status(status),
        .aud_req(aud_req), .aud_frames(granted),
        .echo_we(echo_we), .echo_waddr(echo_addr), .echo_wdata(echo_data), .echo_done(echo_done),
        .echo_len(echo_len),
        .img_ready(img_ready), .img_idx(img_idx), .img_d(img_d), .img_sel(img_sel), .img_byte(img_byte),
        .img_done(img_done),
        .tap_req(tap_req && r_tap), .tap_line(tap_line), .tap_drops(tap_drops),
        .strobe(up_strobe), .nib(up_data), .packets(up_packets)
    );
endmodule
