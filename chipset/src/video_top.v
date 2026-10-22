// Composite in (ADV7180) to 720x480p VGA: fields filed in SDRAM slots by stream tag, one stream shown. SW: vga_out fx/HUD.
// KEY0 reset, KEY1 stream, KEY2 raw, KEY3 combs. The blocks are in src/top/.
module video_top (
    input  wire       CLOCK_50,
    input  wire [3:0] KEY,
    input  wire [9:0] SW,
    output wire [9:0] LEDR,
    output wire [6:0] HEX0,
    output wire [6:0] HEX1,
    output wire [6:0] HEX2,
    output wire [6:0] HEX3,
    output wire [6:0] HEX4,
    output wire [6:0] HEX5,
    input  wire       TD_CLK27,
    input  wire [7:0] TD_DATA,
    output wire       TD_RESET_N,
    output wire       FPGA_I2C_SCLK,
    inout  wire       FPGA_I2C_SDAT,
    output wire [7:0] VGA_R,
    output wire [7:0] VGA_G,
    output wire [7:0] VGA_B,
    output wire       VGA_CLK,
    output wire       VGA_BLANK_N,
    output wire       VGA_SYNC_N,
    output wire       VGA_HS,
    output wire       VGA_VS,
    output wire [12:0] DRAM_ADDR,
    output wire [1:0]  DRAM_BA,
    output wire        DRAM_CAS_N,
    output wire        DRAM_CKE,
    output wire        DRAM_CLK,
    output wire        DRAM_CS_N,
    inout  wire [15:0] DRAM_DQ,
    output wire        DRAM_LDQM,
    output wire        DRAM_RAS_N,
    output wire        DRAM_UDQM,
    output wire        DRAM_WE_N,
    output wire        AUD_XCK,
    output wire        AUD_BCLK,
    output wire        AUD_DACLRCK,
    output wire        AUD_ADCLRCK,
    output wire        AUD_DACDAT,
    input  wire        AUD_ADCDAT,
    input  wire        IRDA_RXD,
    output wire        IRDA_TXD,
    inout  wire [35:0] GPIO_0,
    input  wire [35:0] GPIO_1
);
    wire        vclk, sd_clk, lk_clk, rst50, rst27, rst_sd, ms_tick;
    wire [7:0]  key_resets;
    clocks_resets clocks (
        .clk50(CLOCK_50), .clk27(TD_CLK27), .key0_n(KEY[0]),
        .vclk(vclk), .sd_clk(sd_clk), .dram_clk(DRAM_CLK), .lk_clk(lk_clk),
        .rst50(rst50), .rst27(rst27), .rst_sd(rst_sd), .key_resets(key_resets), .ms_tick(ms_tick)
    );

    wire        cfg_done, i2c_err, codec_err, in_blue, codes_seen, parity_ok;
    wire [7:0]  status1, fields_per_sec;
    wire [9:0]  lines_f0, lines_f1, active_f0, active_f1;
    wire [11:0] active_bytes, line_min, line_max;
    wire [15:0] parity_errs, bad_codes;
    wire        wl_we, wr_req, wr_half;
    wire [10:0] wl_waddr;
    wire [15:0] wl_wdata;
    wire [12:0] wr_row;
    wire        line_start, frame_start, field_start, lock_start;
    wire        tag_strobe, tag_seen, tag_hit, frozen;
    wire [3:0]  tag_scheme;
    wire [7:0]  tag_count, tag_hit_count;
    wire [15:0] tag_bad;
    wire [8:0]  tag_hit_line;
    wire        vitc_hit;
    wire [8:0]  vitc_hit_line;
    wire [63:0] vitc_data;
    wire        ptag_hit;
    wire [15:0] ptag_pos;
    wire [6:0]  ptag_at;
    wire [8:0]  ptag_line;
    wire        vitc_line_done, vitc_like, tap_sav, tap_y_en;
    wire [2:0]  vitc_why;
    wire [3:0]  vitc_why_grp;
    wire [7:0]  tap_y;
    wire [8:0]  tap_line;
    wire [1:0]  cur_slot;
    wire        st_sav, st_y_en, st_eav;
    wire [7:0]  st_y, st_row;
    wire        raw, comb_off;
    video_in vin (
        .clk50(CLOCK_50), .rst50(rst50), .clk27(TD_CLK27), .rst27(rst27), .td_data(TD_DATA),
        .td_reset_n(TD_RESET_N), .i2c_scl(FPGA_I2C_SCLK), .i2c_sda(FPGA_I2C_SDAT),
        .comb_off(comb_off), .raw(raw),
        .cfg_done(cfg_done), .i2c_err(i2c_err), .codec_err(codec_err), .status1(status1),
        .fields_per_sec(fields_per_sec),
        .lines_f0(lines_f0), .lines_f1(lines_f1), .active_f0(active_f0), .active_f1(active_f1),
        .active_bytes(active_bytes), .line_min(line_min), .line_max(line_max),
        .parity_errs(parity_errs), .bad_codes(bad_codes), .codes_seen(codes_seen), .parity_ok(parity_ok),
        .in_blue(in_blue),
        .wl_we(wl_we), .wl_waddr(wl_waddr), .wl_wdata(wl_wdata),
        .wr_req(wr_req), .wr_row(wr_row), .wr_half(wr_half),
        .line_start(line_start), .frame_start(frame_start), .field_start(field_start), .lock_start(lock_start),
        .tag_strobe(tag_strobe), .tag_scheme(tag_scheme), .tag_count(tag_count), .tag_seen(tag_seen), .tag_bad(tag_bad),
        .tag_hit(tag_hit), .tag_hit_line(tag_hit_line), .tag_hit_count(tag_hit_count),
        .vitc_hit(vitc_hit), .vitc_hit_line(vitc_hit_line), .vitc_data(vitc_data),
        .ptag_hit(ptag_hit), .ptag_pos(ptag_pos), .ptag_at(ptag_at), .ptag_line(ptag_line),
        .vitc_line_done(vitc_line_done), .vitc_why(vitc_why), .vitc_why_grp(vitc_why_grp), .vitc_like(vitc_like),
        .tap_sav(tap_sav), .tap_y_en(tap_y_en), .tap_y(tap_y), .tap_line(tap_line),
        .frozen(frozen), .cur_slot(cur_slot),
        .st_sav(st_sav), .st_y_en(st_y_en), .st_y(st_y), .st_eav(st_eav), .st_row(st_row)
    );

    // Game CPU registers, carried into 27 and 50 MHz on each write; once own is set they replace the switches and keys.
    wire        own27, r_comb27, r_ov_en27, r_loader_rst27;
    wire [8:0]  r_video27;
    wire [5:0]  r_audio27;
    wire [9:0]  r_fx27;
    wire [25:0] r_hud27;
    wire [18:0] r_win_pos27, r_win_size27;
    wire [19:0] r_win_hstep27, r_win_vstep27;
    wire [24:0] r_win_pan27;
    wire [19:0] r_ov_hwin27;
    wire [24:0] r_ov_hstep27;
    wire        own50, cpu_ir_send50;
    wire [1:0]  r_aud_off50;
    wire [47:0] cpu_ir_payload50;

    wire [15:0] jtag_src;               // VSTA source
    wire        mosaic, aud_expand, aud_cells, aud_fit, aud_seen;
    wire [9:0]  fx;
    wire [3:0]  stream;
    wire [7:0]  view;
    wire [1:0]  r_log2;
    wire [2:0]  aud_nl, aud_sel;
    stream_select sel (
        .clk(TD_CLK27), .sw(SW), .key_n(KEY[3:1]), .jtag_src(jtag_src),
        .own(own27), .r_video(r_video27), .r_audio(r_audio27), .r_fx(r_fx27), .r_comb(r_comb27),
        .frame_start(frame_start), .tag_seen(tag_seen), .frozen(frozen), .tag_scheme(tag_scheme), .cur_slot(cur_slot),
        .aud_seen(aud_seen),
        .raw(raw), .mosaic(mosaic), .comb_off(comb_off), .fx(fx), .stream(stream), .view(view), .r_log2(r_log2),
        .aud_nl(aud_nl), .aud_sel(aud_sel), .aud_expand(aud_expand), .aud_cells(aud_cells), .aud_fit(aud_fit)
    );

    wire        cpu_a_re, cpu_a_empty;
    wire [31:0] cpu_a_data;
    wire [15:0] cpu_a_under, aud_rate, aud_no_pilot, aud_under, aud_over, aud_peak, r_in;
    wire [10:0] aud_cell0;
    wire [11:0] aud_pilot;
    wire        atap_v, r_valid, dt_active;
    wire [87:0] atap_cells;
    wire [8:0]  aud_fill;
    audio_top audio (
        .clk27(TD_CLK27), .rst27(rst27), .clk50(CLOCK_50), .rst50(rst50),
        .st_sav(st_sav), .st_y_en(st_y_en), .st_y(st_y), .st_eav(st_eav), .field_start(field_start),
        .frozen(frozen), .n_log2(aud_nl[1:0]), .sel(aud_sel), .expand(aud_expand),
        .own50(own50), .cpu_off(r_aud_off50), .jtag_off(jtag_src[15:14]), .dt_active(dt_active),
        .cpu_re(cpu_a_re), .cpu_empty(cpu_a_empty), .cpu_data(cpu_a_data), .cpu_under(cpu_a_under),
        .xck(AUD_XCK), .bclk(AUD_BCLK), .lrck(AUD_DACLRCK), .dacdat(AUD_DACDAT), .adcdat(AUD_ADCDAT),
        .aud_seen(aud_seen), .rate(aud_rate), .no_pilot(aud_no_pilot), .cell0(aud_cell0), .pilot(aud_pilot),
        .line_v(atap_v), .cells(atap_cells),
        .fill(aud_fill), .under(aud_under), .over(aud_over), .peak(aud_peak), .r_valid(r_valid), .r_in(r_in)
    );
    assign AUD_ADCLRCK = AUD_DACLRCK;

    wire        pk_wr_req, pk_wr_done, pk_rd_req, pk_rv;
    wire [1:0]  pk_wr_entry;
    wire [14:0] pk_wr_idx, pk_rd_idx;
    wire [5:0]  pk_addr;
    wire [15:0] pk_data, pk_rdata;
    wire [3:0]  pk_rcol;
    wire        img_req, img_ready, img_done;
    wire [15:0] img_first, img_count;
    wire [14:0] img_idx;
    wire [4:0]  img_sel;
    wire [7:0]  img_byte;
    wire [3:0]  slot_data;
    wire        dt_have_session, dt_complete, dt_verified, dt_verify_ok;
    wire [7:0]  dt_session;
    wire [4:0]  dt_d;
    wire [15:0] dt_n, dt_have;
    wire [31:0] dt_crc_exp;
    datatape_top datatape (
        .clk27(TD_CLK27), .rst27(rst27), .sd_clk(sd_clk), .rst_sd(rst_sd),
        .st_sav(st_sav), .st_row(st_row), .st_y_en(st_y_en), .st_y(st_y), .st_eav(st_eav),
        .field_start(field_start), .wr_req(wr_req), .wr_row(wr_row), .loader_rst(r_loader_rst27),
        .pk_wr_req(pk_wr_req), .pk_wr_entry(pk_wr_entry), .pk_wr_idx(pk_wr_idx), .pk_addr(pk_addr), .pk_data(pk_data),
        .pk_wr_done(pk_wr_done), .pk_rd_req(pk_rd_req), .pk_rd_idx(pk_rd_idx),
        .pk_rv(pk_rv), .pk_rcol(pk_rcol), .pk_rdata(pk_rdata),
        .img_req(img_req), .img_first(img_first), .img_count(img_count), .img_ready(img_ready), .img_idx(img_idx),
        .img_sel(img_sel), .img_byte(img_byte), .img_done(img_done),
        .slot_data(slot_data), .data_active(dt_active), .have_session(dt_have_session), .complete(dt_complete),
        .session(dt_session), .d(dt_d), .n(dt_n), .have(dt_have), .crc_exp(dt_crc_exp),
        .verified(dt_verified), .verify_ok(dt_verify_ok)
    );

    wire        fetch;
    wire [1:0]  fetch_slot;
    wire [12:0] fetch_row;
    wire [11:0] rl_raddr;
    wire [15:0] rl_rdata;
    wire [7:0]  rl_rdata_p;
    wire        bist_ok, sd_dqm;
    wire [2:0]  sd_lat;
    wire [49:0] bist_counts;
    frame_store store (
        .clk27(TD_CLK27), .vclk(vclk), .sd_clk(sd_clk), .rst_sd(rst_sd),
        .wl_we(wl_we), .wl_waddr(wl_waddr), .wl_wdata(wl_wdata), .wr_req(wr_req), .wr_row(wr_row), .wr_half(wr_half),
        .fetch(fetch), .fetch_slot(fetch_slot), .fetch_row(fetch_row),
        .rl_raddr(rl_raddr), .rl_rdata(rl_rdata), .rl_rdata_p(rl_rdata_p),
        .pk_wr_req(pk_wr_req), .pk_wr_entry(pk_wr_entry), .pk_wr_idx(pk_wr_idx), .pk_addr(pk_addr), .pk_data(pk_data),
        .pk_wr_done(pk_wr_done), .pk_rd_req(pk_rd_req), .pk_rd_idx(pk_rd_idx),
        .pk_rv(pk_rv), .pk_rcol(pk_rcol), .pk_rdata(pk_rdata),
        .bist_ok(bist_ok), .lat(sd_lat), .bist_counts(bist_counts),
        .sd_addr(DRAM_ADDR), .sd_ba(DRAM_BA), .sd_cs_n(DRAM_CS_N), .sd_ras_n(DRAM_RAS_N),
        .sd_cas_n(DRAM_CAS_N), .sd_we_n(DRAM_WE_N), .sd_dqm(sd_dqm), .sd_dq(DRAM_DQ)
    );
    assign DRAM_CKE  = 1'b1;
    assign DRAM_LDQM = sd_dqm;
    assign DRAM_UDQM = sd_dqm;

    wire        ltc_tick, ltc_locked;
    wire [23:0] ltc_field, vitc_pos;
    wire [31:0] ltc_tc, ltc_user, vitc_tc;
    wire [19:0] hud_tape;
    wire [5:0]  vitc_age, pt_age;
    wire [15:0] pt_agreed;
    tape_position tape (
        .clk50(CLOCK_50), .rst50(rst50), .clk27(TD_CLK27), .r_valid(r_valid), .r_in(r_in),
        .frame_start(frame_start), .tag_strobe(tag_strobe), .tag_count(tag_count),
        .vitc_hit(vitc_hit), .vitc_hit_line(vitc_hit_line), .vitc_data(vitc_data),
        .tap_sav(tap_sav), .tap_line(tap_line),
        .ptag_hit(ptag_hit), .ptag_pos(ptag_pos), .ptag_at(ptag_at), .ptag_line(ptag_line),
        .ltc_tick(ltc_tick), .ltc_locked(ltc_locked), .ltc_field(ltc_field), .ltc_tc(ltc_tc), .ltc_user(ltc_user),
        .hud_tape(hud_tape), .vitc_pos(vitc_pos), .vitc_tc(vitc_tc), .vitc_age(vitc_age),
        .pt_age(pt_age), .pt_agreed(pt_agreed)
    );

    wire        ov_we, pal_we, ov_back, c_flip, vblank, top, front;
    wire [16:0] ov_waddr;
    wire [7:0]  ov_wdata, pal_addr, flips;
    wire [24:0] pal_data;
    wire [31:0] frames;
    video_out vout (
        .clk27(TD_CLK27), .vclk(vclk), .sd_clk(sd_clk), .line_start(line_start), .lock_start(lock_start),
        .view(view), .fx(fx), .raw(raw), .slot_data(slot_data),
        .aud_cells(aud_cells), .aud_fit(aud_fit), .aud_nl(aud_nl[1:0]), .hud_tape(hud_tape),
        .own(own27), .r_hud(r_hud27), .r_ov_en(r_ov_en27), .r_win_pos(r_win_pos27), .r_win_size(r_win_size27),
        .r_win_hstep(r_win_hstep27), .r_win_vstep(r_win_vstep27), .r_win_pan(r_win_pan27),
        .r_ov_hwin(r_ov_hwin27), .r_ov_hstep(r_ov_hstep27),
        .ov_we(ov_we), .ov_waddr(ov_waddr), .ov_wdata(ov_wdata), .ov_back(ov_back),
        .pal_we(pal_we), .pal_addr(pal_addr), .pal_data(pal_data), .flip(c_flip),
        .fetch(fetch), .fetch_slot(fetch_slot), .fetch_row(fetch_row),
        .rl_raddr(rl_raddr), .rl_rdata(rl_rdata), .rl_rdata_p(rl_rdata_p),
        .r(VGA_R), .g(VGA_G), .b(VGA_B), .hs_n(VGA_HS), .vs_n(VGA_VS), .blank_n(VGA_BLANK_N),
        .vblank(vblank), .top(top), .frames(frames), .flips(flips), .front(front)
    );
    assign VGA_CLK    = ~vclk;          // ADV7123 latches on the rising edge, mid-way through our cycle
    assign VGA_SYNC_N = 1'b0;           // no sync on green

    wire [15:0] ir_seq;
    ir_top ir (
        .clk(CLOCK_50), .rst(rst50), .rx(IRDA_RXD), .tx(IRDA_TXD),
        .cpu_send(cpu_ir_send50), .cpu_payload(cpu_ir_payload50), .rx_seq(ir_seq)
    );

    wire        up_strobe, lk_rdy;
    wire [3:0]  up_data;
    cpu_bridge cpu (
        .lk_clk(lk_clk), .sd_clk(sd_clk), .rst_sd(rst_sd), .clk50(CLOCK_50), .rst50(rst50), .clk27(TD_CLK27),
        .vclk(vclk), .dn_pins(GPIO_0[8:0]), .up_strobe(up_strobe), .up_data(up_data), .rdy(lk_rdy),
        .own27(own27), .r_video27(r_video27), .r_audio27(r_audio27), .r_fx27(r_fx27), .r_comb27(r_comb27),
        .r_hud27(r_hud27), .r_ov_en27(r_ov_en27), .r_loader_rst27(r_loader_rst27),
        .r_win_pos27(r_win_pos27), .r_win_size27(r_win_size27), .r_win_hstep27(r_win_hstep27),
        .r_win_vstep27(r_win_vstep27), .r_win_pan27(r_win_pan27), .r_ov_hwin27(r_ov_hwin27), .r_ov_hstep27(r_ov_hstep27),
        .own50(own50), .r_aud_off50(r_aud_off50), .ir_send50(cpu_ir_send50), .ir_payload50(cpu_ir_payload50),
        .ov_we(ov_we), .ov_waddr(ov_waddr), .ov_wdata(ov_wdata), .pal_we(pal_we), .pal_addr(pal_addr),
        .pal_data(pal_data), .ov_back(ov_back), .flip(c_flip),
        .img_req(img_req), .img_first(img_first), .img_count(img_count), .img_ready(img_ready), .img_idx(img_idx),
        .img_d(dt_d), .img_sel(img_sel), .img_byte(img_byte), .img_done(img_done),
        .a_re(cpu_a_re), .a_empty(cpu_a_empty), .a_data(cpu_a_data),
        .atap_v(atap_v), .atap_cells(atap_cells), .st_sav(st_sav), .st_row(st_row), .field_start(field_start),
        .top(top), .frames(frames), .flips(flips), .front(front),
        .tag_seen(tag_seen), .frozen(frozen), .raw(raw), .aud_seen(aud_seen), .mosaic(mosaic), .comb_off(comb_off),
        .tag_scheme(tag_scheme), .tag_count(tag_count), .stream(stream), .aud_sel(aud_sel),
        .dt_have_session(dt_have_session), .dt_complete(dt_complete), .dt_session(dt_session), .dt_d(dt_d),
        .dt_n(dt_n), .dt_have(dt_have), .dt_crc_exp(dt_crc_exp),
        .vitc_pos(vitc_pos), .vitc_tc(vitc_tc), .vitc_age(vitc_age), .pt_age(pt_age), .pt_agreed(pt_agreed),
        .aud_rate(aud_rate), .tag_bad(tag_bad), .in_blue(in_blue), .dt_active(dt_active),
        .ms_tick(ms_tick), .in_lock(status1[0]), .ltc_tick(ltc_tick), .ltc_locked(ltc_locked), .ltc_field(ltc_field),
        .ltc_tc(ltc_tc), .ltc_user(ltc_user), .ir_seq(ir_seq), .cpu_under(cpu_a_under),
        .key_resets(key_resets), .dt_verified(dt_verified), .dt_verify_ok(dt_verify_ok)
    );

    // GPIO_0[8:0]: the CPU's DN link; [15:9]: UP link, RDY, VBLANK; HDRP can drive them instead.
    wire [17:0] hdr_src;
    assign GPIO_0[8:0]  = hdr_src[17] ? hdr_src[8:0] : 9'bz;
    assign GPIO_0[15:9] = hdr_src[16] ? hdr_src[15:9] : {vblank, lk_rdy, up_data, up_strobe};
    assign GPIO_0[16]   = IRDA_TXD;     // external IR LED (header pin 19, 150R to the anode)

    // VSTA (scripts/de1_ctl.sh): the 27 MHz half is double-flopped; SDRAM bits are static after start-up.
    debug_probes debug (
        .clk50(CLOCK_50), .clk27(TD_CLK27), .lk_clk(lk_clk),
        .stats27({aud_sel, aud_rate, aud_no_pilot, aud_cell0, aud_pilot, raw, comb_off, view[7:6], stream, tag_bad,
                  tag_seen, tag_scheme, tag_count, fields_per_sec, lines_f0, lines_f1, active_f0, active_f1,
                  active_bytes, line_min, line_max, parity_errs, bad_codes, codes_seen, parity_ok, frozen, in_blue}),
        .stats50({codec_err, aud_fill, aud_under, aud_over, aud_peak,
                  bist_ok, sd_lat, bist_counts, status1, cfg_done, i2c_err}),
        .sel_src(jtag_src),
        .tag_hit(tag_hit), .tag_hit_line(tag_hit_line), .tag_hit_count(tag_hit_count),
        .vitc_line_done(vitc_line_done), .vitc_like(vitc_like), .vitc_why(vitc_why), .vitc_why_grp(vitc_why_grp),
        .tap_sav(tap_sav), .tap_y_en(tap_y_en), .tap_y(tap_y), .tap_line(tap_line),
        .hdr_pins({GPIO_1, GPIO_0}), .hdr_src(hdr_src), .link_pins(GPIO_0[7:0]),
        .vga_hs(VGA_HS), .vga_vs(VGA_VS)
    );

    front_panel panel (
        .clk27(TD_CLK27), .bist_ok(bist_ok), .tag_seen(tag_seen), .bad_codes(bad_codes), .parity_ok(parity_ok),
        .codes_seen(codes_seen), .status1(status1), .i2c_err(i2c_err), .cfg_done(cfg_done),
        .fields_per_sec(fields_per_sec), .tag_scheme(tag_scheme), .r_log2(r_log2), .raw(raw), .stream(stream),
        .ledr(LEDR), .hex0(HEX0), .hex1(HEX1), .hex2(HEX2), .hex3(HEX3), .hex4(HEX4), .hex5(HEX5)
    );
endmodule
