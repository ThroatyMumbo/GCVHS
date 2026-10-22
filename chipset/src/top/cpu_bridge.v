// Game CPU link on sd_clk: its registers carried to 27 and 50 MHz, the TAPE_AUD tap, and STATUS
// (cpu/src/link_proto.h vhs_status_t) assembled at each output frame top. JTAG instance CPUL.
module cpu_bridge (
    input  wire         lk_clk,
    input  wire         sd_clk,
    input  wire         rst_sd,
    input  wire         clk50,
    input  wire         rst50,
    input  wire         clk27,          // TD_CLK27
    input  wire         vclk,
    input  wire [8:0]   dn_pins,
    output wire         up_strobe,
    output wire [3:0]   up_data,
    output wire         rdy,
    output wire         own27,          // clk27 copies of the registers
    output wire [8:0]   r_video27,
    output wire [5:0]   r_audio27,      // {expand, show strip, own stream, stream[2:0]}
    output wire [9:0]   r_fx27,
    output wire         r_comb27,
    output wire [25:0]  r_hud27,
    output wire         r_ov_en27,
    output wire         r_loader_rst27,
    output wire [18:0]  r_win_pos27,
    output wire [18:0]  r_win_size27,
    output wire [19:0]  r_win_hstep27,
    output wire [19:0]  r_win_vstep27,
    output wire [24:0]  r_win_pan27,
    output wire [19:0]  r_ov_hwin27,
    output wire [24:0]  r_ov_hstep27,
    output wire         own50,          // clk50 copies
    output wire [1:0]   r_aud_off50,    // {tape off, line-in off}
    output wire         ir_send50,
    output wire [47:0]  ir_payload50,
    output wire         ov_we,          // sd_clk: overlay framebuffer and palette
    output wire [16:0]  ov_waddr,
    output wire [7:0]   ov_wdata,
    output wire         pal_we,
    output wire [7:0]   pal_addr,
    output wire [24:0]  pal_data,
    output wire         ov_back,
    output wire         flip,
    output wire         img_req,        // sd_clk: tape slot reads (datatape_top)
    output wire [15:0]  img_first,
    output wire [15:0]  img_count,
    input  wire         img_ready,
    input  wire [14:0]  img_idx,
    input  wire [4:0]   img_d,
    output wire [4:0]   img_sel,
    input  wire [7:0]   img_byte,
    output wire         img_done,
    input  wire         a_re,           // clk50: CPU audio to audio_out
    output wire         a_empty,
    output wire [31:0]  a_data,
    input  wire         atap_v,         // clk27: a line's audio cells, with the stored row and field start
    input  wire [87:0]  atap_cells,
    input  wire         st_sav,
    input  wire [7:0]   st_row,
    input  wire         field_start,
    input  wire         top,            // vclk: video_out's frame top and counts
    input  wire [31:0]  frames,
    input  wire [7:0]   flips,
    input  wire         front,
    input  wire         tag_seen,       // clk27 STATUS fields
    input  wire         frozen,
    input  wire         raw,
    input  wire         aud_seen,
    input  wire         mosaic,
    input  wire         comb_off,
    input  wire [3:0]   tag_scheme,
    input  wire [7:0]   tag_count,
    input  wire [3:0]   stream,
    input  wire [2:0]   aud_sel,
    input  wire         dt_have_session,
    input  wire         dt_complete,
    input  wire [7:0]   dt_session,
    input  wire [4:0]   dt_d,
    input  wire [15:0]  dt_n,
    input  wire [15:0]  dt_have,
    input  wire [31:0]  dt_crc_exp,
    input  wire [23:0]  vitc_pos,
    input  wire [31:0]  vitc_tc,
    input  wire [5:0]   vitc_age,
    input  wire [5:0]   pt_age,
    input  wire [15:0]  pt_agreed,
    input  wire [15:0]  aud_rate,
    input  wire [15:0]  tag_bad,
    input  wire         in_blue,
    input  wire         dt_active,
    input  wire         ms_tick,        // clk50 STATUS fields, sampled each ms
    input  wire         in_lock,
    input  wire         ltc_tick,
    input  wire         ltc_locked,
    input  wire [23:0]  ltc_field,
    input  wire [31:0]  ltc_tc,
    input  wire [31:0]  ltc_user,
    input  wire [15:0]  ir_seq,
    input  wire [15:0]  cpu_under,
    input  wire [7:0]   key_resets,     // sd_clk STATUS fields
    input  wire         dt_verified,
    input  wire         dt_verify_ok
);
    `include "cpu/link_proto.vh"
    localparam integer SB = LINK_STATUS_BYTES;
    wire         c_own, c_comb, c_ov_en, c_loader_rst, c_ir_send, c_reg_wr;
    wire [8:0]   c_video;
    wire [7:0]   c_audio;
    wire [9:0]   c_fx;
    wire [25:0]  c_hud;
    wire [47:0]  c_ir_payload;
    wire [18:0]  c_win_pos, c_win_size;
    wire [19:0]  c_win_hstep, c_win_vstep;
    wire [24:0]  c_win_pan;
    wire [19:0]  c_ov_hwin;
    wire [24:0]  c_ov_hstep;
    wire [15:0]  dn_packets, dn_crc_errs, dn_len_errs, up_packets;
    wire         st_req;
    wire [8*SB-1:0] st_vec;
    wire         atap_req;
    wire [175:0] atap_line;
    wire [15:0]  tap_drops;

    // Tape-audio tap (TAPE_AUD): every cell of each line with a pilot, with the LTC field.
    // frame_start (so the LTC field) comes at stored row 2: count fields from row 0, set by LTC after it.
    wire        atap_fv, atap_lk;
    wire [23:0] atap_lf;
    reg         atap_locked;
    reg  [23:0] atap_field;
    reg  [7:0]  atap_row;
    req_cdc #(.W(25)) atap_ltc_cdc (
        .sclk(clk50), .sreq(ltc_tick), .sdata({ltc_locked, ltc_field}),
        .dclk(clk27), .dreq(atap_fv), .ddata({atap_lk, atap_lf})
    );
    always @(posedge clk27) begin
        if (st_sav) atap_row <= st_row;
        if (atap_fv) {atap_locked, atap_field} <= {atap_lk, atap_lf};
        else if (field_start) atap_field <= atap_field + 1'b1;
    end
    wire [175:0] atap_rec = {5'd0, atap_cells[87:77], 5'd0, atap_cells[76:66], 5'd0, atap_cells[65:55],
                             5'd0, atap_cells[54:44], 5'd0, atap_cells[43:33], 5'd0, atap_cells[32:22],
                             5'd0, atap_cells[21:11], 5'd0, atap_cells[10:0],
                             8'd0, atap_row, 7'd0, atap_locked, atap_field};
    req_cdc #(.W(176)) atap_cdc (
        .sclk(clk27), .sreq(atap_v), .sdata(atap_rec), .dclk(sd_clk), .dreq(atap_req), .ddata(atap_line)
    );

    cpu_link #(.SB(SB)) cpu (
        .lk_clk(lk_clk), .clk(sd_clk), .rst(rst_sd), .aclk(clk50), .arst(rst50),
        .dn_pins(dn_pins), .up_strobe(up_strobe), .up_data(up_data), .rdy(rdy),
        .own(c_own), .r_video(c_video), .r_audio(c_audio), .r_fx(c_fx), .r_comb(c_comb), .r_hud(c_hud),
        .r_ov_en(c_ov_en), .r_loader_rst(c_loader_rst), .r_win_pos(c_win_pos), .r_win_size(c_win_size),
        .r_win_hstep(c_win_hstep), .r_win_vstep(c_win_vstep), .r_win_pan(c_win_pan), .r_ov_hwin(c_ov_hwin),
        .r_ov_hstep(c_ov_hstep), .ir_payload(c_ir_payload), .ir_send(c_ir_send),
        .reg_wr(c_reg_wr),
        .fb_we(ov_we), .fb_addr(ov_waddr), .fb_data(ov_wdata),
        .pal_we(pal_we), .pal_addr(pal_addr), .pal_data(pal_data), .flip(flip),
        .img_req(img_req), .img_first(img_first), .img_count(img_count), .img_ready(img_ready),
        .img_idx(img_idx), .img_d(img_d), .img_sel(img_sel), .img_byte(img_byte), .img_done(img_done),
        .status_req(st_req), .status(st_vec),
        .r_tap(), .tap_req(atap_req), .tap_line(atap_line), .tap_drops(tap_drops),
        .a_re(a_re), .a_empty(a_empty), .a_data(a_data),
        .dn_packets(dn_packets), .dn_crc_errs(dn_crc_errs), .dn_len_errs(dn_len_errs), .up_packets(up_packets)
    );

    // Registers out to 27 and 50 MHz after every write and as the link leaves reset; a burst sends its last value.
    snap_cdc #(.W(203)) regs27_cdc (
        .sclk(sd_clk), .srst(rst_sd), .sreq(c_reg_wr),
        .sdata({c_own, c_video, c_audio[7], c_audio[4:0], c_fx, c_comb, c_hud, c_ov_en, c_loader_rst,
                c_win_pos, c_win_size, c_win_hstep, c_win_vstep, c_win_pan, c_ov_hwin, c_ov_hstep}),
        .dclk(clk27), .dreq(),
        .ddata({own27, r_video27, r_audio27, r_fx27, r_comb27, r_hud27, r_ov_en27, r_loader_rst27,
                r_win_pos27, r_win_size27, r_win_hstep27, r_win_vstep27, r_win_pan27, r_ov_hwin27, r_ov_hstep27})
    );
    snap_cdc #(.W(52)) regs50_cdc (
        .sclk(sd_clk), .srst(rst_sd), .sreq(c_reg_wr), .sdata({c_own, c_audio[6:5], c_ir_send, c_ir_payload}),
        .dclk(clk50), .dreq(), .ddata({own50, r_aud_off50, ir_send50, ir_payload50})
    );

    // STATUS at each output frame top, with the latest 27 MHz snapshot (refreshed continuously, so a stalled TD_CLK27
    // only freezes it) and 50 MHz state from the last millisecond.
    wire [31:0] s_frame, s_dt_crc;
    wire [7:0]  s_flips, s_count, s_dt_ses;
    wire        s_front, s_tag_seen, s_frozen, s_raw, s_aud_seen, s_mosaic, s_comb, s_dt_hs, s_dt_comp;
    wire        s_blue, s_dt_live;
    wire [3:0]  s_scheme, s_stream;
    wire [2:0]  s_aud_sel;
    wire [4:0]  s_dt_d;
    wire [15:0] s_dt_n, s_dt_have, s_pagreed, s_arate, s_tag_bad;
    wire [23:0] s_vpos;
    wire [31:0] s_vtc;
    wire [5:0]  s_vage, s_page;
    req_cdc #(.W(41)) top_cdc (
        .sclk(vclk), .sreq(top), .sdata({frames, flips, front}),
        .dclk(sd_clk), .dreq(st_req), .ddata({s_frame, s_flips, s_front})
    );
    snap_cdc #(.W(222)) st27_cdc (
        .sclk(clk27), .srst(1'b0), .sreq(1'b1),
        .sdata({tag_seen, frozen, raw, aud_seen, mosaic, comb_off, tag_scheme, tag_count,
                stream, aud_sel, dt_have_session, dt_complete, dt_session, dt_d, dt_n, dt_have, dt_crc_exp,
                vitc_pos, vitc_tc, vitc_age, pt_age, pt_agreed, aud_rate, tag_bad, in_blue, dt_active}),
        .dclk(sd_clk), .dreq(),
        .ddata({s_tag_seen, s_frozen, s_raw, s_aud_seen, s_mosaic, s_comb, s_scheme, s_count,
                s_stream, s_aud_sel, s_dt_hs, s_dt_comp, s_dt_ses, s_dt_d, s_dt_n, s_dt_have, s_dt_crc,
                s_vpos, s_vtc, s_vage, s_page, s_pagreed, s_arate, s_tag_bad, s_blue, s_dt_live})
    );
    assign ov_back = ~s_front;

    wire        s_lock, s_ltc_locked;
    wire [23:0] s_ltc_field;
    wire [31:0] s_ltc_tc, s_ltc_user;
    wire [15:0] s_ir_seq, s_cpu_under;
    req_cdc #(.W(122)) st50_cdc (
        .sclk(clk50), .sreq(ms_tick),
        .sdata({in_lock, ltc_locked, ltc_field, ltc_tc, ltc_user, ir_seq, cpu_under}),
        .dclk(sd_clk), .dreq(),
        .ddata({s_lock, s_ltc_locked, s_ltc_field, s_ltc_tc, s_ltc_user, s_ir_seq, s_cpu_under})
    );

    assign st_vec = {
        tap_drops, 8'd0, key_resets,                                // 60
        s_tag_bad,                                                  // 58
        s_cpu_under,                                                // 56
        up_packets,                                                 // 54
        dn_len_errs,                                                // 52
        dn_crc_errs,                                                // 50
        dn_packets,                                                 // 48
        s_ir_seq,                                                   // 46
        s_arate,                                                    // 44
        s_pagreed,                                                  // 42
        2'd0, s_page,                                               // 41
        2'd0, s_vage,                                               // 40
        s_vtc,                                                      // 36
        8'd0, s_vpos,                                               // 32
        s_ltc_user,                                                 // 28
        s_ltc_tc,                                                   // 24
        8'd0, s_ltc_field,                                          // 20
        s_dt_crc,                                                   // 16
        s_dt_have,                                                  // 14
        s_dt_n,                                                     // 12
        3'd0, s_dt_d,                                               // 11
        s_dt_ses,                                                   // 10
        s_blue, s_dt_live, s_mosaic, s_comb, dt_verify_ok, dt_verified, s_dt_comp, s_dt_hs, // 9
        1'b0, s_aud_sel, s_stream,                                  // 8
        s_count,                                                    // 7
        4'd0, s_scheme,                                             // 6
        s_aud_seen, s_ltc_locked, s_lock, c_own, s_raw, s_frozen, s_tag_seen, s_front,     // 5
        s_flips,                                                    // 4
        s_frame                                                     // 0
    };

    // Link counters. JTAG instance CPUL: probe {rdy, own, dn packets, dn CRC errors, dn length errors, up packets}.
    jtag_probe #(.IDX(12), .ID("CPUL"), .PW(66), .SW(1)) cpulp (
        .probe({rdy, c_own, dn_packets, dn_crc_errs, dn_len_errs, up_packets}),
        .source()
    );
endmodule
