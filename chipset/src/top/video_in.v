// Composite in: ADV7180 setup, BT.656 stats, blue-back detect, and field_writer filing each field in an SDRAM slot.
module video_in (
    input  wire        clk50,
    input  wire        rst50,
    input  wire        clk27,           // TD_CLK27
    input  wire        rst27,
    input  wire [7:0]  td_data,
    output wire        td_reset_n,
    output wire        i2c_scl,
    inout  wire        i2c_sda,
    input  wire        comb_off,        // clk27: the decoder's chroma combs off
    input  wire        raw,
    output wire        cfg_done,
    output wire        i2c_err,
    output wire        codec_err,
    output wire [7:0]  status1,
    output wire [7:0]  fields_per_sec,
    output wire [9:0]  lines_f0,
    output wire [9:0]  lines_f1,
    output wire [9:0]  active_f0,
    output wire [9:0]  active_f1,
    output wire [11:0] active_bytes,
    output wire [11:0] line_min,
    output wire [11:0] line_max,
    output wire [15:0] parity_errs,
    output wire [15:0] bad_codes,
    output wire        codes_seen,
    output wire        parity_ok,
    output wire        in_blue,
    output wire        wl_we,
    output wire [10:0] wl_waddr,
    output wire [15:0] wl_wdata,
    output wire        wr_req,
    output wire [12:0] wr_row,
    output wire        wr_half,
    output wire        line_start,
    output wire        frame_start,
    output wire        field_start,
    output wire        lock_start,      // a frame start the output may lock to
    output wire        tag_strobe,
    output wire [3:0]  tag_scheme,
    output wire [7:0]  tag_count,
    output reg         tag_seen,        // the last field was filed by tag (valid, or predicted over a dropout)
    output reg  [15:0] tag_bad,         // since reset: fields whose tag failed
    output wire        tag_hit,
    output wire [8:0]  tag_hit_line,
    output wire [7:0]  tag_hit_count,
    output wire        vitc_hit,
    output wire [8:0]  vitc_hit_line,
    output wire [63:0] vitc_data,
    output wire        ptag_hit,
    output wire [15:0] ptag_pos,
    output wire [6:0]  ptag_at,
    output wire [8:0]  ptag_line,
    output wire        vitc_line_done,
    output wire [2:0]  vitc_why,
    output wire [3:0]  vitc_why_grp,
    output wire        vitc_like,
    output wire        tap_sav,
    output wire        tap_y_en,
    output wire [7:0]  tap_y,
    output wire [8:0]  tap_line,
    output wire        frozen,
    output wire [1:0]  cur_slot,
    output wire        st_sav,
    output wire        st_y_en,
    output wire [7:0]  st_y,
    output wire        st_eav,
    output wire [7:0]  st_row
);
    wire comb_off50;
    sync2 comb_sync (.clk(clk50), .d(comb_off), .q(comb_off50));
    adv7180_ctrl ctrl (
        .clk(clk50), .rst(rst50), .comb_off(comb_off50),
        .td_reset_n(td_reset_n), .scl(i2c_scl), .sda(i2c_sda),
        .cfg_done(cfg_done), .i2c_err(i2c_err), .codec_err(codec_err), .status1(status1)
    );

    bt656_rx rx (
        .clk(clk27), .rst(rst27), .data(td_data),
        .fields_per_sec(fields_per_sec),
        .lines_f0(lines_f0), .lines_f1(lines_f1), .active_f0(active_f0), .active_f1(active_f1),
        .active_bytes(active_bytes), .line_min(line_min), .line_max(line_max),
        .parity_errs(parity_errs), .bad_codes(bad_codes),
        .codes_seen(codes_seen), .parity_ok(parity_ok)
    );
    blue_detect bluedet (.clk(clk27), .rst(rst27), .data(td_data), .blue(in_blue));

    // V_SKIP 13 puts line 21 of the GCV2 (vcr_sim) on stored line 0 in both fields.
    wire tag_ok, tag_held;
    field_writer #(.V_SKIP(13)) writer (
        .clk(clk27), .rst(rst27), .data(td_data), .raw(raw),
        .we(wl_we), .waddr(wl_waddr), .wdata(wl_wdata),
        .wr_req(wr_req), .wr_row(wr_row), .wr_half(wr_half),
        .line_start(line_start), .frame_start(frame_start), .field_start(field_start),
        .tag_strobe(tag_strobe), .tag_ok(tag_ok), .tag_held(tag_held), .tag_scheme(tag_scheme), .tag_count(tag_count),
        .tag_hit(tag_hit), .tag_hit_line(tag_hit_line), .tag_hit_count(tag_hit_count),
        .vitc_hit(vitc_hit), .vitc_hit_line(vitc_hit_line), .vitc_data(vitc_data),
        .ptag_hit(ptag_hit), .ptag_pos(ptag_pos), .ptag_at(ptag_at), .ptag_line(ptag_line),
        .vitc_line_done(vitc_line_done), .vitc_why(vitc_why), .vitc_why_grp(vitc_why_grp), .vitc_like(vitc_like),
        .tap_sav(tap_sav), .tap_y_en(tap_y_en), .tap_y(tap_y), .tap_line(tap_line),
        .frozen(frozen), .buf_slot(cur_slot),
        .st_sav(st_sav), .st_y_en(st_y_en), .st_y(st_y), .st_eav(st_eav), .st_row(st_row)
    );

    always @(posedge clk27)
        if (rst27) tag_bad <= 16'd0;
        else if (tag_strobe) begin
            tag_seen <= tag_held;
            tag_bad  <= tag_bad + (!tag_ok && tag_bad != 16'hFFFF);
        end

    // Frame lock only to NTSC lines (1716 clocks): the ADV7180 free-runs at other lengths with no input.
    localparam integer NTSC_LINE = 1716, NTSC_TOL = 16;
    reg  [11:0] lcount;
    reg         ntsc;
    always @(posedge clk27) begin
        lcount <= line_start ? 12'd0 : lcount + (lcount != 12'hFFF);
        if (line_start) ntsc <= lcount >= NTSC_LINE - 1 - NTSC_TOL && lcount <= NTSC_LINE - 1 + NTSC_TOL;
        else if (lcount == 12'hFFF) ntsc <= 1'b0;
    end
    // Lock only while the deck plays: four fields running of 262/263 lines. Pause (264/262), cue and review fields
    // would swing the trim between its limits frame by frame and tear the picture; free-run at 525 instead.
    reg  [9:0] ilines;
    reg  [2:0] playing_n;
    always @(posedge clk27)
        if (frame_start) begin
            playing_n <= ilines + 1'b1 == 10'd262 || ilines + 1'b1 == 10'd263 ? playing_n + (playing_n != 3'd7) : 3'd0;
            ilines    <= 10'd0;
        end else if (line_start) begin
            ilines <= ilines + (ilines != 10'h3FF);
        end
    assign lock_start = frame_start && ntsc && playing_n >= 3'd4;
endmodule
