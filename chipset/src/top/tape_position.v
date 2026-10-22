// Tape position from LTC on the right line in, VITC in play and position tags in pause and cue,
// and the HUD tape counter. JTAG instances LTCP, VITC and PTAG.
module tape_position (
    input  wire        clk50,
    input  wire        rst50,
    input  wire        clk27,           // TD_CLK27
    input  wire        r_valid,         // clk50: right line in
    input  wire [15:0] r_in,
    input  wire        frame_start,     // clk27, from field_writer
    input  wire        tag_strobe,
    input  wire [7:0]  tag_count,
    input  wire        vitc_hit,
    input  wire [8:0]  vitc_hit_line,
    input  wire [63:0] vitc_data,
    input  wire        tap_sav,
    input  wire [8:0]  tap_line,
    input  wire        ptag_hit,
    input  wire [15:0] ptag_pos,
    input  wire [6:0]  ptag_at,
    input  wire [8:0]  ptag_line,
    output reg         ltc_tick,        // clk50: a field start, with the LTC below for it
    output wire        ltc_locked,
    output wire [23:0] ltc_field,
    output wire [31:0] ltc_tc,
    output wire [31:0] ltc_user,
    output wire [19:0] hud_tape,        // clk27: BCD {H, M tens, M, S tens, S}
    output wire [23:0] vitc_pos,
    output reg  [31:0] vitc_tc,
    output reg  [5:0]  vitc_age = 6'h3F,    // fields since a good word, saturating
    output reg  [5:0]  pt_age = 6'h3F,
    output reg  [15:0] pt_agreed
);
    `include "timecode.vh"
    // Each locked field start checks the tag count agrees (streams.c). JTAG instance LTCP: source {reset counters}.
    wire        ltc_src, ltc_fs, ltc_clr;
    wire [15:0] ltc_words, ltc_bad, ltc_jumps, ltc_peak;
    wire [19:0] ltc_phase;
    sync2 ltc_sync (.clk(clk50), .d(ltc_src), .q(ltc_clr));
    req_cdc #(.W(1)) fs_cdc (
        .sclk(clk27), .sreq(frame_start), .sdata(1'b0), .dclk(clk50), .dreq(ltc_fs), .ddata()
    );
    ltc_rx ltc (
        .clk(clk50), .rst(rst50 | ltc_clr), .x_valid(r_valid), .x(r_in), .field_start(ltc_fs),
        .field(ltc_field), .tc(ltc_tc), .locked(ltc_locked), .user(ltc_user),
        .words(ltc_words), .bad(ltc_bad), .jumps(ltc_jumps), .phase(ltc_phase), .peak(ltc_peak)
    );
    wire [6:0]  tag50;
    reg         ltc_fs_d;
    reg  [15:0] ltc_tag_ok, ltc_tag_bad;
    sync2 #(.W(7)) tag50_sync (.clk(clk50), .d(tag_count[6:0]), .q(tag50));
    always @(posedge clk50) begin
        {ltc_tick, ltc_fs_d} <= {ltc_fs_d, ltc_fs};
        if (rst50 | ltc_clr) begin
            ltc_tag_ok  <= 16'd0;
            ltc_tag_bad <= 16'd0;
        end else if (ltc_tick && ltc_locked) begin
            if (ltc_field[6:0] == tag50) ltc_tag_ok <= ltc_tag_ok + (ltc_tag_ok != 16'hFFFF);
            else ltc_tag_bad <= ltc_tag_bad + (ltc_tag_bad != 16'hFFFF);
        end
    end
    jtag_probe #(.IDX(2), .ID("LTCP"), .PW(205), .SW(1)) ltcp (
        .probe({ltc_tag_ok, ltc_tag_bad, ltc_peak, ltc_phase, ltc_jumps, ltc_bad, ltc_words, ltc_user,
                ltc_tc, ltc_locked, ltc_field}),
        .source(ltc_src)
    );
    wire        ltc_hud_locked;
    wire [19:0] ltc_hud;
    req_cdc #(.W(21)) tape_cdc (
        .sclk(clk50), .sreq(ltc_tick), .sdata({ltc_locked, ltc_tc[27:8]}), .dclk(clk27), .dreq(),
        .ddata({ltc_hud_locked, ltc_hud})
    );

    // VITC: last good word (BCD tc, mark, user) and position 2 * frame + mark; on the HUD while the LTC is unlocked.
    // JTAG VITC: source {clear}; probe {reads[15:0], seq[7:0], line[4:0], age[5:0], mark, tc[31:0], pos[23:0], user[31:0]}.
    reg  [31:0] vitc_user;
    reg         vitc_mark;
    reg  [4:0]  vitc_line_r;
    reg  [7:0]  vitc_seq;
    reg  [15:0] vitc_reads;
    localparam [5:0] VITC_HUD_AGE = 6'd30;
    wire        vitc_src, vitc_clr;
    sync2 vitc_sync (.clk(clk27), .d(vitc_src), .q(vitc_clr));
    always @(posedge clk27) begin
        if (tag_strobe) vitc_age <= vitc_age + (vitc_age != 6'h3F);
        if (vitc_clr) begin
            vitc_reads <= 16'd0;
            vitc_seq   <= 8'd0;
        end else if (vitc_hit) begin
            vitc_tc     <= tc_bcd(vitc_data);
            vitc_user   <= tc_user(vitc_data);
            vitc_mark   <= vitc_data[27];
            vitc_line_r <= vitc_hit_line[4:0];
            vitc_seq    <= vitc_seq + 1'b1;
            vitc_reads  <= vitc_reads + 1'b1;
            vitc_age    <= 6'd0;
        end
    end
    // The field position; vitc_tc holds for a line at least.
    wire [21:0] vitc_frame;
    assign vitc_pos = {1'b0, vitc_frame, vitc_mark};
    tc_frames vitc_frames (.clk(clk27), .tc(vitc_tc), .frame(vitc_frame));
    assign hud_tape = !ltc_hud_locked && vitc_age < VITC_HUD_AGE ? vitc_tc[27:8] : ltc_hud;
    jtag_probe #(.IDX(6), .ID("VITC"), .PW(124), .SW(1)) vitcp (
        .probe({vitc_reads, vitc_seq, vitc_line_r, vitc_age, vitc_mark, vitc_tc, vitc_pos, vitc_user}),
        .source(vitc_src)
    );

    // Position tags; agreed needs 2+ reads (a lone one can be a CRC-8 fluke). PTAG source {clear}; probe {reads, fields read,
    // agreed, conflict (16 b each), seq[7:0], line[4:0], at[6:0], age[5:0], pos[15:0], field reads[4:0], agreed pos[15:0]}.
    wire        ptag_src, ptag_clr;
    reg  [15:0] pt_reads, pt_fread, pt_fagree, pt_fconf, pt_pos, pf_pos;
    reg  [7:0]  pt_seq;
    reg  [4:0]  pt_line, pf_n, pt_fn;
    reg  [6:0]  pt_at;
    reg         pf_conf;
    sync2 ptag_sync (.clk(clk27), .d(ptag_src), .q(ptag_clr));
    always @(posedge clk27) begin
        if (tag_strobe) pt_age <= pt_age + (pt_age != 6'h3F);
        if (ptag_clr) begin
            {pt_reads, pt_fread, pt_fagree, pt_fconf, pt_seq} <= 72'd0;
        end else if (tap_sav && tap_line == 9'd1) begin      // a field's first SAV: its first EAV has counted line 0
            pt_fn <= pf_n;
            if (pf_n != 5'd0) pt_fread <= pt_fread + 1'b1;
            if (pf_n > 5'd1 && !pf_conf) begin
                pt_fagree <= pt_fagree + 1'b1;
                pt_agreed <= pf_pos;
            end
            if (pf_conf) pt_fconf <= pt_fconf + 1'b1;
            pf_n    <= 5'd0;
            pf_conf <= 1'b0;
        end else if (ptag_hit) begin
            pt_reads <= pt_reads + 1'b1;
            pt_seq   <= pt_seq + 1'b1;
            pt_pos   <= ptag_pos;
            pt_line  <= ptag_line[4:0];
            pt_at    <= ptag_at;
            pt_age   <= 6'd0;
            pf_n     <= pf_n + (pf_n != 5'h1F);
            if (pf_n == 5'd0) pf_pos <= ptag_pos;
            else if (ptag_pos != pf_pos) pf_conf <= 1'b1;
        end
    end
    jtag_probe #(.IDX(9), .ID("PTAG"), .PW(127), .SW(1)) ptagp (
        .probe({pt_reads, pt_fread, pt_fagree, pt_fconf, pt_seq, pt_line, pt_at, pt_age, pt_pos, pt_fn, pt_agreed}),
        .source(ptag_src)
    );
endmodule
