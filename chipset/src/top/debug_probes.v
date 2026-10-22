// JTAG debug taps (In-System Sources and Probes): stats and stream select (VSTA, scripts/de1_ctl.sh), tag and VITC
// line statistics, line capture, header pins, the link test and VGA sync timing.
module debug_probes (
    input  wire         clk50,
    input  wire         clk27,          // TD_CLK27
    input  wire         lk_clk,
    input  wire [214:0] stats27,        // VSTA fields from 27 MHz
    input  wire [121:0] stats50,        // and the rest, static or counters that may read torn
    output wire [15:0]  sel_src,        // VSTA source: stream and audio select
    input  wire         tag_hit,
    input  wire [8:0]   tag_hit_line,
    input  wire [7:0]   tag_hit_count,
    input  wire         vitc_line_done,
    input  wire         vitc_like,
    input  wire [2:0]   vitc_why,
    input  wire [3:0]   vitc_why_grp,
    input  wire         tap_sav,
    input  wire         tap_y_en,
    input  wire [7:0]   tap_y,
    input  wire [8:0]   tap_line,
    input  wire [71:0]  hdr_pins,       // {GPIO_1, GPIO_0}
    output wire [17:0]  hdr_src,
    input  wire [7:0]   link_pins,
    input  wire         vga_hs,
    input  wire         vga_vs
);
    // Stats change at most once per line and are only displayed, so a plain double flop into
    // the 50 MHz domain is enough; a rare torn multi-bit read just flickers one readout.
    wire [214:0] s50;
    sync2 #(.W(215)) s50_sync (.clk(clk50), .d(stats27), .q(s50));
    jtag_probe #(.IDX(0), .ID("VSTA"), .PW(337), .SW(16)) probe (
        .probe({stats50, s50}),
        .source(sel_src)
    );

    // Where the tag was found: hits per active line 0-20 (12-bit, saturating) and the last hit.
    // JTAG instance TAGW: source {clear}; probe {hit seq[7:0], last line[4:0], last count[7:0], bins 20..0}.
    wire        tagw_src, tagw_clr;
    reg  [11:0] tag_bins [0:20];
    reg  [7:0]  tag_hit_seq;
    reg  [4:0]  tag_hit_last;
    reg  [7:0]  tag_hit_cnt;
    wire [251:0] tag_bins_flat;
    integer bi;
    sync2 tagw_sync (.clk(clk27), .d(tagw_src), .q(tagw_clr));
    always @(posedge clk27) begin
        if (tagw_clr) begin
            for (bi = 0; bi < 21; bi = bi + 1) tag_bins[bi] <= 12'd0;
            tag_hit_seq <= 8'd0;
        end else if (tag_hit && tag_hit_line < 9'd21) begin
            if (tag_bins[tag_hit_line] != 12'hFFF) tag_bins[tag_hit_line] <= tag_bins[tag_hit_line] + 1'b1;
            tag_hit_seq  <= tag_hit_seq + 1'b1;
            tag_hit_last <= tag_hit_line[4:0];
            tag_hit_cnt  <= tag_hit_count;
        end
    end
    genvar gb;
    generate for (gb = 0; gb < 21; gb = gb + 1) begin : g_bins
        assign tag_bins_flat[12 * gb +: 12] = tag_bins[gb];
    end endgenerate
    jtag_probe #(.IDX(5), .ID("TAGW"), .PW(273), .SW(1)) tagw (
        .probe({tag_hit_seq, tag_hit_last, tag_hit_cnt, tag_bins_flat}),
        .source(tagw_src)
    );

    // Why VITC-like lines (a short first white run) fail, per vitc_slicer verdict and first bad sync group (0-8, none).
    // JTAG instance VDIA: source {clear}; probe {verdicts 5..0 (16 bits each), groups none, 8..0 (12 bits each)}.
    wire        vdia_src, vdia_clr;
    reg  [15:0] vd_why [0:5];
    reg  [11:0] vd_grp [0:9];
    wire [3:0]  vd_g = vitc_why_grp > 4'd8 ? 4'd9 : vitc_why_grp;
    integer vi;
    sync2 vdia_sync (.clk(clk27), .d(vdia_src), .q(vdia_clr));
    always @(posedge clk27) begin
        if (vdia_clr) begin
            for (vi = 0; vi < 6; vi = vi + 1) vd_why[vi] <= 16'd0;
            for (vi = 0; vi < 10; vi = vi + 1) vd_grp[vi] <= 12'd0;
        end else if (vitc_line_done && vitc_like) begin
            if (vd_why[vitc_why] != 16'hFFFF) vd_why[vitc_why] <= vd_why[vitc_why] + 1'b1;
            if (vitc_why == 3'd3 && vd_grp[vd_g] != 12'hFFF) vd_grp[vd_g] <= vd_grp[vd_g] + 1'b1;
        end
    end
    jtag_probe #(.IDX(7), .ID("VDIA"), .PW(216), .SW(1)) vdiap (
        .probe({vd_why[5], vd_why[4], vd_why[3], vd_why[2], vd_why[1], vd_why[0],
                vd_grp[9], vd_grp[8], vd_grp[7], vd_grp[6], vd_grp[5], vd_grp[4], vd_grp[3], vd_grp[2],
                vd_grp[1], vd_grp[0]}),
        .source(vdia_src)
    );

    // Raw luma of any one line, for looking at what trick play does to VITC. JTAG instance LCAP: source
    // {arm, line[8:0], page[4:0]}; probe {captures[7:0], page[4:0], 32 samples} (line_cap).
    wire [14:0]  lcap_src, lcap_s;
    wire [7:0]   lcap_caps;
    wire [4:0]   lcap_page;
    wire [255:0] lcap_data;
    sync2 #(.W(15)) lcap_sync (.clk(clk27), .d(lcap_src), .q(lcap_s));
    line_cap lcap (
        .clk(clk27), .sav(tap_sav), .y_en(tap_y_en), .y(tap_y), .line(tap_line),
        .arm(lcap_s[14]), .sel(lcap_s[13:5]), .page(lcap_s[4:0]),
        .caps(lcap_caps), .page_out(lcap_page), .page_data(lcap_data)
    );
    jtag_probe #(.IDX(8), .ID("LCAP"), .PW(269), .SW(15)) lcapp (
        .probe({lcap_caps, lcap_page, lcap_data}),
        .source(lcap_src)
    );

    // JTAG HDRP: probe {GPIO_1, GPIO_0}; source {drive [8:0], drive [15:9], out[15:0]}: pin tests drive out, 0 = normal.
    wire [71:0] hdr;
    sync2 #(.W(72)) hdr_sync (.clk(clk50), .d(hdr_pins), .q(hdr));
    jtag_probe #(.IDX(10), .ID("HDRP"), .PW(72), .SW(18)) hdrp (
        .probe(hdr),
        .source(hdr_src)
    );

    // PGA2350 link test (cpu link_test), sampled at 250 MHz. JTAG instance LINK: probe
    // {last, rate, errs, words}, source {clr}.
    wire        lk_clr;
    wire [31:0] lk_words, lk_errs, lk_rate;
    wire [6:0]  lk_last;
    link_rx lk (
        .clk(lk_clk), .clr(lk_clr), .pins(link_pins),
        .words(lk_words), .errs(lk_errs), .rate(lk_rate), .last(lk_last)
    );
    jtag_probe #(.IDX(11), .ID("LINK"), .PW(103), .SW(1)) lkp (
        .probe({lk_last, lk_rate, lk_errs, lk_words}),
        .source(lk_clr)
    );

    // VGA sync periods in CLOCK_50 cycles, min/max over each 2^20-cycle window (21 ms): 1589 and 834166 nominal.
    // JTAG instance VGAT: probe {window count, hs min, hs max, vs min, vs max}.
    reg  [2:0]  vt_hs, vt_vs;
    reg  [11:0] vt_hc, vt_hmin, vt_hmax, vt_hmin_l, vt_hmax_l;
    reg  [21:0] vt_vc, vt_vmin, vt_vmax, vt_vmin_l, vt_vmax_l;
    reg  [19:0] vt_win;
    reg  [7:0]  vt_n;
    always @(posedge clk50) begin
        vt_hs  <= {vt_hs[1:0], vga_hs};
        vt_vs  <= {vt_vs[1:0], vga_vs};
        vt_hc  <= vt_hc + (vt_hc != 12'hFFF);
        vt_vc  <= vt_vc + (vt_vc != 22'h3FFFFF);
        vt_win <= vt_win + 1'b1;
        if (vt_hs[2] && !vt_hs[1]) begin
            vt_hc <= 12'd1;
            if (vt_hc < vt_hmin) vt_hmin <= vt_hc;
            if (vt_hc > vt_hmax) vt_hmax <= vt_hc;
        end
        if (vt_vs[2] && !vt_vs[1]) begin
            vt_vc <= 22'd1;
            if (vt_vc < vt_vmin) vt_vmin <= vt_vc;
            if (vt_vc > vt_vmax) vt_vmax <= vt_vc;
        end
        if (vt_win == 20'd0) begin
            {vt_hmin_l, vt_hmax_l, vt_vmin_l, vt_vmax_l} <= {vt_hmin, vt_hmax, vt_vmin, vt_vmax};
            {vt_hmin, vt_hmax, vt_vmin, vt_vmax} <= {12'hFFF, 12'd0, 22'h3FFFFF, 22'd0};
            vt_n <= vt_n + 1'b1;
        end
    end
    jtag_probe #(.IDX(13), .ID("VGAT"), .PW(76), .SW(1)) vgatp (
        .probe({vt_n, vt_hmin_l, vt_hmax_l, vt_vmin_l, vt_vmax_l}),
        .source()
    );
endmodule
