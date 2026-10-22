// Output side: the shown stream's controls carried to vclk once per input line, vga_out, and the overlay's double
// buffer (FLIP swaps at the next frame top) with the frame counts for STATUS.
module video_out (
    input  wire        clk27,           // TD_CLK27
    input  wire        vclk,
    input  wire        sd_clk,
    input  wire        line_start,      // clk27
    input  wire        lock_start,
    input  wire [7:0]  view,
    input  wire [9:0]  fx,
    input  wire        raw,
    input  wire [3:0]  slot_data,
    input  wire        aud_cells,
    input  wire        aud_fit,
    input  wire [1:0]  aud_nl,
    input  wire [19:0] hud_tape,
    input  wire        own,             // game CPU registers, clk27
    input  wire [25:0] r_hud,
    input  wire        r_ov_en,
    input  wire [18:0] r_win_pos,
    input  wire [18:0] r_win_size,
    input  wire [19:0] r_win_hstep,
    input  wire [19:0] r_win_vstep,
    input  wire [24:0] r_win_pan,
    input  wire [19:0] r_ov_hwin,
    input  wire [24:0] r_ov_hstep,
    input  wire        ov_we,           // sd_clk: framebuffer and palette writes
    input  wire [16:0] ov_waddr,
    input  wire [7:0]  ov_wdata,
    input  wire        ov_back,
    input  wire        pal_we,
    input  wire [7:0]  pal_addr,
    input  wire [24:0] pal_data,
    input  wire        flip,
    output wire        fetch,           // vclk: to frame_store
    output wire [1:0]  fetch_slot,
    output wire [12:0] fetch_row,
    output wire [11:0] rl_raddr,
    input  wire [15:0] rl_rdata,
    input  wire [7:0]  rl_rdata_p,
    output wire [7:0]  r,
    output wire [7:0]  g,
    output wire [7:0]  b,
    output wire        hs_n,
    output wire        vs_n,
    output wire        blank_n,
    output wire        vblank,
    output reg         top,             // vclk: a frame top, with its counts below
    output reg  [31:0] frames,
    output reg  [7:0]  flips,
    output reg         front
);
    `include "cpu/link_proto.vh"
    // The registers' reset values unless the CPU owns the controls: the video full screen.
    wire [18:0] win_pos   = own ? r_win_pos   : {REG_WIN_POS_RESET[24:16], REG_WIN_POS_RESET[9:0]};
    wire [18:0] win_size  = own ? r_win_size  : {REG_WIN_SIZE_RESET[24:16], REG_WIN_SIZE_RESET[9:0]};
    wire [19:0] win_hstep = own ? r_win_hstep : REG_WIN_HSTEP_RESET[19:0];
    wire [19:0] win_vstep = own ? r_win_vstep : REG_WIN_VSTEP_RESET[19:0];
    wire [24:0] win_pan   = own ? r_win_pan   : REG_WIN_PAN_RESET[24:0];
    wire [19:0] ov_hwin   = own ? r_ov_hwin   : {REG_OV_HWIN_RESET[25:16], REG_OV_HWIN_RESET[9:0]};
    wire [24:0] ov_hstep  = own ? r_ov_hstep  : REG_OV_HSTEP_RESET[24:0];
    wire        hide_video = !raw && slot_data[view[7:6]];      // the shown slot holds a data field

    wire [7:0]  view_v;
    wire [9:0]  fx_v, win_w_v, win_x_v;
    wire        cells_v, fit_v, own_v, ov_en_v;
    wire [1:0]  nl_v;
    wire [19:0] tape_v, hstep_v, vstep_v;
    wire [25:0] hud_v;
    wire [8:0]  win_y_v, win_h_v;
    wire [19:0] ov_hwin_v;
    wire [24:0] ov_hstep_v, pan_v;
    req_cdc #(.W(218)) vctl_cdc (
        .sclk(clk27), .sreq(line_start),
        .sdata({view, fx, aud_cells, aud_fit, aud_nl, hud_tape, own, r_hud, r_ov_en,
                win_pos[9:0], win_pos[18:10], hide_video ? 10'd0 : win_size[9:0], win_size[18:10], win_hstep, win_vstep,
                win_pan, ov_hwin, ov_hstep}),
        .dclk(vclk), .dreq(),
        .ddata({view_v, fx_v, cells_v, fit_v, nl_v, tape_v, own_v, hud_v, ov_en_v,
                win_x_v, win_y_v, win_w_v, win_h_v, hstep_v, vstep_v, pan_v, ov_hwin_v, ov_hstep_v})
    );
    wire fin;
    req_cdc #(.W(1)) fin_cdc (
        .sclk(clk27), .sreq(lock_start), .sdata(1'b0), .dclk(vclk), .dreq(fin), .ddata()
    );
    wire frame_top;
    vga_out vga (
        .clk(vclk), .fin(fin),
        .view(view_v), .fx(fx_v), .mask_audio(cells_v),
        .fit(fit_v), .aud_n_log2(nl_v), .tape(tape_v),
        .hud_cpu(own_v), .hud_vals(hud_v),
        .ov_wclk(sd_clk), .ov_we(ov_we), .ov_waddr(ov_waddr), .ov_wdata(ov_wdata), .ov_back(ov_back),
        .pal_we(pal_we), .pal_addr(pal_addr), .pal_data(pal_data), .ov_enable(ov_en_v), .ov_front(front),
        .win_x(win_x_v), .win_y(win_y_v), .win_w(win_w_v), .win_h(win_h_v),
        .win_hstep(hstep_v), .win_vstep(vstep_v), .win_pan(pan_v), .ov_hwin(ov_hwin_v), .ov_hstep(ov_hstep_v),
        .frame_top(frame_top), .vblank(vblank),
        .fetch(fetch), .fetch_row(fetch_row), .fetch_slot(fetch_slot),
        .raddr(rl_raddr), .rdata(rl_rdata), .rdata_p(rl_rdata_p),
        .r(r), .g(g), .b(b), .hs_n(hs_n), .vs_n(vs_n), .blank_n(blank_n)
    );

    wire flip_v;
    reg  flip_pend;
    req_cdc #(.W(1)) flip_cdc (
        .sclk(sd_clk), .sreq(flip), .sdata(1'b0), .dclk(vclk), .dreq(flip_v), .ddata()
    );
    always @(posedge vclk) begin
        top <= frame_top;
        if (flip_v) flip_pend <= 1'b1;
        if (frame_top) begin
            frames <= frames + 1'b1;
            if (flip_pend) begin
                front     <= ~front;
                flips     <= flips + 1'b1;
                flip_pend <= 1'b0;
            end
        end
    end
endmodule
