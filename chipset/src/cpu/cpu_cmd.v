// DN packet parser and register file: one byte per clock, stalling on a full CPU audio FIFO, per
// register applied and per palette entry. FB_WR and AUD act as bytes arrive, the rest after the CRC.
module cpu_cmd #(
    parameter integer IDLE_MAX = 1000,  // 10 us at 100 MHz without a byte mid-packet: the CPU abandoned it
    parameter integer REG_GAP  = 32     // least clocks between reg_wr pulses, for the clock crossings after
) (
    input  wire        clk,             // sd_clk
    input  wire        rst,
    input  wire [7:0]  in_data,
    input  wire        in_empty,
    output wire        in_re,

    output reg         own,
    output reg  [8:0]  r_video,
    output reg  [7:0]  r_audio,
    output reg  [9:0]  r_fx,
    output reg         r_comb,
    output reg  [25:0] r_hud,
    output reg         r_ov_en,
    output reg         r_loader_rst,
    output reg  [18:0] r_win_pos,       // {y[8:0], x[9:0]}, output pixels
    output reg  [18:0] r_win_size,      // {h[8:0], w[9:0]}
    output reg  [19:0] r_win_hstep,     // Q16
    output reg  [19:0] r_win_vstep,     // Q16
    output reg  [24:0] r_win_pan,       // {wrap, inset Q8, pan Q16}
    output reg  [19:0] r_ov_hwin,       // {w[9:0], x[9:0]}, output pixels
    output reg  [24:0] r_ov_hstep,      // {first x[8:0], Q16 step[15:0]}
    output reg         r_tap,
    output reg  [47:0] ir_payload,
    output reg         ir_send,         // toggles
    output reg         reg_wr,          // pulse after any REG_WR packet

    output reg         fb_we,
    output reg  [16:0] fb_addr,
    output reg  [7:0]  fb_data,
    output reg         pal_we,          // one PAL_WR's entries on consecutive clocks
    output reg  [7:0]  pal_addr,
    output reg  [24:0] pal_data,        // {transparent, r, g, b}
    output reg         aud_we,
    output reg  [31:0] aud_data,        // {right, left}
    input  wire        aud_full,
    output reg         flip,            // pulse
    output reg         img_req,         // pulse
    output reg  [15:0] img_first,
    output reg  [15:0] img_count,
    output reg         echo_we,
    output reg  [7:0]  echo_addr,
    output reg  [7:0]  echo_data,
    output reg         echo_done,       // pulse, with echo_len
    output reg  [8:0]  echo_len,

    output reg  [15:0] packets,
    output reg  [15:0] crc_errs,
    output reg  [15:0] len_errs         // also packets abandoned mid-way
);
    `include "link_proto.vh"
    `include "crc16.vh"
    localparam [3:0] S_HUNT = 4'd0, S_TYPE = 4'd1, S_L0 = 4'd2, S_L1 = 4'd3, S_PAY = 4'd4, S_C0 = 4'd5,
                     S_C1 = 4'd6, S_APPLY = 4'd7, S_PAL = 4'd8;

    function [12:0] max_len(input [7:0] t);
        case (t)
            DN_REG_WR: max_len = DN_REG_WR_MAX;
            DN_FB_WR:  max_len = DN_FB_WR_MAX;
            DN_FLIP:   max_len = DN_FLIP_MAX;
            DN_PAL_WR: max_len = DN_PAL_WR_MAX;
            DN_AUD:    max_len = DN_AUD_MAX;
            DN_IMG_RD: max_len = DN_IMG_RD_MAX;
            DN_ECHO:   max_len = DN_ECHO_MAX;
            default:   max_len = 13'd0;
        endcase
    endfunction

    function len_ok(input [7:0] t, input [15:0] n);
        integer k;
        if (t == DN_REG_WR) begin
            len_ok = 1'b0;
            for (k = 1; k <= LINK_REG_MAX; k = k + 1) if (n == 5 * k) len_ok = 1'b1;
        end else len_ok = n <= max_len(t);
    endfunction

    reg  [3:0]  state;
    reg  [7:0]  typ;
    reg  [15:0] len, i, crc, crc_rx;
    reg  [7:0]  rbuf [0:DN_REG_WR_MAX-1];
    reg  [3:0]  apply_j;
    reg  [1:0]  sub;                    // byte within a 4-byte PAL entry or AUD frame
    reg  [23:0] acc;
    reg  [10:0] idle;
    reg  [5:0]  rw_gap;

    wire   stall = state == S_PAY && typ == DN_AUD && i >= 16'd2 && aud_full;
    assign in_re = !in_empty && !stall && state != S_APPLY && state != S_PAL;
    wire   mid   = state != S_HUNT && state != S_APPLY && state != S_PAL;
    wire [7:0] b = in_data;
    wire [15:0] i_n = i + 1'b1;

    wire [5:0]  ab  = {apply_j, 2'b0} + apply_j;        // 5 * apply_j
    wire [7:0]  ra  = rbuf[ab];
    wire [31:0] rv  = {rbuf[ab + 6'd4], rbuf[ab + 6'd3], rbuf[ab + 6'd2], rbuf[ab + 6'd1]};

    // PAL_WR entries are held until the CRC checks, then sent as one burst.
    reg  [24:0] pbuf [0:255];
    reg  [24:0] pb_q;
    reg  [8:0]  pal_n, pal_k;
    reg  [7:0]  pal_first;
    reg         pal_v;
    wire        pb_we = in_re && state == S_PAY && typ == DN_PAL_WR && i >= 16'd2 && sub == 2'd3 && !pal_n[8];
    always @(posedge clk) begin
        if (pb_we) pbuf[pal_n[7:0]] <= {b[0], acc[7:0], acc[15:8], acc[23:16]};
        pb_q <= pbuf[pal_k[7:0]];
    end

    // One register's value, as written by REG_WR or at reset; IR_HI also sends.
    task set_reg(input [7:0] a, input [31:0] v);
        case (a)
            REG_OWN:       own          <= v[0];
            REG_VIDEO:     r_video      <= v[8:0];
            REG_AUDIO:     r_audio      <= v[7:0];
            REG_FX:        r_fx         <= v[9:0];
            REG_COMB:      r_comb       <= v[0];
            REG_HUD:       r_hud        <= v[25:0];
            REG_OVERLAY:   r_ov_en      <= v[0];
            REG_LOADER:    r_loader_rst <= v[0];
            REG_IR_LO:     ir_payload[31:0] <= v;
            REG_IR_HI: begin
                ir_payload[47:32] <= v[15:0];
                ir_send           <= ~ir_send;
            end
            REG_WIN_POS:   r_win_pos   <= {v[24:16], v[9:0]};
            REG_WIN_SIZE:  r_win_size  <= {v[24:16], v[9:0]};
            REG_WIN_HSTEP: r_win_hstep <= v[19:0];
            REG_WIN_VSTEP: r_win_vstep <= v[19:0];
            REG_OV_HWIN:   r_ov_hwin   <= {v[25:16], v[9:0]};
            REG_OV_HSTEP:  r_ov_hstep  <= v[24:0];
            REG_TAP:       r_tap       <= v[0];
            REG_WIN_PAN:   r_win_pan   <= v[24:0];
            default: ;
        endcase
    endtask

    always @(posedge clk) begin
        {fb_we, pal_we, aud_we, flip, img_req, echo_we, echo_done, reg_wr, pal_v} <= 9'd0;
        if (rw_gap != 6'd0) rw_gap <= rw_gap - 1'b1;
        if (pal_v) begin
            pal_we   <= 1'b1;
            pal_addr <= pal_addr + 1'b1;
            pal_data <= pb_q;
        end
        if (rst) begin
            state        <= S_HUNT;
            rw_gap       <= REG_GAP[5:0];
            idle         <= 11'd0;
            set_reg(REG_OWN, REG_OWN_RESET);
            set_reg(REG_VIDEO, REG_VIDEO_RESET);
            set_reg(REG_AUDIO, REG_AUDIO_RESET);
            set_reg(REG_FX, REG_FX_RESET);
            set_reg(REG_COMB, REG_COMB_RESET);
            set_reg(REG_HUD, REG_HUD_RESET);
            set_reg(REG_OVERLAY, REG_OVERLAY_RESET);
            set_reg(REG_LOADER, REG_LOADER_RESET);
            set_reg(REG_WIN_POS, REG_WIN_POS_RESET);
            set_reg(REG_WIN_SIZE, REG_WIN_SIZE_RESET);
            set_reg(REG_WIN_HSTEP, REG_WIN_HSTEP_RESET);
            set_reg(REG_WIN_VSTEP, REG_WIN_VSTEP_RESET);
            set_reg(REG_OV_HWIN, REG_OV_HWIN_RESET);
            set_reg(REG_OV_HSTEP, REG_OV_HSTEP_RESET);
            set_reg(REG_TAP, REG_TAP_RESET);
            set_reg(REG_WIN_PAN, REG_WIN_PAN_RESET);
            ir_send      <= 1'b0;
            packets      <= 16'd0;
            crc_errs     <= 16'd0;
            len_errs     <= 16'd0;
        end else if (state == S_PAL) begin
            pal_k <= pal_k + 1'b1;
            pal_v <= 1'b1;
            if (pal_k + 1'b1 == pal_n) state <= S_HUNT;
        end else if (state == S_APPLY && rw_gap == 6'd0) begin
            set_reg(ra, rv);
            apply_j <= apply_j + 1'b1;
            if ({apply_j + 1'b1, 2'b0} + apply_j + 1'b1 >= len) begin
                reg_wr <= 1'b1;
                rw_gap <= REG_GAP[5:0];
                state  <= S_HUNT;
            end
        end else if (!in_re) begin
            idle <= mid && !stall ? idle + 1'b1 : 11'd0;
            if (mid && !stall && idle == IDLE_MAX - 1) begin
                len_errs <= len_errs + 1'b1;
                state    <= S_HUNT;
            end
        end else begin
            idle <= 11'd0;
            case (state)
                S_HUNT: if (b == LINK_SYNC) begin
                    state <= S_TYPE;
                    crc   <= LINK_CRC_INIT;
                end
                S_TYPE: begin
                    typ   <= b;
                    crc   <= crc16(crc, b);
                    state <= S_L0;
                end
                S_L0: begin
                    len[7:0] <= b;
                    crc      <= crc16(crc, b);
                    state    <= S_L1;
                end
                S_L1: begin
                    len[15:8] <= b;
                    crc       <= crc16(crc, b);
                    i         <= 16'd0;
                    sub       <= 2'd0;
                    pal_n     <= 9'd0;
                    if (!len_ok(typ, {b, len[7:0]})) begin
                        len_errs <= len_errs + 1'b1;
                        state    <= S_HUNT;
                    end else state <= {b, len[7:0]} == 16'd0 ? S_C0 : S_PAY;
                end
                S_PAY: begin
                    crc <= crc16(crc, b);
                    i   <= i_n;
                    if (i_n == len) state <= S_C0;
                    case (typ)
                        DN_REG_WR: rbuf[i[5:0]] <= b;
                        DN_FB_WR: begin
                            if (i < 16'd4) acc <= {b, acc[23:8]};
                            if (i == 16'd3) fb_addr <= acc[16:0] - 1'b1;    // acc = offset bytes 2, 1, 0
                            if (i >= 16'd4) begin
                                fb_we   <= 1'b1;
                                fb_addr <= fb_addr + 1'b1;
                                fb_data <= b;
                            end
                        end
                        DN_PAL_WR: begin
                            if (i == 16'd0) pal_first <= b;
                            if (i >= 16'd2) begin
                                sub <= sub + 1'b1;
                                acc <= {b, acc[23:8]};
                                if (sub == 2'd3 && !pal_n[8]) pal_n <= pal_n + 1'b1;
                            end
                        end
                        DN_AUD: if (i >= 16'd2) begin
                            sub <= sub + 1'b1;
                            acc <= {b, acc[23:8]};
                            if (sub == 2'd3) begin
                                aud_we   <= 1'b1;
                                aud_data <= {b, acc};
                            end
                        end
                        DN_IMG_RD: rbuf[i[5:0]] <= b;
                        DN_ECHO: begin
                            echo_we   <= 1'b1;
                            echo_addr <= i[7:0];
                            echo_data <= b;
                        end
                        default: ;
                    endcase
                end
                S_C0: begin
                    crc_rx[7:0] <= b;
                    state       <= S_C1;
                end
                S_C1: begin
                    state <= S_HUNT;
                    if ({b, crc_rx[7:0]} != crc) crc_errs <= crc_errs + 1'b1;
                    else begin
                        packets <= packets + 1'b1;
                        case (typ)
                            DN_REG_WR: begin
                                apply_j <= 4'd0;
                                state   <= S_APPLY;
                            end
                            DN_FLIP: flip <= 1'b1;
                            DN_PAL_WR: if (pal_n != 9'd0) begin
                                pal_k    <= 9'd0;
                                pal_addr <= pal_first - 1'b1;
                                state    <= S_PAL;
                            end
                            DN_IMG_RD: begin
                                img_first <= {rbuf[1], rbuf[0]};
                                img_count <= {rbuf[3], rbuf[2]};
                                img_req   <= 1'b1;
                            end
                            DN_ECHO: begin
                                echo_len  <= len[8:0];
                                echo_done <= 1'b1;
                            end
                            default: ;
                        endcase
                    end
                end
                default: state <= S_HUNT;
            endcase
        end
    end
endmodule
