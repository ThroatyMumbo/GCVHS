`timescale 1ns / 1ps
// cpu_link against tools/cpulink.py stim: DN bytes driven as the CPU's PIO does at 315 MHz (9.5 ns either
// side of each strobe edge, waiting on RDY, with the stim's pauses), UP nibbles collected, effects logged.
module tb_cpu;
    `include "out/cpu_params.vh"
    `include "link_proto.vh"

    reg lk_clk = 0, clk = 0, aclk = 0;
    always #2  lk_clk = ~lk_clk;
    always #5  clk    = ~clk;
    always #10 aclk   = ~aclk;
    reg rst = 1, arst = 1;

    reg  [8:0]  dn = 9'd0;
    wire        up_strobe, rdy;
    wire [3:0]  up_data;
    wire        own, r_comb, r_ov_en, r_loader_rst, ir_send, reg_wr;
    wire [8:0]  r_video;
    wire [7:0]  r_audio;
    wire [9:0]  r_fx;
    wire [25:0] r_hud;
    wire [47:0] ir_payload;
    wire        fb_we, pal_we, flip, img_req, img_done, a_empty;
    wire [16:0] fb_addr;
    wire [7:0]  fb_data, pal_addr;
    wire [24:0] pal_data;
    wire [15:0] img_first, img_count, dn_packets, dn_crc_errs, dn_len_errs, up_packets;
    wire [4:0]  img_sel;
    wire [31:0] a_data;
    reg         img_ready = 0, status_req = 0, a_re = 0, tap_req = 0;
    reg  [175:0] tap_line;
    wire        r_tap;
    wire [24:0] r_win_pan;
    wire [15:0] tap_drops;
    reg  [14:0] img_idx;
    reg  [8*LINK_STATUS_BYTES-1:0] status;
    wire [15:0] img_m = img_idx * 7 + img_sel * 13 + 1;
    wire [7:0]  img_b = img_m[7:0];

    cpu_link #(.SB(LINK_STATUS_BYTES)) dut (
        .lk_clk(lk_clk), .clk(clk), .rst(rst), .aclk(aclk), .arst(arst),
        .dn_pins(dn), .up_strobe(up_strobe), .up_data(up_data), .rdy(rdy),
        .own(own), .r_video(r_video), .r_audio(r_audio), .r_fx(r_fx), .r_comb(r_comb), .r_hud(r_hud),
        .r_ov_en(r_ov_en), .r_loader_rst(r_loader_rst), .r_win_pan(r_win_pan), .ir_payload(ir_payload),
        .ir_send(ir_send), .reg_wr(reg_wr),
        .fb_we(fb_we), .fb_addr(fb_addr), .fb_data(fb_data),
        .pal_we(pal_we), .pal_addr(pal_addr), .pal_data(pal_data), .flip(flip),
        .img_req(img_req), .img_first(img_first), .img_count(img_count), .img_ready(img_ready),
        .img_idx(img_idx), .img_d(5'd17), .img_sel(img_sel), .img_byte(img_b),
        .img_done(img_done),
        .status_req(status_req), .status(status),
        .r_tap(r_tap), .tap_req(tap_req), .tap_line(tap_line), .tap_drops(tap_drops),
        .a_re(a_re), .a_empty(a_empty), .a_data(a_data),
        .dn_packets(dn_packets), .dn_crc_errs(dn_crc_errs), .dn_len_errs(dn_len_errs), .up_packets(up_packets)
    );

    integer fo, i, t, j;
    reg [16:0] stim [0:DN_BYTES-1];     // {hold the audio drain 300 us, pause in us before the byte, byte}
    initial begin
        $readmemh("out/cpu_dn.hex", stim);
        fo = $fopen("out/cpu_out.txt", "w");
        for (i = 0; i < 64; i = i + 1) status[8 * i +: 8] = (i * 3 + 5) & 8'hFF;
    end

    // Stands in for data_verify's slot reads.
    reg [15:0] left, next;
    always @(posedge clk) begin
        if (img_req) begin
            next <= img_first;
            left <= img_count;
        end else if (img_done) begin
            img_ready <= 1'b0;
            next      <= next + 1'b1;
            left      <= left - 1'b1;
        end else if (!img_ready && left != 0) begin
            img_ready <= 1'b1;
            img_idx   <= next[14:0];
        end
    end

    integer cyc = 0, rw_last = 0, rw_min = 1 << 30, ir_sends = 0, stall_n = 0, stall_max = 0;
    always @(posedge clk) begin
        cyc = cyc + 1;
        stall_n = dut.aud_full && !dut.dn_re ? stall_n + 1 : 0;
        if (stall_n > stall_max) stall_max = stall_n;
        if (fb_we && fb_addr < 17'd76800) $fdisplay(fo, "FB %05x %02x", fb_addr, fb_data);
        if (pal_we) $fdisplay(fo, "PAL %02x %07x %0d", pal_addr, pal_data, cyc);
        if (reg_wr) begin
            if (rw_last != 0 && cyc - rw_last < rw_min) rw_min = cyc - rw_last;
            rw_last = cyc;
        end
    end
    always @(ir_send) if (!rst) ir_sends = ir_sends + 1;
    reg   a_hold = 0;
    event hold_ev;
    always @(hold_ev) begin
        a_hold = 1;
        #300000 a_hold = 0;
    end
    always @(posedge aclk) begin
        a_re <= !a_empty && !a_re && !a_hold;
        if (a_re) $fdisplay(fo, "AUD %08x", a_data);
    end

    reg [3:0] lo;
    always @(posedge up_strobe) lo = up_data;
    always @(negedge up_strobe) if (!rst) $fdisplay(fo, "UP %02x", {up_data, lo});

    initial begin
        #200 rst = 0; arst = 0;
        #500;
        for (i = 0; i < DN_BYTES; i = i + 1) begin
            if (stim[i][16]) -> hold_ev;
            if (stim[i][15:8]) #(stim[i][15:8] * 1000);
            while (!rdy) #3.17;
            dn[8:1] = stim[i][7:0];
            #9.5 dn[0] = ~dn[0];
            #9.5;
        end
        @(posedge clk) status_req <= 1'b1;
        @(posedge clk) status_req <= 1'b0;
        for (t = 0; t < TAP_LINES; t = t + 1) begin     // a line every 10 us (STATUS takes 5.8), then a burst
            for (j = 0; j < 22; j = j + 1) tap_line[8 * j +: 8] = (t * 11 + j * 3 + 1) & 8'hFF;
            @(posedge clk) tap_req <= 1'b1;
            @(posedge clk) tap_req <= 1'b0;
            if (t < TAP_LINES - 3) #10000;
        end
        #1200000;                                       // past the 1 ms AUD_REQ repeat, with credit unmet
        $fdisplay(fo, "REG 00 %x", own);
        $fdisplay(fo, "REG 01 %x", r_video);
        $fdisplay(fo, "REG 02 %x", r_audio);
        $fdisplay(fo, "REG 03 %x", r_fx);
        $fdisplay(fo, "REG 04 %x", r_comb);
        $fdisplay(fo, "REG 05 %x", r_hud);
        $fdisplay(fo, "REG 06 %x", r_ov_en);
        $fdisplay(fo, "REG 07 %x", r_loader_rst);
        $fdisplay(fo, "REG 08 %x", ir_payload[31:0]);
        $fdisplay(fo, "REG 09 %x", ir_payload[47:32]);
        $fdisplay(fo, "REG 10 %x", r_tap);
        $fdisplay(fo, "REG 11 %x", r_win_pan);
        $fdisplay(fo, "CNT %0d %0d %0d %0d", dn_packets, dn_crc_errs, dn_len_errs, tap_drops);
        $fdisplay(fo, "IRS %0d", ir_sends);
        $fdisplay(fo, "RWGAP %0d", rw_min);
        $fdisplay(fo, "STALL %0d", stall_max);
        $display("flip seen: %0d, ir_send %0d, DN packets %0d, UP packets %0d", flips, ir_send, dn_packets, up_packets);
        $fclose(fo);
        $finish;
    end
    integer flips = 0;
    always @(posedge clk) if (flip) flips = flips + 1;
endmodule
