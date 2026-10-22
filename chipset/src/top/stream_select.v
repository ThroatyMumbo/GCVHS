// Which stream is shown and heard: keys, JTAG (VSTA source) or the game CPU, mapped onto the tag's slot layout.
// JTAG {tape off, line-in off, show strip, own audio, audio[2:0], raw, -, use_jtag, flip, mosaic, stream[3:0]} beats the
// keys until a press; mosaic: slots uncropped; flip: stream LSB every frame (stress test); audio follows video unless own.
module stream_select (
    input  wire        clk,             // TD_CLK27
    input  wire [9:0]  sw,
    input  wire [2:0]  key_n,           // KEY[3:1]: combs, raw, next stream
    input  wire [15:0] jtag_src,        // unsynchronised
    input  wire        own,             // game CPU registers, clk
    input  wire [8:0]  r_video,
    input  wire [5:0]  r_audio,         // {expand, show strip, own stream, stream[2:0]}
    input  wire [9:0]  r_fx,
    input  wire        r_comb,
    input  wire        frame_start,
    input  wire        tag_seen,
    input  wire        frozen,
    input  wire [3:0]  tag_scheme,
    input  wire [1:0]  cur_slot,
    input  wire        aud_seen,
    output wire        raw,
    output wire        mosaic,
    output wire        comb_off,
    output wire [9:0]  fx,
    output wire [3:0]  stream,
    output reg  [7:0]  view,            // {slot[1:0], zoom x, right, zoom y, bottom, line pick, parity}
    output wire [1:0]  r_log2,
    output wire [2:0]  aud_nl,          // audio cells: 1 << aud_nl, at most 8
    output wire [2:0]  aud_sel,
    output wire        aud_expand,      // the tape's audio is mu-law companded
    output wire        aud_cells,       // audio cells on the shown stream: mask them out
    output wire        aud_fit          // and stretch the picture right of them
);
    wire [9:0]  fx_s;
    wire [15:0] jtag_s2;
    wire [2:0]  key_s;
    sync2 #(.W(10)) fx_sync (.clk(clk), .d(sw), .q(fx_s));
    sync2 #(.W(16)) jtag_sync (.clk(clk), .d(jtag_src), .q(jtag_s2));
    sync2 #(.W(3)) key_sync (.clk(clk), .d(~key_n), .q(key_s));

    reg  [8:0]  jtag_prev;
    reg  [2:0]  key_prev;
    reg  [16:0] key_tick;
    reg         flip, k_raw, use_jtag;
    reg         k_comb_off;             // KEY[3]; combs on by default: no subcarrier checkerboard, some line mixing
    reg  [3:0]  k_stream;
    wire [2:0]  key_press = key_s & ~key_prev;
    assign      fx        = own ? r_fx : fx_s;
    assign      mosaic    = use_jtag ? jtag_s2[4] : own && r_video[4];
    assign      raw       = use_jtag ? jtag_s2[8] : own ? r_video[8] : k_raw;
    wire        flip_en   = use_jtag ? jtag_s2[5] : own && r_video[5];
    wire [4:0]  aud_bits  = own ? r_audio[4:0] : jtag_s2[13:9];     // {show strip, own stream, stream[2:0]}
    assign      aud_expand = own && r_audio[5];
    assign      comb_off  = own ? r_comb : k_comb_off;
    wire        by_tag    = tag_seen || frozen;     // frozen: the slots still hold the last scheme's fields
    // 1 << t_log2 fields per cycle, 1 << r_log2 regions per field; stream = phase * regions + region, in slot phase.
    wire [1:0]  t_log2    = tag_scheme[3:2];
    wire [1:0]  lay       = tag_scheme[1:0];
    assign      r_log2    = lay == 2'd0 ? 2'd0 : lay == 2'd2 ? 2'd2 : 2'd1;
    wire [4:0]  n_streams = !by_tag ? 5'd2 : 5'd1 << (mosaic ? {1'b0, t_log2} : {1'b0, t_log2} + r_log2);
    // Wrapped into the current mode, so a mode with fewer streams never points at a slot left unwritten.
    wire [3:0]  s_mask    = n_streams[3:0] - 1'b1;
    wire [3:0]  k_cur     = k_stream & s_mask;
    assign      stream    = ((use_jtag ? jtag_s2[3:0] : own ? r_video[3:0] : k_stream) ^ {3'b0, flip_en & flip})
                            & s_mask;
    wire [3:0]  phase     = stream >> r_log2;
    wire [1:0]  region    = stream[1:0] & ~(2'b11 << r_log2);
    always @(posedge clk) begin
        jtag_prev <= jtag_s2[8:0];
        if (jtag_s2[8:0] != jtag_prev) use_jtag <= jtag_s2[6];
        else if (&key_tick && |key_press) use_jtag <= 1'b0;
        key_tick           <= key_tick + 1'b1;
        if (frame_start) flip <= ~flip;
        if (&key_tick) begin            // 206 Hz: slower than any key bounce
            key_prev <= key_s;
            if (key_press[0]) k_stream <= k_cur + 1'b1;
            if (key_press[1]) k_raw <= ~k_raw;
            if (key_press[2]) k_comb_off <= ~k_comb_off;
        end
    end

    always @* begin
        view = 8'd0;
        if (raw) view[7:6] = cur_slot;
        else if (mosaic || !by_tag) view[7:6] = stream[1:0];
        else case (lay)
            2'd1:    view = {phase[1:0], 1'b1, region[0], 4'd0};
            2'd2:    view = {phase[1:0], 1'b1, region[0], 1'b1, region[1], 2'd0};
            2'd3:    view = {phase[1:0], 4'd0, 1'b1, region[0]};
            default: view = {phase[1:0], 6'd0};
        endcase
    end

    // One audio cell per video stream, at most 8; streams 8-15 share the audio of 0-7.
    assign aud_nl    = {1'b0, t_log2} + r_log2 > 3'd3 ? 3'd3 : {1'b0, t_log2} + r_log2;
    assign aud_sel   = (aud_bits[3] ? aud_bits[2:0] : stream[2:0]) & ~(3'b111 << aud_nl);
    assign aud_cells = aud_seen && !aud_bits[4] && !raw && !mosaic;
    assign aud_fit   = aud_cells && by_tag;
endmodule
