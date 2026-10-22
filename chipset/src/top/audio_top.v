// In-line audio: one sample per stored line, sliced at 27 MHz, played by the codec at 50 MHz / 1024 with its
// left line in and the game CPU's audio mixed on top.
module audio_top (
    input  wire               clk27,    // TD_CLK27
    input  wire               rst27,
    input  wire               clk50,
    input  wire               rst50,
    input  wire               st_sav,
    input  wire               st_y_en,
    input  wire [7:0]         st_y,
    input  wire               st_eav,
    input  wire               field_start,
    input  wire               frozen,
    input  wire [1:0]         n_log2,
    input  wire [2:0]         sel,
    input  wire               expand,
    input  wire               own50,
    input  wire [1:0]         cpu_off,  // clk50: {tape off, line-in off}
    input  wire [1:0]         jtag_off, // unsynchronised, used while the CPU doesn't own the controls
    input  wire               dt_active,
    output wire               cpu_re,
    input  wire               cpu_empty,
    input  wire [31:0]        cpu_data,
    output wire [15:0]        cpu_under,
    output wire               xck,
    output wire               bclk,
    output wire               lrck,
    output wire               dacdat,
    input  wire               adcdat,
    output reg                aud_seen, // the stored fields carry audio cells
    output wire [15:0]        rate,
    output wire [15:0]        no_pilot,
    output wire [10:0]        cell0,
    output wire [11:0]        pilot,
    output wire               line_v,   // TAPE_AUD tap: a line with a pilot, and its cells
    output wire [87:0]        cells,
    output wire [8:0]         fill,
    output wire [15:0]        under,
    output wire [15:0]        over,
    output wire [15:0]        peak,
    output wire               r_valid,  // right line in (LTC), clk50
    output wire [15:0]        r_in
);
    wire               valid, ftick, fgood;
    wire signed [15:0] sample;
    audio_slicer slicer (
        .clk(clk27), .rst(rst27), .start(st_sav), .y_en(st_y_en), .y(st_y), .done(st_eav),
        .field(field_start),
        .n_log2(n_log2), .sel(sel), .expand(expand), .valid(valid), .sample(sample),
        .rate(rate), .no_pilot(no_pilot), .field_tick(ftick), .field_good(fgood),
        .cell0(cell0), .pilot(pilot), .line_v(line_v), .cells(cells)
    );
    // White picture detail fakes a pilot on some lines: need nearly every line two fields running; four bad fields drop it.
    // Frozen, the stored fields keep the cell layout they were recorded with.
    reg  [1:0] good_n, bad_n;
    always @(posedge clk27)
        if (!frozen && ftick) begin
            good_n <= fgood ? good_n + (good_n != 2'd3) : 2'd0;
            bad_n  <= fgood ? 2'd0 : bad_n + (bad_n != 2'd3);
            if (fgood && good_n != 2'd0) aud_seen <= 1'b1;
            if (!fgood && bad_n == 2'd3) aud_seen <= 1'b0;
        end

    wire        in_valid;
    wire [15:0] in_data;
    req_cdc #(.W(16)) aud_cdc (
        .sclk(clk27), .sreq(valid), .sdata(frozen ? 16'd0 : sample),
        .dclk(clk50), .dreq(in_valid), .ddata(in_data)
    );

    wire [1:0]  off_s;                  // tape off, line-in off
    wire        dt_mute;                // data fields: their cells would play as noise
    sync2 #(.W(3)) off_sync (.clk(clk50), .d({own50 ? cpu_off : jtag_off, dt_active}), .q({off_s, dt_mute}));
    audio_out aout (
        .clk(clk50), .rst(rst50), .in_valid(in_valid), .in_data(in_data),
        .tape_off(off_s[1] | dt_mute), .line_off(off_s[0]),
        .cpu_re(cpu_re), .cpu_empty(cpu_empty), .cpu_data(cpu_data), .cpu_under(cpu_under),
        .xck(xck), .bclk(bclk), .lrck(lrck), .dacdat(dacdat), .adcdat(adcdat),
        .fill(fill), .underruns(under), .overruns(over), .peak(peak),
        .r_valid(r_valid), .r_in(r_in)
    );
endmodule
