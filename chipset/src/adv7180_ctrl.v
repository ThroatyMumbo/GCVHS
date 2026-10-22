// Bring up the on-board ADV7180: pulse reset, write the DE1-SoC init table, then poll Status 1 (0x10).
// The WM8731 codec on the same bus is set up after it: line in to the ADC, DAC out, I2S slave.
module adv7180_ctrl (
    input  wire       clk,              // 50 MHz
    input  wire       rst,
    input  wire       comb_off,         // NTSC luma and chroma combs off: no mixing between lines; live
    output reg        td_reset_n,
    output wire       scl,
    inout  wire       sda,
    output reg        cfg_done,
    output reg        i2c_err,          // sticky: some transaction was NACKed
    output reg        codec_err,        // ... one to the WM8731
    output reg  [7:0] status1           // [0] IN_LOCK, [2] FSC_LOCK, [6:4] AD_RESULT (detected standard)
);
    localparam [6:0]  ADDR   = 7'h20;   // 0x40 in 8-bit write-address form
    localparam integer MS    = 50_000;
    localparam [6:0]  CODEC  = 7'h1A;
    localparam [5:0]  N_ADV  = 6'd47;
    localparam [5:0]  N_REGS = 6'd58;
    localparam [5:0]  I_COMB = 6'd29;

    localparam [2:0] S_RST_LO = 3'd0, S_RST_WAIT = 3'd1, S_WRITE = 3'd2, S_WRITE_WAIT = 3'd3,
                     S_POLL_DELAY = 3'd4, S_POLL = 3'd5, S_POLL_WAIT = 3'd6;

    reg  [2:0]  state;
    reg  [23:0] timer;
    reg  [5:0]  idx;
    reg  [15:0] entry;                  // {register, value}
    reg         i2c_start, i2c_read;
    reg         comb_now;               // what register 0x38 holds, or is being written with
    reg         single;                 // writing only entry idx, then back to polling
    wire        i2c_busy, i2c_done, i2c_nack;
    wire [7:0]  i2c_rdata;

    // Copied from Intel's altera_up_av_config_auto_init_ob_adv7180.v, which the DE1-SoC config uses.
    always @* case (idx)
        6'd0:  entry = 16'h0000;        // input control: autodetect standard, CVBS on AIN1
        6'd1:  entry = 16'h01C8;
        6'd2:  entry = 16'h030C;        // output: 8-bit 4:2:2 BT.656
        6'd3:  entry = 16'h0445;
        6'd4:  entry = 16'h077F;        // autodetect all standards
        6'd5:  entry = 16'h0A00;
        6'd6:  entry = 16'h0B00;
        6'd7:  entry = 16'h0C36;
        6'd8:  entry = 16'h0D7C;
        6'd9:  entry = 16'h0F00;        // power up
        6'd10: entry = 16'h1412;
        6'd11: entry = 16'h1500;
        6'd12: entry = 16'h1701;
        6'd13: entry = 16'h1893;
        6'd14: entry = 16'h19F1;
        6'd15: entry = 16'h2758;
        6'd16: entry = 16'h2BE1;
        6'd17: entry = 16'h2CAE;
        6'd18: entry = 16'h2DF4;
        6'd19: entry = 16'h2E00;
        6'd20: entry = 16'h2FF0;
        6'd21: entry = 16'h3000;
        6'd22: entry = 16'h3112;
        6'd23: entry = 16'h3241;
        6'd24: entry = 16'h3384;
        6'd25: entry = 16'h3400;
        6'd26: entry = 16'h3502;
        6'd27: entry = 16'h3600;
        6'd28: entry = 16'h3701;
        I_COMB: entry = comb_now ? 16'h38A4 : 16'h3880; // NTSC comb: CCMN, YCMN 100 = disabled
        6'd30: entry = 16'h39C0;
        6'd31: entry = 16'h3A10;
        6'd32: entry = 16'h3DB2;
        6'd33: entry = 16'h4101;
        6'd34: entry = 16'hB21C;
        6'd35: entry = 16'hC300;
        6'd36: entry = 16'hC400;
        6'd37: entry = 16'hDCAC;
        6'd38: entry = 16'hDD4C;
        6'd39: entry = 16'hE525;        // NTSC V bit begin/end, F bit toggle
        6'd40: entry = 16'hE604;
        6'd41: entry = 16'hE763;
        6'd42: entry = 16'hE865;        // PAL V bit begin/end, F bit toggle
        6'd43: entry = 16'hE914;
        6'd44: entry = 16'hEA63;
        6'd45: entry = 16'hEB55;
        6'd46: entry = 16'hEC55;
        // WM8731, {register[6:0], value[8:0]}
        6'd47: entry = 16'h1E00;        // reset
        6'd48: entry = 16'h0C62;        // power: all on but mic, oscillator, CLKOUT
        6'd49: entry = 16'h0017;        // left line in 0 dB, unmuted
        6'd50: entry = 16'h0217;        // right line in
        6'd51: entry = 16'h0479;        // left headphone 0 dB
        6'd52: entry = 16'h0679;        // right headphone
        6'd53: entry = 16'h0812;        // analogue path: DAC selected, line in to ADC, mic muted
        6'd54: entry = 16'h0A00;        // digital path: DAC unmuted, no de-emphasis
        6'd55: entry = 16'h0E02;        // interface: I2S, 16 bit, slave
        6'd56: entry = 16'h1000;        // sampling: normal mode, 256 fs
        6'd57: entry = 16'h1201;        // active
        default: entry = 16'h0000;
    endcase

    i2c_master i2c (
        .clk(clk), .rst(rst),
        .start(i2c_start), .read(i2c_read), .dev(idx >= N_ADV && !i2c_read ? CODEC : ADDR),
        .reg_addr(i2c_read ? 8'h10 : entry[15:8]), .wdata(entry[7:0]),
        .busy(i2c_busy), .done(i2c_done), .nack(i2c_nack), .rdata(i2c_rdata),
        .scl(scl), .sda(sda)
    );

    always @(posedge clk) begin
        i2c_start <= 1'b0;
        if (rst) begin
            state      <= S_RST_LO;
            timer      <= 24'd0;
            td_reset_n <= 1'b0;
            cfg_done   <= 1'b0;
            i2c_err    <= 1'b0;
            codec_err  <= 1'b0;
            status1    <= 8'd0;
            idx        <= 6'd0;
            i2c_read   <= 1'b0;
            comb_now   <= comb_off;
            single     <= 1'b0;
        end else begin
            if (i2c_done && i2c_nack) i2c_err <= 1'b1;
            if (i2c_done && i2c_nack && idx >= N_ADV && !i2c_read) codec_err <= 1'b1;
            case (state)
                S_RST_LO:
                    if (timer == 10 * MS) begin
                        timer      <= 24'd0;
                        td_reset_n <= 1'b1;
                        state      <= S_RST_WAIT;
                    end else timer <= timer + 1'b1;
                S_RST_WAIT:
                    if (timer == 20 * MS) begin
                        timer <= 24'd0;
                        state <= S_WRITE;
                    end else timer <= timer + 1'b1;
                S_WRITE: begin
                    i2c_start <= 1'b1;
                    i2c_read  <= 1'b0;
                    state     <= S_WRITE_WAIT;
                end
                S_WRITE_WAIT:
                    if (i2c_done) begin
                        if (single || idx == N_REGS - 1'b1) begin
                            cfg_done <= 1'b1;
                            single   <= 1'b0;
                            state    <= S_POLL_DELAY;
                        end else begin
                            idx   <= idx + 1'b1;
                            state <= S_WRITE;
                        end
                    end
                S_POLL_DELAY:
                    if (comb_off != comb_now) begin
                        comb_now <= comb_off;
                        idx      <= I_COMB;
                        single   <= 1'b1;
                        state    <= S_WRITE;
                    end else if (timer == 100 * MS) begin
                        timer <= 24'd0;
                        state <= S_POLL;
                    end else timer <= timer + 1'b1;
                S_POLL: begin
                    i2c_start <= 1'b1;
                    i2c_read  <= 1'b1;
                    state     <= S_POLL_WAIT;
                end
                S_POLL_WAIT:
                    if (i2c_done) begin
                        if (!i2c_nack) status1 <= i2c_rdata;
                        state <= S_POLL_DELAY;
                    end
                default: state <= S_RST_LO;
            endcase
        end
    end
endmodule
