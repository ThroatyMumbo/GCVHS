// In-line audio (~14.1 kHz) to the WM8731 at 50 MHz / 1024 = 48.83 kHz, I2S master, resampled at a fill-steered rate.
// Left line-in and the CPU stream are mixed on top; right line-in (LTC) goes out on r_in.
module audio_out #(
    parameter [31:0]  STEP0 = 32'd1244282489,  // Q32 input samples per output sample: 236 x 59.94 / 48828.1
    parameter [8:0]   FILL  = 9'd128           // target fill: 9.05 ms at 236 lines a field
) (
    input  wire              clk,              // CLOCK_50
    input  wire              rst,
    input  wire              in_valid,
    input  wire signed [15:0] in_data,
    input  wire              tape_off,
    input  wire              line_off,
    output reg               cpu_re,           // pop one CPU frame
    input  wire              cpu_empty,
    input  wire [31:0]       cpu_data,         // {right, left}
    output reg  [15:0]       cpu_under,        // frames played as silence after CPU audio had started
    output wire              xck,
    output wire              bclk,
    output wire              lrck,
    output reg               dacdat,
    input  wire              adcdat,
    output wire [8:0]        fill,
    output reg  [15:0]       underruns,
    output reg  [15:0]       overruns,
    output wire [15:0]       peak,             // largest |left| over the last second
    output reg               r_valid,          // pulse per codec frame: r_in updated
    output reg  signed [15:0] r_in
);
    reg [9:0] c;                               // one output frame; bit 9 = LRCK (0 left)
    always @(posedge clk) c <= c + 1'b1;
    assign xck  = c[1];
    assign bclk = c[3];
    assign lrck = c[9];
    wire [4:0] bit_n = c[8:4];                 // BCLK period within the channel; MSB in period 1

    reg  [15:0] fifo [0:511];
    reg  [8:0]  wp, rp;
    reg  [15:0] head, fresh_d;
    reg         fresh;                         // the last write went to rp: head read the old word
    assign fill = wp - rp;
    always @(posedge clk) begin
        head    <= fifo[rp];
        fresh   <= !rst && in_valid && fill == 9'd0;
        fresh_d <= in_data;
        if (rst) begin
            wp       <= 9'd0;
            overruns <= 16'd0;
        end else if (in_valid) begin
            if (fill == 9'd511) overruns <= overruns + (overruns != 16'hFFFF);
            else begin
                fifo[wp] <= in_data;
                wp       <= wp + 1'b1;
            end
        end
    end

    // Once per frame (c == 0): step the read phase, pull a sample on carry, then interpolate.
    reg                playing;
    reg  [31:0]        ph, step;
    reg  signed [15:0] x0, x1;
    reg  signed [25:0] fill_f;                 // Q16, time constant 4096 frames (84 ms)
    wire signed [9:0]  err  = $signed({1'b0, fill_f[24:16]}) - $signed({1'b0, FILL});
    wire signed [9:0]  errc = err > 64 ? 10'sd64 : err < -64 ? -10'sd64 : err;
    wire [32:0]        ph_n = ph + step;
    wire signed [25:0] fill_d = $signed({1'b0, fill, 16'd0}) - fill_f;
    always @(posedge clk)
        if (rst) begin
            playing   <= 1'b0;
            rp        <= 9'd0;
            underruns <= 16'd0;
        end else if (c == 10'd0) begin
            fill_f <= fill_f + (fill_d >>> 12);
            step   <= STEP0 + ({{22{errc[9]}}, errc} <<< 18);
            if (!playing) begin
                x0 <= 16'sd0;
                x1 <= 16'sd0;
                ph <= 32'd0;
                if (fill >= FILL) begin
                    playing <= 1'b1;
                    fill_f  <= $signed({1'b0, FILL, 16'd0});
                end
            end else begin
                ph <= ph_n[31:0];
                if (ph_n[32]) begin
                    if (fill == 9'd0) begin
                        playing   <= 1'b0;
                        underruns <= underruns + (underruns != 16'hFFFF);
                    end else begin
                        x0 <= x1;
                        x1 <= fresh ? fresh_d : head;
                        rp <= rp + 1'b1;
                    end
                end
            end
        end

    // c == 2: interpolate; c == 3: DC block (pole at 7.6 Hz); c == 4: mix. All long before period 1.
    reg  signed [15:0] tape;
    reg  signed [31:0] dc_y;                   // Q8
    reg  signed [15:0] dc_x;
    reg  signed [15:0] line_l, out_l, out_r;
    wire signed [33:0] lerp = (x1 - x0) * $signed({1'b0, ph[31:16]});
    function signed [15:0] sat(input signed [17:0] v);
        sat = v > 18'sd32767 ? 16'h7FFF : v < -18'sd32768 ? 16'h8000 : v[15:0];
    endfunction
    reg  signed [15:0] cpu_l, cpu_r;
    reg                cpu_on;
    always @(posedge clk) begin
        cpu_re <= 1'b0;
        if (rst) begin
            cpu_on    <= 1'b0;
            cpu_under <= 16'd0;
        end else if (c == 10'd1) begin
            if (cpu_empty) begin
                {cpu_r, cpu_l} <= 32'd0;
                cpu_under      <= cpu_under + (cpu_on && cpu_under != 16'hFFFF);
            end else begin
                {cpu_r, cpu_l} <= cpu_data;
                cpu_re         <= 1'b1;
                cpu_on         <= 1'b1;
            end
        end
        if (c == 10'd2) tape <= playing && !tape_off ? x0 + (lerp >>> 16) : 16'sd0;
        if (c == 10'd3) begin
            dc_x <= tape;
            dc_y <= dc_y + (($signed({tape, 8'd0}) - $signed({dc_x, 8'd0}))) - (dc_y >>> 10);
        end
        if (c == 10'd4) begin
            out_l <= sat((dc_y >>> 8) + (line_off ? 18'sd0 : line_l) + cpu_l);
            out_r <= sat((dc_y >>> 8) + (line_off ? 18'sd0 : line_l) + cpu_r);
        end
    end

    // I2S: both data lines change on BCLK falling (c[3:0] == 0); ADCDAT is read at c[3:0] == 12 through a
    // 2-flop sync, so the pin is sampled ~10 clocks after that edge.
    reg  [1:0]  adc_s;
    reg  [15:0] adc_sh;
    wire [15:0] word = c[9] ? out_r : out_l;
    always @(posedge clk) begin
        adc_s   <= {adc_s[0], adcdat};
        r_valid <= 1'b0;
        if (c[3:0] == 4'd0) dacdat <= bit_n >= 5'd1 && bit_n <= 5'd16 ? word[5'd16 - bit_n] : 1'b0;
        if (c[3:0] == 4'd12 && bit_n >= 5'd1 && bit_n <= 5'd16) begin
            adc_sh <= {adc_sh[14:0], adc_s[1]};
            if (bit_n == 5'd16) begin
                if (c[9]) begin
                    r_in    <= {adc_sh[14:0], adc_s[1]};
                    r_valid <= 1'b1;
                end else line_l <= {adc_sh[14:0], adc_s[1]};
            end
        end
    end

    peak_meter meter (.clk(clk), .en(c == 10'd5), .x(out_l), .peak(peak));
endmodule
