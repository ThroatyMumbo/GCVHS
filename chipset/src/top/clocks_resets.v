// PLLs off CLOCK_50 (VGA, SDRAM/link, link sampler), KEY[0] resets per domain, the reset count and a 1 ms tick.
module clocks_resets (
    input  wire       clk50,
    input  wire       clk27,            // TD_CLK27
    input  wire       key0_n,
    output wire       vclk,             // 27 MHz output clock
    output wire       sd_clk,           // 100 MHz: SDRAM and game CPU link
    output wire       dram_clk,
    output wire       lk_clk,           // 250 MHz: link sampler
    output wire       rst50,
    output wire       rst27,
    output wire       rst_sd,
    output reg  [7:0] key_resets = 8'd0,
    output wire       ms_tick           // clk50, one cycle per ms
);
    sync2 rst50_sync (.clk(clk50), .d(~key0_n), .q(rst50));
    sync2 rst27_sync (.clk(clk27), .d(~key0_n), .q(rst27));

    // The output's own clock: TD_CLK27 stalls as the ADV7180 finds or loses a deck, and a monitor (or the
    // Magewell) re-detects the mode on the bent sync.
    altera_pll #(
        .fractional_vco_multiplier("false"),
        .reference_clock_frequency("50.0 MHz"),
        .operation_mode("direct"),
        .number_of_clocks(1),
        .output_clock_frequency0("27.000000 MHz"),
        .phase_shift0("0 ps"),
        .duty_cycle0(50),
        .pll_type("General"),
        .pll_subtype("General")
    ) vga_pll (
        .refclk(clk50), .rst(1'b0), .outclk(vclk), .locked(), .fboutclk(), .fbclk(1'b0)
    );

    // DRAM_CLK leads the controller clock by 4 ns (6 ns lag): balances read setup against write setup in
    // video.sdc at the slow corner.
    wire pll_locked;
    altera_pll #(
        .fractional_vco_multiplier("false"),
        .reference_clock_frequency("50.0 MHz"),
        .operation_mode("direct"),
        .number_of_clocks(2),
        .output_clock_frequency0("100.000000 MHz"),
        .phase_shift0("0 ps"),
        .duty_cycle0(50),
        .output_clock_frequency1("100.000000 MHz"),
        .phase_shift1("6000 ps"),
        .duty_cycle1(50),
        .pll_type("General"),
        .pll_subtype("General")
    ) pll (
        .refclk(clk50), .rst(1'b0), .outclk({dram_clk, sd_clk}), .locked(pll_locked),
        .fboutclk(), .fbclk(1'b0)
    );
    sync2 rst_sd_sync (.clk(sd_clk), .d(~key0_n | ~pll_locked), .q(rst_sd));

    altera_pll #(
        .fractional_vco_multiplier("false"),
        .reference_clock_frequency("50.0 MHz"),
        .operation_mode("direct"),
        .number_of_clocks(1),
        .output_clock_frequency0("250.000000 MHz"),
        .phase_shift0("0 ps"),
        .duty_cycle0(50),
        .pll_type("General"),
        .pll_subtype("General")
    ) lk_pll (
        .refclk(clk50), .rst(1'b0), .outclk(lk_clk), .locked(), .fboutclk(), .fbclk(1'b0)
    );

    // KEY[0] presses since configuration, 1-255 (never back to 0): STATUS byte 60, the main CPU reboots on a change.
    reg [2:0] key0_sd = 3'd0;
    always @(posedge sd_clk) begin
        key0_sd <= {key0_sd[1:0], ~key0_n};
        if (key0_sd[1] && !key0_sd[2]) key_resets <= key_resets == 8'd255 ? 8'd1 : key_resets + 1'b1;
    end

    localparam [15:0] MS_CYCLES = 16'd50000;    // CLOCK_50 cycles per ms
    reg  [15:0] ms50;
    always @(posedge clk50) ms50 <= ms50 == MS_CYCLES - 1'b1 ? 16'd0 : ms50 + 1'b1;
    assign ms_tick = ms50 == 16'd0;
endmodule
