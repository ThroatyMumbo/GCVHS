create_clock -name CLOCK_50 -period 20.000 [get_ports CLOCK_50]
create_clock -name TD_CLK27 -period 37.037 [get_ports TD_CLK27]
derive_pll_clocks
set sd_clk [get_clocks {clocks|pll|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}]

# SDRAM clock as it leaves the pin, so the pad delay counts against the I/O paths.
create_generated_clock -name clk_dram -source [get_pins {clocks|pll|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}] \
    [get_ports DRAM_CLK]
# VGA_CLK = ~vclk (vga_pll); the ADV7123 latches on its rising edge.
create_generated_clock -name clk_vga -invert \
    -source [get_pins {clocks|vga_pll|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}] [get_ports VGA_CLK]

# The 250 MHz link sampler (lk_pll) and the VGA clock only meet the rest through synchronisers and Gray-coded FIFOs.
set_clock_groups -asynchronous -group [get_clocks {CLOCK_50 clocks|pll|* clk_dram}] -group [get_clocks {clocks|lk_pll|*}] \
    -group [get_clocks {TD_CLK27}] -group [get_clocks {clocks|vga_pll|* clk_vga}] -group [get_clocks altera_reserved_tck]
derive_clock_uncertainty

# IS42S16320D -7 at CL3 (ISSI Rev. B1 AC table): tAC 5.4, tOH 2.7, setup 1.5, hold 0.8 ns; traces 0.1-0.5 ns each.
set dram_out [get_ports {DRAM_ADDR[*] DRAM_BA[*] DRAM_CAS_N DRAM_RAS_N DRAM_WE_N DRAM_CS_N DRAM_CKE DRAM_LDQM DRAM_UDQM DRAM_DQ[*]}]
set_input_delay  -clock clk_dram -max 6.4  [get_ports {DRAM_DQ[*]}]
set_input_delay  -clock clk_dram -min 2.9  [get_ports {DRAM_DQ[*]}]
set_output_delay -clock clk_dram -max 1.9  $dram_out
set_output_delay -clock clk_dram -min -1.2 $dram_out
# Read data from a DRAM_CLK edge is taken on the second sd_clk edge after it (BIST picks lat 5 to match).
set_multicycle_path -setup -end -from [get_clocks clk_dram] -to $sd_clk 2

# ADV7180 (Table 4): data turns over from 2.4 ns before to 3.6 ns after LLC falling, 45:55 duty: 13 ns setup, 14 ns hold.
set_input_delay -clock TD_CLK27 -max 24.0 [get_ports {TD_DATA[*]}]
set_input_delay -clock TD_CLK27 -min 14.2 [get_ports {TD_DATA[*]}]

# ADV7123 at 3.3 V: setup 1.5 ns, hold 2.5 ns.
set vga_dac [get_ports {VGA_R[*] VGA_G[*] VGA_B[*] VGA_BLANK_N}]
set_output_delay -clock clk_vga -max 1.5  $vga_dac
set_output_delay -clock clk_vga -min -2.5 $vga_dac

# Straight to the monitor, or slow and resynchronised inside: nothing to time.
set_false_path -to [get_ports {VGA_HS VGA_VS VGA_SYNC_N LEDR[*] HEX*[*] TD_RESET_N FPGA_I2C_SCLK FPGA_I2C_SDAT AUD_* IRDA_TXD}]
set_false_path -from [get_ports {SW[*] KEY[*] FPGA_I2C_SDAT AUD_ADCDAT IRDA_RXD}]
# Expansion headers: oversampled through synchronisers, or static JTAG-driven test levels.
set_false_path -from [get_ports {GPIO_0[*] GPIO_1[*]}]
set_false_path -to [get_ports {GPIO_0[*]}]
