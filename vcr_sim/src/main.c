// Loops one pre-rendered 240p NTSC field (tools/pattern.py) out of the 6-bit R-2R DAC on GPIO2..7.
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/vreg.h"
#include "tvout.pio.h"

#define DAC_BASE_PIN 2
#define DAC_BITS 6
#define SYS_KHZ 315000
#define PIO_DIV 11 // 315 MHz / (2 instr * 11) = 14.31818 MHz = 910 samples per 63.556 us line

extern const uint32_t field_wave[];
extern const uint32_t field_wave_words;

static const uint32_t *field_ptr = field_wave;

static void dac_pio_init(PIO pio, uint sm) {
    uint off = pio_add_program(pio, &tvout_program);
    for (uint i = 0; i < DAC_BITS; i++) {
        pio_gpio_init(pio, DAC_BASE_PIN + i);
        gpio_set_drive_strength(DAC_BASE_PIN + i, GPIO_DRIVE_STRENGTH_12MA);
    }
    pio_sm_set_consecutive_pindirs(pio, sm, DAC_BASE_PIN, DAC_BITS, true);

    pio_sm_config c = tvout_program_get_default_config(off);
    sm_config_set_out_pins(&c, DAC_BASE_PIN, DAC_BITS);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv_int_frac8(&c, PIO_DIV, 0);
    pio_sm_init(pio, sm, off, &c);
}

// data streams the whole field to the PIO, then chains to ctrl, which re-arms data at the field start.
static void dac_dma_start(PIO pio, uint sm) {
    uint data = dma_claim_unused_channel(true);
    uint ctrl = dma_claim_unused_channel(true);

    dma_channel_config dc = dma_channel_get_default_config(data);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, true);
    channel_config_set_write_increment(&dc, false);
    channel_config_set_dreq(&dc, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&dc, ctrl);
    channel_config_set_high_priority(&dc, true);
    dma_channel_configure(data, &dc, &pio->txf[sm], field_wave, field_wave_words, false);

    dma_channel_config cc = dma_channel_get_default_config(ctrl);
    channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
    channel_config_set_read_increment(&cc, false);
    channel_config_set_write_increment(&cc, false);
    dma_channel_configure(ctrl, &cc, &dma_hw->ch[data].al3_read_addr_trig, &field_ptr, 1, false);

    dma_channel_start(data);
    pio_sm_set_enabled(pio, sm, true);
}

int main(void) {
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    sleep_ms(3);
    set_sys_clock_khz(SYS_KHZ, true);

    dac_pio_init(pio0, 0);
    dac_dma_start(pio0, 0);

    while (true)
        tight_loop_contents();
}
