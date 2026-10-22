// Link test to the DE1 (chipset link_rx): PRBS-7 words on GP3-GP9 with a strobe on GP2, forever.
// Rate is set by writing the PIO clock divider over SWD (see ctl below); the words never stop.
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "link_pads.h"
#include "link_tx.pio.h"

#define STB_PIN  2
#define DATA_PIN 3

// 4 words per entry: 508 words is exactly 4 PRBS-7 periods, so the table loops seamlessly.
static uint32_t prbs[127];
static const uint32_t *prbs_start = prbs;

// Set over SWD: div is the PIO divider in 16.8 fixed point, drive in mA (2/4/8/12), fast slew; seq applies.
volatile struct {
    uint32_t seq, div, drive_ma, fast;
    uint32_t applied;
} ctl = { 0, 1 << 8, 4, 0, 0 };

static void apply(PIO pio, uint sm) {
    uint32_t div = ctl.div < (1 << 8) ? (1 << 8) : ctl.div;
    pio_sm_set_clkdiv_int_frac8(pio, sm, div >> 8, div & 0xff);
    pio_sm_clkdiv_restart(pio, sm);
    link_pads_drive(STB_PIN, DATA_PIN + 7 - STB_PIN, ctl.drive_ma, ctl.fast);
    ctl.applied = ctl.seq;
}

int main(void) {
    uint8_t s = 1;
    for (uint i = 0; i < 127; i++) {
        uint32_t w = 0;
        for (uint b = 0; b < 4; b++) {
            s = ((s << 1) | (((s >> 6) ^ (s >> 5)) & 1)) & 0x7f;
            w |= (uint32_t)s << (8 * b);
        }
        prbs[i] = w;
    }

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &link_tx_program);
    link_tx_init(pio, sm, offset, STB_PIN, DATA_PIN);
    apply(pio, sm);

    int ch_data = dma_claim_unused_channel(true);
    int ch_ctrl = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(ch_ctrl);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(ch_ctrl, &c, &dma_hw->ch[ch_data].al3_read_addr_trig, &prbs_start, 1, false);

    c = dma_channel_get_default_config(ch_data);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&c, ch_ctrl);
    dma_channel_configure(ch_data, &c, &pio->txf[sm], prbs, 127, true);

    pio_sm_set_enabled(pio, sm, true);
    for (;;) {
        if (ctl.seq != ctl.applied) apply(pio, sm);
        tight_loop_contents();
    }
}
