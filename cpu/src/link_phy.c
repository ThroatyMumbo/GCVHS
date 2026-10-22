// DE1 link transport: pio0 sm0 sends DN packets from two staging buffers by DMA, pio0 sm1 lands UP bytes in a
// ring by an endless DMA.
#include "link_phy.h"

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "link.h"
#include "link_pads.h"
#include "link.pio.h"

#define STB_PIN   2
#define DN_PIN    3                     // 8 data pins
#define UP_STB    11
#define UP_PIN    12                    // 4 data pins
#define RDY_PIN   16                    // link_dn waits on it

#define RING_BITS 14
#define RING_SIZE (1u << RING_BITS)

#define DN_TIMEOUT_US 50000             // no DN progress this long: the DE1 is gone or reloading
#define DN_LOST_US    1000              // the same once a packet was dropped, until STATUS returns

static PIO const pio = pio0;
static const uint sm_dn = 0, sm_up = 1;
static uint off_dn, off_up;
static int dma_dn, dma_up;
static bool dn_lost;

static uint16_t crc_tab[256];
static uint8_t up_ring[RING_SIZE] __attribute__((aligned(RING_SIZE)));
static unsigned rd;
static uint8_t stage[2][4 + 4 + LINK_FB_CHUNK + 2] __attribute__((aligned(4)));
static int stage_cur;

static phy_stats_t stats;

uint16_t phy_crc16(uint16_t crc, const uint8_t *p, unsigned n) {
    while (n--)
        crc = (uint16_t)(crc << 8) ^ crc_tab[(crc >> 8) ^ *p++];
    return crc;
}

// Abandon the packet in flight; the DE1 resyncs on the next LINK_SYNC. Restart on the half of the program
// that matches the strobe's level, so the next byte gets exactly one edge.
static void dn_reset(void) {
    dma_channel_abort(dma_dn);
    pio_sm_set_enabled(pio, sm_dn, false);
    pio_sm_clear_fifos(pio, sm_dn);
    pio_sm_restart(pio, sm_dn);
    pio_sm_exec(pio, sm_dn, pio_encode_jmp(off_dn + (gpio_get(STB_PIN) ? 3 : 0)));
    pio_sm_set_enabled(pio, sm_dn, true);
    stats.dn_drops++;
    dn_lost = true;
}

static bool dn_idle(void) {
    uint32_t t0 = time_us_32(), left = dma_hw->ch[dma_dn].transfer_count;
    while (dma_channel_is_busy(dma_dn)) {
        uint32_t now = time_us_32(), n = dma_hw->ch[dma_dn].transfer_count;
        if (n != left) {
            left = n;
            t0 = now;
        } else if (now - t0 > (dn_lost ? DN_LOST_US : DN_TIMEOUT_US)) {
            dn_reset();
            return false;
        }
    }
    return true;
}

void phy_send(uint8_t type, const void *a, unsigned na, const void *b, unsigned nb) {
    uint8_t *s = stage[stage_cur];
    unsigned len = na + nb;
    s[0] = LINK_SYNC;
    s[1] = type;
    s[2] = (uint8_t)len;
    s[3] = (uint8_t)(len >> 8);
    if (na) memcpy(s + 4, a, na);
    if (nb) memcpy(s + 4 + na, b, nb);
    uint16_t crc = phy_crc16(LINK_CRC_INIT, s + 1, 3 + len);
    s[4 + len] = (uint8_t)crc;
    s[5 + len] = (uint8_t)(crc >> 8);
    dn_idle();
    dma_channel_transfer_from_buffer_now(dma_dn, s, 6 + len);
    stage_cur ^= 1;
    stats.dn_packets++;
    stats.dn_bytes += 6 + len;
}

bool phy_dn_lost(void) {
    return dn_lost;
}

void phy_dn_found(void) {
    dn_lost = false;
}

unsigned phy_up_take(const uint8_t **p) {
    unsigned wr = (dma_hw->ch[dma_up].write_addr - (uintptr_t)up_ring) & (RING_SIZE - 1);
    unsigned n = wr >= rd ? wr - rd : RING_SIZE - rd;
    *p = up_ring + rd;
    rd = (rd + n) & (RING_SIZE - 1);
    stats.up_bytes += n;
    return n;
}

void phy_up_restart(void) {
    pio_sm_set_enabled(pio, sm_up, false);
    dma_channel_abort(dma_up);
    pio_sm_clear_fifos(pio, sm_up);
    pio_sm_restart(pio, sm_up);
    pio_sm_exec(pio, sm_up, pio_encode_jmp(off_up));
    dma_channel_set_write_addr(dma_up, up_ring, false);
    dma_channel_set_trans_count(dma_up, dma_encode_endless_transfer_count(), true);
    rd = 0;
    pio_sm_set_enabled(pio, sm_up, true);
    stats.up_resyncs++;
}

void phy_get_stats(phy_stats_t *out) {
    *out = stats;
}

void link_set_drive(unsigned ma, bool fast) {
    link_pads_drive(STB_PIN, 9, ma, fast);
}

void phy_init(void) {
    for (unsigned n = 0; n < 256; n++) {
        uint16_t c = (uint16_t)(n << 8);
        for (int k = 0; k < 8; k++)
            c = c & 0x8000 ? (uint16_t)(c << 1) ^ LINK_CRC_POLY : (uint16_t)(c << 1);
        crc_tab[n] = c;
    }
    for (uint p = UP_STB; p <= RDY_PIN; p++) {
        gpio_init(p);
        gpio_set_dir(p, GPIO_IN);
    }

    off_dn = pio_add_program(pio, &link_dn_program);
    off_up = pio_add_program(pio, &link_up_program);
    pio_sm_claim(pio, sm_dn);
    pio_sm_claim(pio, sm_up);
    link_dn_init(pio, sm_dn, off_dn, STB_PIN, DN_PIN);
    link_up_init(pio, sm_up, off_up, UP_PIN);
    link_set_drive(2, false);           // 4 mA and up corrupts DN

    dma_dn = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_dn);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm_dn, true));
    dma_channel_configure(dma_dn, &c, &pio->txf[sm_dn], NULL, 0, false);

    // A byte per push, from the top byte lane (the ISR shifts right), so no packet tail waits in the ISR.
    dma_up = dma_claim_unused_channel(true);
    c = dma_channel_get_default_config(dma_up);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, RING_BITS);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm_up, false));
    dma_channel_configure(dma_up, &c, up_ring, (io_rw_8 *)&pio->rxf[sm_up] + 3, dma_encode_endless_transfer_count(),
                          true);

    pio_sm_set_enabled(pio, sm_up, true);
    pio_sm_set_enabled(pio, sm_dn, true);
}
