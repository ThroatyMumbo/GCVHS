// DE1 link transport: DN packets out of pio0 sm0 by DMA from two staging buffers, UP bytes into
// a ring by an endless DMA from sm1. phy_init on core 0 before core 1 starts; the rest on core 1.
#ifndef LINK_PHY_H
#define LINK_PHY_H

#include <stdbool.h>
#include <stdint.h>

void phy_init(void);

// One DN packet, payload a then b (at most 4 + LINK_FB_CHUNK bytes), started once the previous one has left;
// a DE1 that stops taking bytes has that one dropped.
void phy_send(uint8_t type, const void *a, unsigned na, const void *b, unsigned nb);
bool phy_dn_lost(void);                 // packets dropped since phy_dn_found
void phy_dn_found(void);                // a STATUS came: the DE1 is back, so a stall waits the full timeout again

// UP bytes landed since the last call, n of them contiguous at *p; 0 when there are none.
unsigned phy_up_take(const uint8_t **p);
// From a low strobe: one lost or extra strobe edge would pair every later nibble wrongly.
void phy_up_restart(void);

uint16_t phy_crc16(uint16_t crc, const uint8_t *p, unsigned n);

typedef struct {
    uint32_t dn_packets, dn_bytes, dn_drops, up_resyncs, up_bytes;
} phy_stats_t;
void phy_get_stats(phy_stats_t *out);

#endif
