// The VCR's remote (tools/vcr_ir.json): Panasonic (Kaseikyo) frames, vendor 0x2002, bytes {0x90, 0, cmd, 0x90 ^ cmd}.
#ifndef VCR_H
#define VCR_H

#include <stdint.h>

enum { KEY_STOP = 0, KEY_REW = 2, KEY_FF = 3, KEY_PAUSE = 6, KEY_PLAY = 10, KEY_POWER = 61 };

// The 48-bit payload for link_ir_send.
static inline uint64_t vcr_ir_payload(uint8_t cmd) {
    return 0x2002u | 0x90ull << 16 | (uint64_t)cmd << 32 | (uint64_t)(0x90u ^ cmd) << 40;
}

#endif
