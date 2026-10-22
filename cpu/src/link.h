// DE1 link driver. Core 1 runs link_core1_main() and owns both PIO state machines
// and DMA channels; core 0 talks to it through the calls below.
#ifndef LINK_H
#define LINK_H

#include <stdbool.h>
#include <stdint.h>
#include "link_proto.h"

typedef void (*link_audio_fill_fn)(int16_t *stereo_dst, unsigned frames);

typedef struct {
    uint32_t up_packets, up_crc_errs, up_bytes;
    uint32_t dn_packets, dn_bytes;
    uint32_t aud_frames, presents;
    uint32_t dn_drops;                      // DN packets abandoned with RDY held low
    uint32_t up_resyncs;                    // UP receiver restarts after STATUS stopped
} link_stats_t;

void link_init(void);                       // core 0, before core 1 starts
void link_core1_main(void);                 // never returns

// Latest STATUS; false until one has arrived. seq counts them.
bool link_status(vhs_status_t *out, uint32_t *seq);
bool link_wait_status(vhs_status_t *out, uint32_t timeout_ms);  // the next one after the call
uint32_t link_status_age_ms(void);          // since the last STATUS; UINT32_MAX before the first

void link_reg_write(uint8_t addr, uint32_t val);
void link_ir_send(uint64_t payload);        // a 48-bit Panasonic frame out of the DE1's IR emitter

// Up to LINK_REG_MAX registers in one REG_WR: the DE1 applies them together, so no frame sees half of them.
typedef struct {
    uint8_t addr;
    uint32_t val;
} link_reg_t;
void link_regs_write(const link_reg_t *regs, unsigned n);

void link_palette(uint8_t first, unsigned count, const uint8_t (*rgbt)[4]);  // {r, g, b, transparent}

// Sends the frame and a FLIP, then waits until the DE1 shows it (or timeout_ms); fb is free on return.
bool link_present(const uint8_t *fb, uint32_t timeout_ms);

// Tape slots first..first+count-1 into dst, d bytes each (d from STATUS). True if all arrived in time.
bool link_img_read(uint16_t first, uint16_t count, uint8_t *dst, unsigned d, uint32_t timeout_ms);

int link_echo(const uint8_t *buf, unsigned len, uint32_t timeout_ms);    // 1 match, -1 mismatch, 0 timeout

// Called on core 1 whenever the DE1 wants samples (48.83 kHz stereo); NULL sends none.
void link_set_audio_fill(link_audio_fill_fn fill);

// Called on core 1 with every TAPE_AUD line; setting it turns REG_TAP on, NULL turns it off.
typedef void (*link_tap_fn)(const vhs_tap_t *line);
void link_set_tap(link_tap_fn fn);

// Called on core 1 when the DE1's KEY[0] count changes, after the first STATUS; NULL ignores it.
typedef void (*link_key_reset_fn)(void);
void link_set_key_reset(link_key_reset_fn fn);

void link_get_stats(link_stats_t *out);

void link_set_drive(unsigned ma, bool fast);    // DN pads: 2, 4, 8 or 12 mA, fast or slow slew

#endif
