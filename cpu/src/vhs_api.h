// The VHS console's game API: GCV2's console_api_t unchanged, then VHS slots, so GCV2 games run as they
// are and VHS games cast the pointer they get.
#ifndef VHS_API_H
#define VHS_API_H

#include <stdbool.h>
#include <stdint.h>
#include "console_api.h"
#include "link_proto.h"

#define VHS_API_MAGIC 0x31534856u   // "VHS1"
#define VHS_PSRAM_GAME 0x11400000u  // PSRAM from 4 MiB up is the game's; the BIOS's tape copy
                                    // (at 2 MiB, <= 65535 slots x 17 bytes) ends below it

typedef struct {
    console_api_t gcv2;
    uint32_t magic;                                         // VHS_API_MAGIC
    bool (*status)(vhs_status_t *out);                      // latest DE1 STATUS; false before the first
    void (*reg_write)(uint8_t addr, uint32_t val);          // REG_* (link_proto.h)
    void (*ir_send)(uint64_t payload);                      // 48-bit Panasonic frame to the VCR
    void (*palette)(uint8_t first, unsigned count, const uint8_t (*rgbt)[4]);  // {r, g, b, transparent}
    // Framebuffer pixels, x -20 to 339 reaching the output's side borders; w or h <= 0 hides the video.
    void (*video_window)(int x, int y, int w, int h);
    void (*video_fullscreen)(void);                         // the whole 720x480 output, as at launch
    // Every in-line audio cell of each tape line, on core 1 (link.h link_set_tap); NULL stops it.
    void (*tape_tap)(void (*fn)(const vhs_tap_t *line));
} vhs_api_t;

#endif
