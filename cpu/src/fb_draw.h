// Drawing into the 320x240 RGB332 framebuffer; callers keep inside it.
#ifndef FB_DRAW_H
#define FB_DRAW_H

#include <stdint.h>
#include <string.h>
#include "video/video.h"

static inline void fill_rect(uint8_t *fb, int x, int y, int w, int h, uint8_t c) {
    for (int j = y; j < y + h; j++)
        memset(fb + j * VIDEO_WIDTH + x, c, (size_t)w);
}

#endif
