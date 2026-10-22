// GCV2's video.h on the DE1 overlay: the game draws a 320x240 RGB332 frame here and vsync presents it
// over the link (link_present), returning once the DE1 shows it.
#include "video_link.h"

#include "video/video.h"
#include "link.h"

static uint8_t fb[VIDEO_WIDTH * VIDEO_HEIGHT];
static video_mode_t mode = VIDEO_MODE_SRAM60;

_Static_assert(VIDEO_WIDTH == LINK_FB_W && VIDEO_HEIGHT == LINK_FB_H, "overlay and GCV2 frame sizes differ");

void video_init(video_mode_t m) {
    mode = m;
}

uint8_t *video_get_framebuffer(void) {
    return fb;
}

void video_wait_vsync(void) {
    link_present(fb, 100);
}

video_mode_t video_get_mode(void) {
    return mode;
}

// Output pixels, clamped to the raster; the steps come from the clamped size. Returns the registers set in r.
static unsigned window_regs(link_reg_t *r, int x, int y, int w, int h) {
    if (x < 0) w += x, x = 0;
    if (y < 0) h += y, y = 0;
    if (x + w > LINK_OUT_W) w = LINK_OUT_W - x;
    if (y + h > LINK_OUT_H) h = LINK_OUT_H - y;
    if (w < 46 || h < 16)
        w = h = 0;
    r[0] = (link_reg_t){REG_WIN_POS, (uint32_t)y << 16 | (uint32_t)x};
    r[1] = (link_reg_t){REG_WIN_SIZE, (uint32_t)h << 16 | (uint32_t)w};
    if (!w)
        return 2;
    r[2] = (link_reg_t){REG_WIN_HSTEP, (uint32_t)(((uint64_t)LINK_OUT_W << 16) / (uint32_t)w)};
    r[3] = (link_reg_t){REG_WIN_VSTEP, (uint32_t)(((uint64_t)240 << 16) / (uint32_t)h)};
    return 4;
}

void video_window(int x, int y, int w, int h) {
    link_reg_t r[4];
    unsigned n = w <= 0 || h <= 0 ? window_regs(r, 0, 0, 0, 0)
                                  : window_regs(r, LINK_FB_X0 + 2 * x, 2 * y, 2 * w, 2 * h);
    link_regs_write(r, n);
}

void video_fullscreen(void) {
    link_reg_t r[5];
    unsigned n = window_regs(r, 0, 0, LINK_OUT_W, LINK_OUT_H);
    r[n++] = (link_reg_t){REG_WIN_PAN, 0};
    link_regs_write(r, n);
}

// The overlay's reset columns: 320 framebuffer pixels doubled into output x 40-679.
void video_overlay_default(void) {
    link_regs_write((const link_reg_t[]){{REG_OV_HWIN, REG_OV_HWIN_RESET}, {REG_OV_HSTEP, REG_OV_HSTEP_RESET}}, 2);
}

// The overlay's reset palette (overlay.v): RGB332 with each field's bits repeated, index 0 transparent.
void video_palette_reset(void) {
    static uint8_t pal[256][4];
    for (int k = 0; k < 256; k++) {
        unsigned r = k >> 5, g = k >> 2 & 7, b = k & 3;
        pal[k][0] = (uint8_t)(r << 5 | r << 2 | r >> 1);
        pal[k][1] = (uint8_t)(g << 5 | g << 2 | g >> 1);
        pal[k][2] = (uint8_t)(b * 0x55);
        pal[k][3] = k == 0;
    }
    link_palette(0, 256, (const uint8_t (*)[4])pal);
}
