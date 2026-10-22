// The tape screen's drawing, over the deck's picture, and the DE1 setup it draws on.
#include <stdio.h>
#include <string.h>
#include "engine/engine.h"
#include "fb_draw.h"
#include "tape_int.h"
#include "tape_ui.h"
#include "video_link.h"

#define C_CLEAR  0                      // transparent: the deck's picture, or black with it hidden
#define C_EDGE   1                      // text outline: opaque black here (edge_black), as index 0 is transparent
#define C_PANEL  RGB332(0, 64, 160)
#define C_TEXT   RGB332(255, 255, 255)
#define C_DIM    RGB332(192, 192, 224)
#define C_TITLE  RGB332(255, 224, 0)
#define C_ERR    RGB332(255, 96, 64)
#define C_SEL    RGB332(255, 224, 0)
#define C_HIT    RGB332(0, 255, 0)
#define C_BAR    RGB332(64, 64, 96)

#define Y_OSD    16                     // deck mode and tape counter
#define Y_TITLE  150
#define Y_SUB    174                    // the line under the title, or the load bar
#define Y_MENU   204
#define BAR_X    40
#define BAR_W    240

static const char *const menu_label[M_N] = {"POWER", "REW", "PLAY", "PAUSE", "FF", "STOP", "LOAD GAME"};

static void box(uint8_t *fb, int x, int y, int w, int h, uint8_t c) {
    fill_rect(fb, x, y, w, 1, c);
    fill_rect(fb, x, y + h - 1, w, 1, c);
    fill_rect(fb, x, y, 1, h, c);
    fill_rect(fb, x + w - 1, y, 1, h, c);
}

static void clip_rect(uint8_t *fb, int x, int y, int w, int h, uint8_t c) {
    if (x < 0) w += x, x = 0;
    if (y < 0) h += y, y = 0;
    if (x + w > VIDEO_WIDTH) w = VIDEO_WIDTH - x;
    if (y + h > VIDEO_HEIGHT) h = VIDEO_HEIGHT - y;
    if (w > 0 && h > 0) fill_rect(fb, x, y, w, h, c);
}

// Outlined like a VCR's on-screen display, so it reads over any picture; rendered at 1x off-screen, then scaled.
static void text(uint8_t *fb, int x, int y, const char *s, uint8_t c, int scale) {
    static uint8_t glyphs[8 * VIDEO_WIDTH];
    int w = font_string_width(s);
    if (w > VIDEO_WIDTH) w = VIDEO_WIDTH;
    memset(glyphs, 0, sizeof glyphs);
    font_draw_string(glyphs, 0, 0, s, 1);
    for (int pass = 0; pass < 2; pass++)
        for (int j = 0; j < 8; j++)
            for (int i = 0; i < w; i++)
                if (glyphs[j * VIDEO_WIDTH + i]) {
                    int px = x + i * scale, py = y + j * scale;
                    if (pass == 0) clip_rect(fb, px - 1, py - 1, scale + 2, scale + 2, C_EDGE);
                    else clip_rect(fb, px, py, scale, scale, c);
                }
}

static void center(uint8_t *fb, int y, const char *s, uint8_t c) {
    text(fb, (VIDEO_WIDTH - font_string_width(s)) / 2, y, s, c, 1);
}

static void center2(uint8_t *fb, int y, const char *s, uint8_t c) {
    int w = font_string_width(s);
    if (2 * w > VIDEO_WIDTH - 8) center(fb, y + 4, s, c);
    else text(fb, (VIDEO_WIDTH - 2 * w) / 2, y, s, c, 2);
}

static const char *deck_label(const vhs_status_t *s) {
    if (ui.state == T_OFF) return "OFF";
    if (ui.unlock) return "";
    if (s->flags1 & ST1_BLUE) {         // winding looks like stop: go by the last key and how long it can take
        if (ui.last_key == KEY_REW && ui.key_age < tape_wind_frames(ui.last_pos)) return "REWIND";
        if (ui.last_key == KEY_FF && ui.key_age < FF_SHOWN) return "FAST FWD";
        return "STOP";
    }
    if (tape_trick(s)) return !tape_searching() ? "PAUSE" : ui.ptag_dir > 0 ? "SEARCH >>" : "SEARCH <<";
    return "PLAY";
}

// Deck mode top left, tape counter top right, where a VCR puts them.
static void draw_osd(uint8_t *fb, const vhs_status_t *s) {
    char line[16];
    int32_t pos = tape_pos(s);
    if (pos >= 0) {
        uint32_t sec = (uint32_t)pos / 2 / 30;
        snprintf(line, sizeof line, "%lu:%02lu:%02lu", (unsigned long)(sec / 3600), (unsigned long)(sec / 60 % 60),
                 (unsigned long)(sec % 60));
    } else {
        snprintf(line, sizeof line, "-:--:--");
    }
    text(fb, 20, Y_OSD, deck_label(s), C_TEXT, 2);
    text(fb, VIDEO_WIDTH - 20 - 2 * font_string_width(line), Y_OSD, line, C_TEXT, 2);
}

static void draw_menu(uint8_t *fb) {
    int w[M_N], total = 0;
    for (int i = 0; i < M_N; i++) {
        w[i] = font_string_width(menu_label[i]) + 8;
        total += w[i] + (i ? 4 : 0);
    }
    int x = (VIDEO_WIDTH - total) / 2;
    for (int i = 0; i < M_N; i++) {
        bool hit = ui.hit == i && ui.hit_age < 10;
        uint8_t fg = hit ? C_HIT : i == ui.cursor ? C_SEL : C_TEXT;
        fill_rect(fb, x, Y_MENU, w[i], 14, i == ui.cursor ? C_PANEL : C_EDGE);
        box(fb, x, Y_MENU, w[i], 14, fg);
        font_draw_string(fb, x + 4, Y_MENU + 4, menu_label[i], fg);
        x += w[i] + 4;
    }
}

static void draw_message(uint8_t *fb, const vhs_status_t *s) {
    const char *title = "", *sub = "";
    uint8_t tc = C_TITLE;
    switch (ui.state) {
    case T_WAIT: title = "PLEASE WAIT"; break;
    case T_OFF: title = "INSERT GAME TAPE"; sub = "THE VCR IS OFF"; break;
    case T_STOPPED: title = "INSERT GAME TAPE"; sub = "REWIND IT AND PRESS PLAY"; break;
    case T_READING: title = "READING TAPE"; break;
    case T_LOADING: case T_LOADED: title = "LOADING"; break;
    case T_BAD: title = "TAPE READ ERROR"; tc = C_ERR; break;
    case T_REWIND: title = "TAPE NOT REWOUND"; sub = "PRESS LOAD GAME TO REWIND AND LOAD"; tc = C_ERR; break;
    case T_FOREIGN: title = "NOT A GAME TAPE"; sub = "EJECT IT AND INSERT A GAME TAPE"; tc = C_ERR; break;
    case T_TRICK: title = tape_searching() ? "SEARCHING" : "TAPE PAUSED"; sub = "PRESS PLAY"; break;
    }
    if (ui.err && (ui.state != T_LOADING || ui.failed)) {
        title = ui.err;
        sub = ui.err_detail ? ui.err_detail : "";
        tc = C_ERR;
    }
    if (ui.al.st != A_IDLE && ui.state != T_LOADING) {
        title = "LOADING GAME";
        sub = tape_auto_msg[ui.al.st];
        tc = C_TITLE;
    }
    center2(fb, Y_TITLE, title, tc);
    if (ui.state == T_LOADING && s->dt_n) {
        int w = (int)((BAR_W - 4u) * s->dt_have / s->dt_n);
        box(fb, BAR_X - 1, Y_SUB - 3, BAR_W + 2, 12, C_EDGE);
        fill_rect(fb, BAR_X, Y_SUB - 2, BAR_W, 10, C_BAR);
        fill_rect(fb, BAR_X + 2, Y_SUB, w, 6, C_HIT);
    } else {
        center(fb, Y_SUB, sub, C_TEXT);
    }
}

void tape_draw(const vhs_status_t *s) {
    uint8_t *fb = video_get_framebuffer();
    bool saver = ui.idle >= IDLE_FRAMES && (ui.state == T_OFF || ui.state == T_STOPPED);
    memset(fb, C_CLEAR, VIDEO_WIDTH * VIDEO_HEIGHT);
    if (saver) {
        const char *m = "INSERT GAME TAPE";
        int w = font_string_width(m);
        if (ui.svx == 0) {
            ui.sx = 40, ui.sy = 40, ui.svx = 0.6f, ui.svy = 0.4f;
        }
        ui.sx += ui.svx, ui.sy += ui.svy;
        if (ui.sx < 8 || ui.sx > VIDEO_WIDTH - 8 - w) ui.svx = -ui.svx;
        if (ui.sy < 8 || ui.sy > VIDEO_HEIGHT - 16) ui.svy = -ui.svy;
        text(fb, (int)ui.sx, (int)ui.sy, m, C_DIM, 1);
        return;
    }
    ui.svx = 0;
    draw_osd(fb, s);
    draw_message(fb, s);
    draw_menu(fb);
}

// The deck's output full screen as it arrives (raw), not the last tagged field the DE1 holds; hidden with no signal.
static int shown = -1;
void tape_picture(bool on) {
    if (shown == (int)on) return;
    shown = on;
    if (on) video_fullscreen();
    else video_window(0, 0, 0, 0);
}

// The stock palette has (0, 0, 85) here; link.c replays it after a DE1 reset.
static void edge_black(void) {
    static const uint8_t black[1][4] = {{0, 0, 0, 0}};
    link_palette(C_EDGE, 1, black);
}

void tape_ui_enter(void) {
    link_reg_write(REG_OWN, 1);
    link_reg_write(REG_VIDEO, 0x100);
    link_reg_write(REG_FX, 0);
    link_reg_write(REG_AUDIO, 0);
    link_reg_write(REG_OVERLAY, 1);
    link_reg_write(REG_HUD, 0);         // a reboot mid-game leaves the DE1 as the game set it
    link_reg_write(REG_COMB, 0);
    link_reg_write(REG_TAP, 0);
    video_overlay_default();
    video_palette_reset();
    edge_black();
    shown = -1;
    tape_picture(true);
}

void tape_ui_show(const char *title, const char *detail) {
    uint8_t *fb = video_get_framebuffer();
    memset(fb, C_CLEAR, VIDEO_WIDTH * VIDEO_HEIGHT);
    tape_picture(true);
    center2(fb, Y_TITLE, title, C_TITLE);
    center(fb, Y_SUB, detail ? detail : "", C_TEXT);
    link_present(fb, 100);
}
