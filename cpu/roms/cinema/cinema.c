// VHS console demo: an usher walks a cinema lobby with 8 theaters; theater n shows the GCV2 flash clip n
// (stream n - 1, T=2 quads) with its sound. D-pad walks, walking into a door (or A at it) enters, B leaves.
#include <stdint.h>
#include "console_api.h"
#include "video.h"
#include "gamepad.h"
#include "vhs_api.h"

#define CLEAR    0                      // palette index 0 is transparent: the video shows through
#define DARK     RGB332(32, 0, 0)
#define WALL     RGB332(128, 0, 32)
#define WALL_LO  RGB332(96, 0, 0)
#define CARPET   RGB332(160, 0, 64)
#define CARPET2  RGB332(224, 160, 0)
#define GOLD     RGB332(224, 192, 0)
#define DOOR     RGB332(96, 32, 0)
#define GLASS    RGB332(32, 32, 64)
#define BULB_ON  RGB332(255, 255, 128)
#define BULB_OFF RGB332(96, 96, 0)
#define MARQUEE  RGB332(32, 32, 32)
#define CURTAIN  RGB332(192, 0, 0)
#define CURT_LO  RGB332(128, 0, 0)
#define SEAT     RGB332(96, 0, 32)
#define FRAME    RGB332(160, 160, 160)
#define TEXT     RGB332(224, 224, 224)
#define BAD      RGB332(255, 64, 64)

#define DOORS    8
#define DOOR_W   40
#define DOOR_H   52
#define FLOOR_T  84                     // top wall's foot
#define FLOOR_B  176                    // bottom wall's top
#define FEET_MIN (FLOOR_T + 4)          // the usher's feet stay between these
#define FEET_MAX FLOOR_B
#define SCR_X    64                     // theater screen: a quad is 4:3
#define SCR_Y    14
#define SCR_W    192
#define SCR_H    144

// 4x5 glyphs: A-Z, 0-9, then : - . / = %
static const uint8_t font[42][5] = {
    {0x6, 0x9, 0xF, 0x9, 0x9}, {0xE, 0x9, 0xE, 0x9, 0xE}, {0x7, 0x8, 0x8, 0x8, 0x7}, {0xE, 0x9, 0x9, 0x9, 0xE},
    {0xF, 0x8, 0xE, 0x8, 0xF}, {0xF, 0x8, 0xE, 0x8, 0x8}, {0x7, 0x8, 0xB, 0x9, 0x7}, {0x9, 0x9, 0xF, 0x9, 0x9},
    {0x7, 0x2, 0x2, 0x2, 0x7}, {0x1, 0x1, 0x1, 0x9, 0x6}, {0x9, 0xA, 0xC, 0xA, 0x9}, {0x8, 0x8, 0x8, 0x8, 0xF},
    {0x9, 0xF, 0xF, 0x9, 0x9}, {0x9, 0xD, 0xB, 0x9, 0x9}, {0x6, 0x9, 0x9, 0x9, 0x6}, {0xE, 0x9, 0xE, 0x8, 0x8},
    {0x6, 0x9, 0x9, 0xA, 0x5}, {0xE, 0x9, 0xE, 0xA, 0x9}, {0x7, 0x8, 0x6, 0x1, 0xE}, {0x7, 0x2, 0x2, 0x2, 0x2},
    {0x9, 0x9, 0x9, 0x9, 0x6}, {0x9, 0x9, 0x9, 0x6, 0x6}, {0x9, 0x9, 0xF, 0xF, 0x9}, {0x9, 0x9, 0x6, 0x9, 0x9},
    {0x9, 0x9, 0x6, 0x2, 0x2}, {0xF, 0x1, 0x2, 0x4, 0xF}, {0x6, 0x9, 0x9, 0x9, 0x6}, {0x2, 0x6, 0x2, 0x2, 0x7},
    {0x6, 0x9, 0x2, 0x4, 0xF}, {0xE, 0x1, 0x6, 0x1, 0xE}, {0x9, 0x9, 0xF, 0x1, 0x1}, {0xF, 0x8, 0xE, 0x1, 0xE},
    {0x7, 0x8, 0xF, 0x9, 0x6}, {0xF, 0x1, 0x2, 0x4, 0x4}, {0x6, 0x9, 0x6, 0x9, 0x6}, {0x6, 0x9, 0x7, 0x1, 0x6},
    {0x0, 0x2, 0x0, 0x2, 0x0}, {0x0, 0x0, 0xF, 0x0, 0x0}, {0x0, 0x0, 0x0, 0x0, 0x2}, {0x1, 0x2, 0x2, 0x4, 0x8},
    {0x0, 0xF, 0x0, 0xF, 0x0}, {0x9, 0x1, 0x2, 0x4, 0x9},
};

// The usher facing right: h hair, s skin, e eye, u uniform, y buttons, p trousers, k shoes.
static const char *const body[11] = {
    "...hhhh...", "..hhhhhh..", "..hhssss..", "..hsssess.", "...ssss...", "...uuuu...",
    "..uuyuuu..", ".suuuuuus.", ".suuyuuus.", "..uuuuuu..", "..pppppp..",
};
static const char *const legs[3][5] = {
    {"..pp..pp..", "..pp..pp..", "..pp..pp..", "..pp..pp..", "..kkk.kkk."},
    {".pp....pp.", ".pp....pp.", "pp......pp", "pp......pp", "kkk....kkk"},
    {"...pppp...", "...pppp...", "...pppp...", "...pppp...", "...kkkkk.."},
};
static const char *const seated[6] = {
    "...hhhh...", "..hhhhhh..", "..hhhhhh..", "...hhhh...", "..uuuuuu..", ".uuuuuuuu.",
};

static uint8_t *fb;

static int glyph(char c) {
    static const char sym[] = ":-./=%";
    if (c >= 'A' && c <= 'Z')
        return c - 'A';
    if (c >= '0' && c <= '9')
        return 26 + c - '0';
    for (int i = 0; sym[i]; i++)
        if (c == sym[i])
            return 36 + i;
    return -1;
}

static void rect(int x, int y, int w, int h, uint8_t c) {
    for (int j = y; j < y + h; j++)
        for (int i = x; i < x + w; i++)
            if ((unsigned)i < VIDEO_WIDTH && (unsigned)j < VIDEO_HEIGHT)
                fb[j * VIDEO_WIDTH + i] = c;
}

static void text(int x, int y, const char *s, uint8_t c, int z) {
    for (; *s; s++, x += 5 * z) {
        int g = glyph(*s);
        if (g < 0)
            continue;
        for (int py = 0; py < 5; py++)
            for (int px = 0; px < 4; px++)
                if (font[g][py] & (8 >> px))
                    rect(x + px * z, y + py * z, z, z, c);
    }
}

static int text_w(const char *s, int z) {
    int n = 0;
    while (s[n])
        n++;
    return n ? (5 * n - 1) * z : 0;
}

static uint8_t sprite_colour(char c) {
    switch (c) {
    case 'h': return RGB332(64, 32, 0);
    case 's': return RGB332(255, 192, 128);
    case 'e': return RGB332(32, 32, 64);
    case 'u': return RGB332(192, 0, 0);
    case 'y': return GOLD;
    case 'p': return RGB332(32, 32, 96);
    case 'k': return RGB332(64, 64, 64);
    default:  return CLEAR;
    }
}

static void sprite(int x, int y, const char *const *rows, int n, int mirror, int z) {
    for (int j = 0; j < n; j++)
        for (int i = 0; i < 10; i++) {
            uint8_t c = sprite_colour(rows[j][mirror ? 9 - i : i]);
            if (c != CLEAR)
                rect(x + i * z, y + j * z, z, z, c);
        }
}

static void usher(int fx, int fy, int frame, int left) {
    sprite(fx - 5, fy - 16, body, 11, left, 1);
    sprite(fx - 5, fy - 5, legs[frame], 5, left, 1);
}

// Doors 0-3 on the top wall, 4-7 on the bottom wall.
static int door_x(int d) { return 24 + (d & 3) * 76; }
static int door_top(int d) { return d < 4 ? FLOOR_T - DOOR_H : FLOOR_B; }

static void draw_door(int d, int lit, uint32_t blink) {
    int x = door_x(d), y = door_top(d), h = d < 4 ? DOOR_H : DOOR_H - 6;
    rect(x - 3, y - (d < 4 ? 3 : 0), DOOR_W + 6, h + 3, GOLD);
    rect(x, y + (d < 4 ? 0 : 3), DOOR_W, h - 3, DOOR);
    rect(x + DOOR_W / 2, y, 1, h, GOLD);
    rect(x + 6, y + 12, 8, 10, GLASS);
    rect(x + DOOR_W - 14, y + 12, 8, 10, GLASS);
    rect(x + DOOR_W / 2 - 4, y + 28, 2, 6, GOLD);
    rect(x + DOOR_W / 2 + 3, y + 28, 2, 6, GOLD);

    int my = d < 4 ? 3 : VIDEO_HEIGHT - 15;
    rect(x - 2, my, DOOR_W + 4, 14, lit ? CURTAIN : MARQUEE);
    for (int i = 0; i < 6; i++) {
        uint8_t c = lit || ((blink + (uint32_t)i) & 1) ? BULB_ON : BULB_OFF;
        rect(x + i * 8, my - 2, 2, 2, c);
        rect(x + i * 8, my + 14, 2, 2, c);
    }
    char n[2] = {(char)('1' + d), 0};
    text(x + DOOR_W / 2 - 4, my + 2, n, lit ? BULB_ON : TEXT, 2);
}

static void draw_lobby(int fx, int fy, int frame, int left, int near, uint32_t blink) {
    rect(0, 0, VIDEO_WIDTH, FLOOR_T, WALL);
    rect(0, FLOOR_T - 4, VIDEO_WIDTH, 4, WALL_LO);
    for (int y = FLOOR_T; y < FLOOR_B; y++)
        for (int x = 0; x < VIDEO_WIDTH; x++)
            fb[y * VIDEO_WIDTH + x] = ((x + y) & 15) == 0 || ((x - y) & 15) == 0 ? CARPET2 : CARPET;
    rect(0, FLOOR_B, VIDEO_WIDTH, VIDEO_HEIGHT - FLOOR_B, WALL);
    for (int d = 0; d < DOORS; d++)
        draw_door(d, d == near, blink);
    const char *title = "VHS CINEMA";
    int ty = (FLOOR_T + FLOOR_B) / 2;
    rect(VIDEO_WIDTH / 2 - 56, ty - 9, 112, 18, WALL_LO);
    text(VIDEO_WIDTH / 2 - text_w(title, 2) / 2, ty - 5, title, GOLD, 2);
    usher(fx, fy, frame, left);
}

static void draw_theater(int n, int seen) {
    rect(0, 0, VIDEO_WIDTH, VIDEO_HEIGHT, DARK);
    rect(SCR_X - 2, SCR_Y - 2, SCR_W + 4, SCR_H + 4, FRAME);
    rect(SCR_X, SCR_Y, SCR_W, SCR_H, CLEAR);
    for (int i = 0; i < 6; i++) {
        rect(SCR_X - 40 + i * 6, 4, 6, 168, i & 1 ? CURT_LO : CURTAIN);
        rect(SCR_X + SCR_W + 4 + i * 6, 4, 6, 168, i & 1 ? CURT_LO : CURTAIN);
    }
    rect(SCR_X - 40, 0, SCR_W + 80, 6, CURTAIN);
    if (!seen)
        text(SCR_X + SCR_W / 2 - text_w("NO PICTURE", 1) / 2, SCR_Y + SCR_H / 2 - 2, "NO PICTURE", BAD, 1);

    for (int row = 0; row < 3; row++) {
        int y = 192 + row * 16, off = row & 1 ? 0 : 10;
        if (row == 1)
            sprite(VIDEO_WIDTH / 2 - 10, y - 12, seated, 6, 0, 2);
        for (int x = off; x < VIDEO_WIDTH; x += 20)
            rect(x + 1, y, 18, 12, SEAT);
    }
    char label[12] = "THEATER 0";
    label[8] = (char)('1' + n);
    text(4, 178, label, TEXT, 1);
    text(VIDEO_WIDTH - 4 - text_w("B LEAVE", 1), 178, "B LEAVE", TEXT, 1);
}

// The door the usher's feet are at, or -1.
static int door_at(int fx, int fy) {
    for (int d = 0; d < DOORS; d++) {
        int x = door_x(d);
        if (fx >= x + 4 && fx < x + DOOR_W - 4 && (d < 4 ? fy <= FEET_MIN + 6 : fy >= FEET_MAX - 6))
            return d;
    }
    return -1;
}

__attribute__((section(".entry"))) void game_main(const console_api_t *api) {
    const vhs_api_t *vhs = (const vhs_api_t *)api;
    api->video_init(VIDEO_MODE_SRAM60);
    fb = api->framebuffer();
    if (vhs->magic != VHS_API_MAGIC) {
        for (;;) {
            rect(0, 0, VIDEO_WIDTH, VIDEO_HEIGHT, WALL);
            text(90, 115, "NOT A VHS CONSOLE", BAD, 1);
            api->vsync();
        }
    }

    int fx = VIDEO_WIDTH / 2, fy = FEET_MAX - 16, left = 0, step = 0;
    int theater = -1, shown = -2;
    uint32_t frames = 0;

    for (;;) {
        gamepad_state_t pad;
        api->gamepad_read(&pad);
        uint32_t held = pad.connected ? pad.buttons : 0, pressed = pad.connected ? pad.buttons_pressed : 0;

        if (theater >= 0) {
            if (pressed & BTN_B) {
                fx = door_x(theater) + DOOR_W / 2;
                fy = theater < 4 ? FEET_MIN + 10 : FEET_MAX - 10;
                left = 0;
                step = 0;
                theater = -1;
            }
        } else {
            int dx = (held & BTN_DPAD_RIGHT ? 2 : 0) - (held & BTN_DPAD_LEFT ? 2 : 0);
            int dy = (held & BTN_DPAD_DOWN ? 2 : 0) - (held & BTN_DPAD_UP ? 2 : 0);
            int pushing = (dy < 0 && fy + dy < FEET_MIN) || (dy > 0 && fy + dy > FEET_MAX);
            fx += dx;
            fy += dy;
            fx = fx < 6 ? 6 : fx > VIDEO_WIDTH - 6 ? VIDEO_WIDTH - 6 : fx;
            fy = fy < FEET_MIN ? FEET_MIN : fy > FEET_MAX ? FEET_MAX : fy;
            if (dx)
                left = dx < 0;
            step = dx || dy ? step + 1 : 0;
            int d = door_at(fx, fy);
            if (d >= 0 && (pushing || (pressed & BTN_A)))
                theater = d;
        }

        if (theater != shown) {
            if (theater < 0) {
                vhs->video_window(0, 0, 0, 0);
                vhs->reg_write(REG_AUDIO, 1u << 6);
            } else {
                vhs->reg_write(REG_VIDEO, (uint32_t)theater);
                vhs->reg_write(REG_AUDIO, 0);
                vhs->video_window(SCR_X, SCR_Y, SCR_W, SCR_H);
            }
            shown = theater;
        }

        if (theater >= 0) {
            vhs_status_t st;
            draw_theater(theater, !vhs->status(&st) || (st.flags0 & ST0_IN_LOCK));
        } else {
            int frame = step ? 1 + (step / 6 & 1) : 0;
            draw_lobby(fx, fy, frame, left, door_at(fx, fy), frames / 15);
        }
        api->vsync();
        frames++;
    }
}
