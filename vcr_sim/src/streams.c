// Multi-stream NTSC generator: real 480i (or 240p), rendered line by line into a ring the DMA plays out
// of the 6-bit R-2R DAC on GPIO2..7. Each field carries a tag line; streams are laid out by `ctl.t_log2`
// (fields per cycle) and `ctl.layout` (regions per field), or baked into the clips in flash (`ctl.src`
// 1); `ctl.src` 2 instead sends the data tape in flash, a game image as black and white cells.
// Picture lines can go through a VHS channel model first. Core 1 renders; core 0 services the DMA and
// derives the model's coefficients. Control and stats live in `ctl` (scripts/gcv2_ctl.sh).
// The deck audio (PCM5102A) carries SMPTE LTC on the right channel, rendered with each line so it stays
// locked to the video. The deck's transport is emulated from its IR codes (vcr.c): every
// field's length, content offset and tape position come from there.
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/pio.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/systick.h"
#include "hardware/vreg.h"
#include "tvout.pio.h"
#include "i2s.pio.h"
#include "pad.h"
#include "sd.h"
#include "tape_store.h"
#include "vcr.h"
#include "dac_cal.h"

#define DAC_BASE_PIN 2
#define DAC_BITS 6
#define SYS_KHZ 315000
#define PIO_DIV 11                      // 315 MHz / (2 instr * 11) = 14.31818 MHz = 4 fsc
#define FS_HZ 14318182.0f

#define SPL 910                         // samples per line, 227.5 subcarrier cycles
#define HALF (SPL / 2)
#define LINE_HW (SPL / 2)               // DMA moves one halfword (two samples) per transfer
#define SYNC_LEN 67
#define EQ_PULSE 33
#define BURST_START (SYNC_LEN + 9)
#define BURST_LEN 36
#define ACTIVE_START 136                // samples; a multiple of 4, so pixel phase depends only on x
#define W 376                           // pixels of two samples each
#define ROWS 240
#define ROW0_F1 21                      // content line of row 0, counted from the field's first whole line
#define TAG_LINE_F1 18
#define TAG_X0 8
#define TAG_CELL 18                     // pixels per tag bit: 2.5 us, well inside VHS luma bandwidth
#define TAG_CELLS 19                    // run-in 1 0, 16 data bits MSB first, stop 1
#define VITC_L1 13                      // SMPTE VITC on frame lines 14 and 16 (277 and 279)
#define VITC_L2 15
#define VITC_X0 160                     // DAC samples: bit 0 starts 11.2 us after the sync's leading edge
#define VITC_BITS 90
#define VITC_SPAN 716                   // 90 cells of H / 115 = 910 / 115 samples, plus the eased tail
#define PTAG_LINES 0xB5E00u             // frame lines 10-13, 15, 17, 18, 20 (bit = 0-based line)
#define PTAG_X0 160
#define PTAG_CELL 7                     // DAC samples (0.489 us), the data tape's proven width 7
#define PTAG_CELLS 32                   // 1 0, 8 bits, 1 0, 8 bits, 1 0, CRC-8, 0 0
#define PTAG_COPIES 3
#define RING 16

// Deck audio: 48 kHz stereo I2S to the PCM5102A on GPIO8 (DIN), 9 (BCK), 10 (LRCK), as on GameConsoleV2.
#define I2S_DIN_PIN 8
#define I2S_BCK_PIN 9
#define I2S_DIV_INT 102                 // 315 MHz / (64 * 48 kHz) = 102 + 138/256 exactly
#define I2S_DIV_FRAC 138
#define A_RING 256                      // stereo frames; the DMA wraps over it
#define A_T 13125                       // one 48 kHz frame, in half sys clocks
#define LINE_T 40040                    // one line, in half sys clocks

// In-line audio: every picture row starts with one level cell per audio stream (one per video stream,
// up to 8), then a white pilot. Each cell is error-diffused over its 12 DAC samples.
#define AUD_CELLS 8
#define AUD_LEAD 2                      // pixels at mid level after blanking
#define AUD_CELL 6                      // pixels per cell: 11.3 DE1 luma samples
#define AUD_PILOT 4
#define AUD_W(n) (AUD_LEAD + (n) * AUD_CELL + AUD_PILOT + 2)   // chipset vga_out's AUD_* tables
#define AUD_MID16 532                   // 55 IRE, in 1/16 DAC codes
enum { AUD_OFF, AUD_TONES, AUD_SILENT, AUD_CLIP };

// Streams = T fields per cycle x the layout's regions per field; stream = phase * regions + region.
enum { LAYOUT_FULL, LAYOUT_HALVES, LAYOUT_QUADS, LAYOUT_LINES, LAYOUT_COUNT };
static const uint8_t layout_regions[LAYOUT_COUNT] = {1, 2, 4, 2};
#define T_LOG2_MAX 2                    // 4 fields per cycle: 15 fps per stream

// Poked and read over SWD; keep the field list in step with scripts/gcv2_ctl.sh.
typedef struct {
    uint32_t magic;
    uint32_t t_log2;                    // requested; this and everything below latch at a frame start
    uint32_t layout;
    uint32_t interlaced;                // 1: 525-line 480i, 0: 262-line 240p
    uint32_t fields;                    // field counter carried in the tag
    uint32_t underruns;                 // lines the DMA reached before they were rendered
    uint32_t active_t_log2;
    uint32_t active_layout;
    uint32_t active_interlaced;
    uint32_t plain;                     // streams as flat colour fields, for bleed measurement
    uint32_t guard_px;                  // neutral grey each side of a vertical stream boundary
    uint32_t guard_rows;                // ... and of a horizontal one
    uint32_t vhs;                       // run picture lines through the VHS channel model
    uint32_t y_bw_khz;
    uint32_t c_bw_khz;                  // overall -3 dB of c_poles cascaded one-pole filters
    uint32_t c_poles;
    uint32_t c_delay_ns;
    uint32_t c_comb;                    // deck 1H chroma averaging
    uint32_t y_noise;                   // peak, 0.1 IRE
    uint32_t c_noise;                   // peak before the chroma filter, 0.1 IRE
    uint32_t hs_lines;                  // head switch: last rows of each field shifted right
    uint32_t hs_shift_px;
    uint32_t cycles_max;                // worst line render in the last field, of 20020 per line
    uint32_t cycles_avg;                // mean over the last field's picture lines
    uint32_t pad_connected;             // bitmask: N64 controllers answering on GP40, GP41
    uint32_t pad_buttons;
    uint32_t pad_edges;                 // falling edges during each port's last probe (bring-up aid)
    uint32_t aud;                       // AUD_OFF, AUD_TONES (stream k plays aud_hz[k]), AUD_SILENT
    uint32_t aud_level;                 // tone peak, 0.1 IRE either side of 55 IRE
    uint32_t aud_lines;                 // lines rendered with audio cells
    uint32_t aud_mid16;                 // cell rest level, 1/16 DAC code (AUD_SILENT sweeps the DAC)
    uint32_t aud_lin;                   // diffuse against dac_lev16, the measured code levels
    uint32_t aud_lead;                  // DAC samples before cell 0: moves the cells against the DE1's windows
    uint32_t aud_cycles;                // draw_audio, last line
    uint32_t src;                       // the game tape after its leader: 0 test streams, 1 the clips in flash (tools/clips.py), 3 the SD card
    uint32_t clips_ok;                  // clip header valid and payload checksum matched at boot
    uint32_t clip_fields;
    uint32_t hs_blank;                  // last rows of each field left black, clear of the deck's head switch
    uint32_t data_ok;                   // data tape blob valid at boot (tools/datatape.py, src 2)
    uint32_t data_width;                // DAC samples per bit cell, 4-7
    uint32_t data_rows;                 // header, packets, parity; the rows after stay black
    uint32_t data_black;                // 0.1 IRE
    uint32_t data_white;
    uint32_t data_drop;                 // test dropouts: events per 1000 data lines ...
    uint32_t data_burst;                // ... each this many lines long
    uint32_t data_skew;                 // test: +- DAC samples of random line position
    uint32_t data_packets;              // packets in the image at the active width
    uint32_t data_sent;                 // data packets sent
    uint32_t data_parity;               // parity rows K, 1-8: rebuilds any K-row burst per field
    uint32_t ltc;                       // SMPTE LTC on the deck audio's right channel
    uint32_t ltc_amp;                   // peak, of 32767
    uint32_t ltc_frame;                 // frame number of the word being sent
    uint32_t ltc_user;                  // the words' 32 user bits
    uint32_t left_hz;                   // left channel test tone; 0 for silence
    uint32_t left_amp;
    uint32_t ltc_seek;                  // frame + 1: the next frame start jumps the tape there, then clears it
    uint32_t vitc;                      // SMPTE VITC of the LTC's frame, with the field mark
    uint32_t ptag;                      // position tag: the field count, repeated for pause and cue
    uint32_t vcr;                       // 1: emulate the deck (IR on GP43); 0: always playing
    uint32_t vcr_state;                 // VS_* in vcr.h
    uint32_t vcr_pos;                   // tape field shown
    uint32_t vcr_len;                   // tape length in fields
    uint32_t vcr_cmd;                   // command byte + 1, as if received by IR; cleared once taken
    uint32_t ir_frames;                 // deck codes decoded
    uint32_t ir_bad;                    // frames that were not a deck code
    uint32_t ir_last;                   // vendor << 16 | command of the last one
    uint32_t vbi_from;                  // pause/cue: content lines above this reproduce nothing
    uint32_t vbi_keep;                  // ... below it a line survives whole, per mille, else only a piece ...
    uint32_t tag_keep;                  // ... the tag line's odds ...
    uint32_t vbi_min;                   // ... and the piece's length in DAC samples
    uint32_t vbi_max;
    uint32_t leader;                    // src 0/1: the tape's first fields carry the data tape
    uint32_t vcr_cass;                  // CASS_* in vcr.h: the cassette in the slot
    uint32_t vcr_saves;                 // tape state writes to flash since boot
    uint32_t sd_ok;                     // src 3: card up and its tape header valid (tools/clips.py --sd)
    uint32_t sd_fields;                 // fields on the card, played after the leader
    uint32_t sd_div;                    // sys clocks per SD clock: 13 is 24.2 MHz
    uint32_t sd_init;                   // 1: bring the card up again; cleared once tried
    uint32_t sd_reads;                  // fields read
    uint32_t sd_crc;                    // reads failing a block CRC
    uint32_t sd_timeouts;
    uint32_t sd_cmd_err;
    uint32_t sd_miss;                   // fields not read in time: the last one repeats
    uint32_t sd_stage;                  // last command of the bring-up (100: up), then of each read
    uint32_t sd_resp;                   // its response's content bits
    uint32_t sd_hdr_magic;              // what sector 0 held
    uint32_t sd_fail;                   // last failed command: index | 0x100 no response, | 0x200 bad CRC or index
    uint32_t sd_test;                   // n: send CMD13 n times now (bench aid); cleared once done ...
    uint32_t sd_test_bad;               // ... and how many failed
    uint32_t vcr_fast;                  // debug: FF/REW reach the end of the tape in one field
    uint32_t st_assets, st_entries;     // asset store: assets on the card, schedule entries in flash
    uint32_t st_dead, st_reads, st_fails;   // entries naming no asset (or the wrong size), card reads, failed ones
} ctl_t;
volatile ctl_t ctl = {
    .magic = 0x47435632, .t_log2 = 1, .layout = LAYOUT_FULL, .interlaced = 1,
    .y_bw_khz = 3000, .c_bw_khz = 500, .c_poles = 2, .c_comb = 1, .y_noise = 10, .c_noise = 30,
    .hs_lines = 5, .hs_shift_px = 8, .aud = AUD_TONES, .aud_level = 300, .aud_mid16 = AUD_MID16,
    .aud_lin = 1, .aud_lead = 2 * AUD_LEAD, .src = 1, .hs_blank = 4,
    .data_width = 6, .data_rows = 230, .data_black = 75, .data_white = 900, .data_burst = 1,
    .data_parity = 8, .ltc = 1, .ltc_amp = 4096, .left_amp = 4096, .vitc = 1, .ptag = 1,
    .vcr = 1, .vcr_len = 431568, .vbi_from = 17, .vbi_keep = 470, .tag_keep = 170, .vbi_min = 50, .vbi_max = 300,
    .leader = 600, .sd_div = 13,
};

// Not harmonics of each other, so one stream's distortion never lands on another's tone.
static const float aud_hz[AUD_CELLS] = {300, 375, 469, 586, 732, 916, 1144, 1430};
static int16_t  sine[4096];
static uint32_t aud_ph[AUD_CELLS], aud_inc[AUD_CELLS];
static int32_t  aud_err[AUD_CELLS];     // diffusion error carried to the next line: first-order shaping
static int32_t  aud_lev[64];            // levels in ascending order (the DAC is not monotonic) ...
static uint8_t  aud_code[64];           // ... and the code for each
static uint8_t  aud_near[64];           // aud_lev index at or below 16 * code: where the search starts
static uint8_t  aud_white;              // pilot

// Stream k's hue: 90 degrees between streams that can share a boundary (k ^ 1, k ^ 2), 22.5 degrees
// on per group of four. C_STREAM0 is the normal pattern's RGB wheel; C_MEAS0 is plain mode's, 50 IRE
// luma and 30 IRE chroma at that angle in the UV plane.
#define NSTREAMS 16
enum { C_BLACK, C_WHITE, C_GREY, C_BLUE, C_STREAM0,C_MEAS0 = C_STREAM0 + NSTREAMS, NCOL = C_MEAS0 + NSTREAMS };
static float stream_deg(int k) {
    static const int16_t base[4] = {0, 90, 270, 180};
    return base[k & 3] + 22.5f * (k >> 2);
}
static uint16_t pal[NCOL][2];           // two samples of a pixel at subcarrier phases (0,1) and (2,3)
static int32_t  yuv[NCOL][3];           // IRE * 256

enum { T_EE, T_BB, T_EB, T_BE, T_E0, T_NE, T_N, NTYPES };
static uint16_t tmpl[NTYPES][2][LINE_HW];
static uint16_t ring[RING][LINE_HW];
static uint8_t  idx[W];                 // the line being drawn, as palette indices

static volatile uint32_t queued = 1;    // line index last handed to a DMA channel
static volatile uint32_t rendered;      // lines rendered so far
static uint dma_a, dma_b;

static const uint8_t font5x7[NSTREAMS][7] = {
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}, {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}, {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E},
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}, {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}, {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}, {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},
    {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}, {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},
    {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}, {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E},
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}, {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10},
};

static uint8_t ire_code(float ire) {
    int c = (int)lroundf((ire + 40.0f) / 180.0f * 63.0f);
    return c < 0 ? 0 : c > 63 ? 63 : (uint8_t)c;
}

static void pal_set_yuv(int i, float Y, float U, float V) {
    pal[i][0] = ire_code(Y + V) | ire_code(Y + U) << 8;
    pal[i][1] = ire_code(Y - V) | ire_code(Y - U) << 8;
    yuv[i][0] = (int32_t)lroundf(Y * 256);
    yuv[i][1] = (int32_t)lroundf(U * 256);
    yuv[i][2] = (int32_t)lroundf(V * 256);
}

static void pal_set(int i, float r, float g, float b) {
    float y = 0.299f * r + 0.587f * g + 0.114f * b;
    pal_set_yuv(i, 7.5f + 92.5f * y, 92.5f * 0.492f * (b - y), 92.5f * 0.877f * (r - y));
}

// Sync pulse at the start of a half line: E equalising, B broad (serrated), N hsync + burst, 0 none.
static void half_pulse(uint8_t *s, int at, char kind, int lp) {
    uint8_t tip = ire_code(-40.0f);
    if (kind == 'E') memset(s + at, tip, EQ_PULSE);
    if (kind == 'B') memset(s + at, tip, HALF - SYNC_LEN);
    if (kind == 'N') {
        memset(s + at, tip, SYNC_LEN);
        static const float sin4[4] = {0, 1, 0, -1};
        for (int i = BURST_START; i < BURST_START + BURST_LEN; i++)
            s[at + i] = ire_code(-20.0f * sin4[(2 * lp + i) & 3]);
    }
}

static const char kinds[NTYPES][2] = {
    {'E', 'E'}, {'B', 'B'}, {'E', 'B'}, {'B', 'E'}, {'E', '0'}, {'N', 'E'}, {'N', '0'}};

static void build_templates(void) {
    for (int t = 0; t < NTYPES; t++)
        for (int lp = 0; lp < 2; lp++) {
            uint8_t *s = (uint8_t *)tmpl[t][lp];
            memset(s, ire_code(0.0f), SPL);
            half_pulse(s, 0, kinds[t][0], lp);
            half_pulse(s, HALF, kinds[t][1], lp);
        }
}

// Half-line s of a field: 6 equalising, 6 broad, 6 equalising pulses, then an hsync at each line start.
static char slot_kind(int s, bool line_start) {
    if (s < 6 || (s >= 12 && s < 18)) return 'E';
    if (s < 12) return 'B';
    return line_start ? 'N' : '0';
}

static int line_type(char a, char b) {
    for (int t = 0; t < NTYPES; t++)
        if (kinds[t][0] == a && kinds[t][1] == b) return t;
    return T_N;
}

static inline void fill(int x0, int x1, int c) {
    if (x0 < 0) x0 = 0;
    if (x1 > W) x1 = W;
    if (x1 > x0) memset(idx + x0, c, (size_t)(x1 - x0));
}

typedef struct { int x0, w, h, ly, stream; } span_t;

// The picture spans pixels x0..W-1 (x0 > 0 leaves room for the audio cells); regions on the bottom edge
// end at `rows`, above the head switch blanking, as tools/clips.py fits its bottom quads.
static int layout_row(int lay, int phase, int row, int x0, int rows, span_t *sp) {
    int base = phase * layout_regions[lay], w = W - x0, h2 = w / 2, n = 1;
    switch (lay) {
    case LAYOUT_HALVES:
        sp[0] = (span_t){x0, h2, ROWS, row, base};
        sp[1] = (span_t){x0 + h2, h2, ROWS, row, base + 1};
        n = 2;
        break;
    case LAYOUT_QUADS: {
        int b = row >= ROWS / 2;
        sp[0] = (span_t){x0, h2, ROWS / 2, row - b * ROWS / 2, base + 2 * b};
        sp[1] = (span_t){x0 + h2, h2, ROWS / 2, row - b * ROWS / 2, base + 2 * b + 1};
        n = 2;
        break;
    }
    case LAYOUT_LINES: sp[0] = (span_t){x0, w, rows / 2, row >> 1, base + (row & 1)}; return 1;
    default: sp[0] = (span_t){x0, w, ROWS, row, base}; break;
    }
    for (int i = 0; i < n; i++)
        sp[i].h = MIN(sp[i].h, rows - (row - sp[i].ly));
    return n;
}

// One row of a stream's picture, in region coordinates: colour field, border, stream digit, a bar moving
// with the stream's own clock t, and t in binary along the bottom so dropped or repeated fields show.
static void draw_stream(const span_t *s, uint32_t t, bool plain) {
    int x0 = s->x0, x1 = s->x0 + s->w, y = s->ly, h = s->h;
    if (plain) {
        fill(x0, x1, C_MEAS0 + s->stream);
        return;
    }
    fill(x0, x1, C_STREAM0 + s->stream);
    if (y < 2 || y >= h - 2) {
        fill(x0, x1, C_WHITE);
        return;
    }
    fill(x0, x0 + 2, C_WHITE);
    fill(x1 - 2, x1, C_WHITE);

    int sc = h / 14, dw = 5 * sc, dx = x0 + (s->w - dw) / 2, dy = h / 2 - 7 * sc / 2 - h / 8;
    if (y >= dy && y < dy + 7 * sc) {
        uint8_t bits = font5x7[s->stream][(y - dy) / sc];
        for (int c = 0; c < 5; c++)
            if (bits & (0x10 >> c)) fill(dx + c * sc, dx + (c + 1) * sc, C_WHITE);
    }

    int cw = (s->w - 8) / 8, by = h - 4 - h / 10;
    if (y >= by && y < h - 4)
        for (int i = 0; i < 8; i++) {
            int a = x0 + 4 + i * cw;
            fill(a + 1, a + cw - 1, (t >> (7 - i)) & 1 ? C_WHITE : C_BLACK);
        }
    else {
        int bx = x0 + 4 + (int)((t * 3) % (uint32_t)(s->w - 16));
        fill(bx, bx + 8, C_BLACK);
    }
}

// {t_log2[1:0], layout[1:0], 240p, count[6:0], check[3:0]}; check = XOR of the payload nibbles ^ 0xA.
static uint16_t tag_word(int t_log2, int lay, bool il, uint32_t fc) {
    uint16_t p = (uint16_t)((t_log2 & 3) << 10 | (lay & 3) << 8 | (il ? 0 : 0x80) | (fc & 0x7F));
    return (uint16_t)(p << 4 | ((p >> 8 ^ p >> 4 ^ p ^ 0xA) & 0xF));
}

static void draw_tag(uint16_t word) {
    uint32_t cells = 1u << 18 | (uint32_t)word << 1 | 1;
    fill(0, W, C_BLACK);
    for (int i = 0; i < TAG_CELLS; i++)
        if ((cells >> (TAG_CELLS - 1 - i)) & 1) fill(TAG_X0 + i * TAG_CELL, TAG_X0 + (i + 1) * TAG_CELL, C_WHITE);
}

static void aud_lev_setup(bool measured) {
    for (int c = 0; c < 64; c++) {
        int32_t lv = measured ? dac_lev16[c] : c << 4;
        int i = c;
        for (; i > 0 && aud_lev[i - 1] > lv; i--) {
            aud_lev[i] = aud_lev[i - 1];
            aud_code[i] = aud_code[i - 1];
        }
        aud_lev[i] = lv;
        aud_code[i] = (uint8_t)c;
    }
    for (int c = 0, i = 0; c < 64; c++) {
        while (i < 62 && aud_lev[i + 1] <= c << 4) i++;
        aud_near[c] = (uint8_t)i;
    }
}

// s is the first picture sample. Audio sample n of every stream goes on the n-th picture row. Each cell
// is a two-level sigma-delta between the levels either side of its target; the error carries on.
static void draw_audio(uint8_t *s, int n, int mode, int32_t mid16, int32_t amp16, int lead,
                       const int8_t *smp) {
    uint8_t mid = (uint8_t)((mid16 + 8) >> 4);
    uint8_t *end = s + 2 * AUD_W(n);
    memset(s, mid, (size_t)lead);
    s += lead;
    for (int k = 0; k < n; k++) {
        int32_t t = mid16, e = aud_err[k];
        if (mode == AUD_TONES) {
            t += sine[aud_ph[k] >> 20] * amp16 >> 15;
            aud_ph[k] += aud_inc[k];
        } else if (mode == AUD_CLIP) {
            t += smp[k] * amp16 >> 7;
        }
        int i = aud_near[MIN(MAX(t >> 4, 0), 63)];
        while (i < 62 && aud_lev[i + 1] <= t) i++;
        const int32_t lo = aud_lev[i], hi = aud_lev[i + 1];
        const uint8_t clo = aud_code[i], chi = aud_code[i + 1];
        for (int j = 0; j < 2 * AUD_CELL; j++) {
            int32_t v = t + e;
            bool up = v >= hi;
            e = v - (up ? hi : lo);
            *s++ = up ? chi : clo;
        }
        aud_err[k] = e;
    }
    memset(s, aud_white, 2 * AUD_PILOT);
    s += 2 * AUD_PILOT;
    memset(s, mid, (size_t)(end - s));
}

static void out_direct(uint16_t *px, int lp, int x0) {
    for (int x = x0; x < W; x++)
        px[x] = pal[idx[x]][(lp ^ x) & 1];
}

// Clips from tools/clips.py: a header, then per field CLIP_PAIRS row-pair records of
// {int8 audio[2][aud_n], luma nibbles[2][width / 2], chroma (U | V << 4) per 4x2 pixels[width / 4]}.
#define CLIPS_OFFSET 0x40000
#define CLIPS_MAGIC 0x43534856          // "VHSC"
#define CLIP_HDR 4096
#define CLIP_PAIRS (ROWS / 2)           // clip.rows of them carry picture and audio, the rest are blank
#define HS_BLANK_MAX 16
#define CLIP_PAIR_MAX 512
#define DATA_OFFSET 0xF00000            // data tape blob; clips must end before it
#define SAVE_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)    // the deck's tape state; the data blob ends before it
typedef struct {
    uint32_t magic, version, t_log2, layout, width, rows, fields, pair_bytes, aud_n, sum;
    int32_t y_tab[16], u_tab[16], v_tab[16];    // IRE * 256
} clip_hdr_t;
static const uint8_t *const clip_flash = (const uint8_t *)(XIP_NOCACHE_NOALLOC_BASE + CLIPS_OFFSET);
static clip_hdr_t clip;
static uint16_t clut[2][16][256];       // like pal[][]: luma level x chroma byte
static uint32_t clip_buf[2][CLIP_PAIR_MAX / 4];
static uint32_t clip_id[2] = {UINT32_MAX, UINT32_MAX}, clip_ids;
static int clip_busy = -1;              // buffer the DMA is filling
static uint clip_dma;

static void clip_fetch(int b, uint32_t id) {
    dma_channel_config c = dma_channel_get_default_config(clip_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_write_increment(&c, true);
    dma_channel_configure(clip_dma, &c, clip_buf[b], clip_flash + CLIP_HDR + id * clip.pair_bytes,
                          clip.pair_bytes / 4, true);
    clip_id[b] = id;
    clip_busy = b;
}

static void clip_wait(void) {
    if (clip_busy < 0) return;
    dma_channel_wait_for_finish_blocking(clip_dma);
    clip_busy = -1;
}

// Record id, with the one after it prefetched into the other buffer.
static const uint8_t *clip_pair(uint32_t id) {
    uint32_t next = id + 1 == clip_ids ? 0 : id + 1;
    int b = id & 1;
    if (clip_busy == b || clip_id[b] != id) {
        clip_wait();
        if (clip_id[b] != id) {
            clip_fetch(b, id);
            clip_wait();
        }
    }
    if (clip_id[b ^ 1] != next) {
        clip_wait();
        clip_fetch(b ^ 1, next);
    }
    return (const uint8_t *)clip_buf[b];
}

static uint32_t flash_sum(const void *src, uint32_t words) {
    static uint32_t sink;
    uint ch = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_sniff_enable(&c, true);
    dma_sniffer_enable(ch, DMA_SNIFF_CTRL_CALC_VALUE_SUM, false);
    dma_sniffer_set_data_accumulator(0);
    dma_channel_configure(ch, &c, &sink, src, words, true);
    dma_channel_wait_for_finish_blocking(ch);
    uint32_t sum = dma_sniffer_get_data_accumulator();
    dma_sniffer_disable();
    dma_channel_unclaim(ch);
    return sum;
}

// A v1 record row: 4-bit luma, two pixels a byte. c0/c1: the line's two chroma phases.
static void out_clip(uint16_t *px, int x0, const uint8_t *rec, int j, int width, int aud_n,
                     const uint16_t (*c0)[256], const uint16_t (*c1)[256]) {
    const uint8_t *y = rec + 2 * aud_n + j * (width / 2), *c = rec + 2 * aud_n + width;
    px += x0;
    for (int i = 0; i < width / 2; i++) {
        px[2 * i] = c0[y[i] & 15][c[i >> 1]];
        px[2 * i + 1] = c1[y[i] >> 4][c[i >> 1]];
    }
}

// A v2 record row: 6-bit luma, four pixels in three bytes, low bits first; one chroma byte per four pixels.
static void out_clip6(uint16_t *px, int x0, const uint8_t *rec, int j, int width, int aud_n,
                      const uint16_t (*c0)[256], const uint16_t (*c1)[256]) {
    const uint8_t *y = rec + 2 * aud_n + j * (3 * width / 4), *c = rec + 2 * aud_n + 3 * width / 2;
    px += x0;
    for (int i = 0; i < width / 4; i++, y += 3, px += 4) {
        uint32_t v = y[0] | (uint32_t)y[1] << 8 | (uint32_t)y[2] << 16, k = c[i];
        px[0] = c0[v & 63][k];
        px[1] = c1[v >> 6 & 63][k];
        px[2] = c0[v >> 12 & 63][k];
        px[3] = c1[v >> 18][k];
    }
}

// levels luma steps: y_tab's 16 entries (v1), or 64 from y_tab[0] to y_tab[15] (v2).
static void clut_build(uint16_t (*lut0)[256], uint16_t (*lut1)[256], int levels, const int32_t *y_tab,
                       const int32_t *u_tab, const int32_t *v_tab) {
    for (int y = 0; y < levels; y++)
        for (int k = 0; k < 256; k++) {
            float Y = levels == 16 ? y_tab[y] / 256.0f
                                   : (y_tab[0] + (y_tab[15] - y_tab[0]) * (float)y / (levels - 1)) / 256.0f;
            float U = u_tab[k & 15] / 256.0f, V = v_tab[k >> 4] / 256.0f;
            lut0[y][k] = ire_code(Y + V) | ire_code(Y + U) << 8;
            lut1[y][k] = ire_code(Y - V) | ire_code(Y - U) << 8;
        }
}

static bool clips_init(void) {
    memcpy(&clip, clip_flash, sizeof clip);
    if (clip.magic != CLIPS_MAGIC || clip.version != 1 || clip.layout >= LAYOUT_COUNT ||
        clip.t_log2 > T_LOG2_MAX)
        return false;
    uint32_t n = MIN((1u << clip.t_log2) * layout_regions[clip.layout], AUD_CELLS);
    if (clip.aud_n != n || clip.width != W - AUD_W(n) || clip.rows > ROWS || clip.rows < ROWS - HS_BLANK_MAX ||
        clip.fields == 0 ||
        clip.fields % 4 || clip.pair_bytes != 2 * n + clip.width + clip.width / 4 ||
        clip.pair_bytes % 4 || clip.pair_bytes > CLIP_PAIR_MAX ||
        CLIPS_OFFSET + CLIP_HDR + (uint64_t)clip.fields * CLIP_PAIRS * clip.pair_bytes > DATA_OFFSET)
        return false;
    clip_ids = clip.fields * CLIP_PAIRS;
    clip_dma = dma_claim_unused_channel(true);
    if (flash_sum(clip_flash + CLIP_HDR, clip_ids * clip.pair_bytes / 4) != clip.sum) return false;
    clut_build(clut[0], clut[1], 16, clip.y_tab, clip.u_tab, clip.v_tab);
    return true;
}

// SD tape, field f at position leader + f: composed from the asset store (tape_store.c), or read verbatim
// (tools/clips.py --sd). Core 0 fills slots ahead of the field shown; core 1 pins the one it shows.
#define SDT_MAGIC 0x54534856            // "VHST"
#define SD_SLOTS 4                      // read-ahead: 4 fields of v1, 3 of v2 in sd_buf
#define SD_FIELD_BLOCKS_MAX 104         // T=2 halves: 438-byte pairs, 103 sectors a field
#define SD_NONE UINT32_MAX
typedef struct {
    uint32_t magic, version, t_log2, layout, width, rows, fields, pair_bytes, aud_n, field_sectors, first_sector, pad;
    int32_t y_tab[16], u_tab[16], v_tab[16];
} sdt_hdr_t;
static sdt_hdr_t sdt;
static bool sd_store;                   // the card is an asset store: fields are composed, and never end
static uint16_t sd_clut[2][64][256];    // v1 fills the first 16 luma levels
static uint8_t sd_buf[SD_SLOTS * SD_FIELD_BLOCKS_MAX * SD_RAW_BLOCK] __attribute__((aligned(4)));
static uint32_t sd_stride = SD_FIELD_BLOCKS_MAX * SD_RAW_BLOCK; // bytes a slot
static int sd_slots = SD_SLOTS;
#define SD_SLOT(s) (sd_buf + (uint32_t)(s) * sd_stride)
static uint8_t sd_hdr_raw[SD_RAW_BLOCK] __attribute__((aligned(4)));
static volatile uint32_t sd_slot_f[SD_SLOTS] = {SD_NONE, SD_NONE, SD_NONE, SD_NONE};    // set once read
static volatile int sd_show = -1, sd_hold = -1;     // core 1's slot, and the one it is about to take
static volatile uint32_t sd_want = SD_NONE;         // field core 1 wants now
static int sd_reading = -1;
static uint32_t sd_reading_f;

static bool sd_card_up;                 // the last sd_tape_init found a card, tape or not

static bool sd_tape_init(void) {
    for (int s = 0; s < SD_SLOTS; s++) sd_slot_f[s] = SD_NONE;
    sd_reading = -1;
    sd_card_up = sd_init(ctl.sd_div) && sd_read(0, 1, sd_hdr_raw);
    if (!sd_card_up) return false;
    uint32_t magic;
    memcpy(&magic, sd_hdr_raw, 4);
    ctl.sd_hdr_magic = magic;
    sd_store = magic == STORE_MAGIC;
    if (sd_store) {
        store_geom_t sg;
        if (!store_init(sd_hdr_raw, &sg)) return false;
        sdt = (sdt_hdr_t){SDT_MAGIC, 1, sg.t_log2, sg.layout, sg.width, sg.rows, SD_NONE - SD_SLOTS,
                          2 * sg.aud_n + sg.width + sg.width / 4, sg.aud_n};
        memcpy(sdt.y_tab, sg.y_tab, sizeof sdt.y_tab);
        memcpy(sdt.u_tab, sg.u_tab, sizeof sdt.u_tab);
        memcpy(sdt.v_tab, sg.v_tab, sizeof sdt.v_tab);
    } else {
        memcpy(&sdt, sd_hdr_raw, sizeof sdt);
    }
    uint32_t n = MIN((1u << sdt.t_log2) * layout_regions[sdt.layout < LAYOUT_COUNT ? sdt.layout : 0], AUD_CELLS);
    uint32_t luma = sdt.version == 2 ? 3 * sdt.width / 2 : sdt.width;      // v2: 6-bit luma
    if (sdt.magic != SDT_MAGIC || (sdt.version != 1 && (sdt.version != 2 || sd_store)) ||
        sdt.layout >= LAYOUT_COUNT || sdt.t_log2 > T_LOG2_MAX ||
        sdt.aud_n != n || sdt.width != W - AUD_W(n) || sdt.width % 4 || sdt.rows > ROWS ||
        sdt.rows < ROWS - HS_BLANK_MAX || sdt.fields == 0 || sdt.pair_bytes != 2 * n + luma + sdt.width / 4 ||
        (!sd_store && (sdt.field_sectors * 512 < CLIP_PAIRS * sdt.pair_bytes ||
                       sdt.field_sectors * SD_RAW_BLOCK * 3 > sizeof sd_buf || sdt.first_sector == 0)))
        return false;
    sd_stride = sd_store ? SD_FIELD_BLOCKS_MAX * SD_RAW_BLOCK : sdt.field_sectors * SD_RAW_BLOCK;
    sd_slots = MIN(SD_SLOTS, (int)(sizeof sd_buf / sd_stride));
    clut_build(sd_clut[0], sd_clut[1], sdt.version == 2 ? 64 : 16, sdt.y_tab, sdt.u_tab, sdt.v_tab);
    ctl.sd_fields = sdt.fields;
    return true;
}

static uint32_t sd_end(void) {
    return sdt.fields;
}

static int sd_slot_of(uint32_t f) {
    for (int s = 0; s < sd_slots; s++)
        if (sd_slot_f[s] == f) return s;
    return -1;
}

// Core 1, at each field start: field f's slot, else the one already shown (a miss), else NULL.
static const uint8_t *sd_take(uint32_t f) {
    sd_want = f;
    int s = sd_slot_of(f);
    if (s >= 0) {
        sd_hold = s;
        __dmb();
        if (sd_slot_f[s] == f) {
            sd_show = s;
            sd_hold = -1;
            return SD_SLOT(s);
        }
        sd_hold = -1;
    }
    ctl.sd_miss = ctl.sd_miss + 1;
    s = sd_show;
    return s >= 0 && sd_slot_f[s] != SD_NONE ? SD_SLOT(s) : NULL;
}

// Core 0: a slot neither pinned nor holding a field in [want, want + sd_slots); the emptiest first.
static int sd_victim(uint32_t want) {
    int best = -1;
    for (int s = 0; s < sd_slots; s++) {
        uint32_t f = sd_slot_f[s];
        if (s == sd_show || s == sd_hold || (f >= want && f - want < (uint32_t)sd_slots)) continue;
        if (f == SD_NONE) return s;
        best = s;
    }
    return best;
}

// No card: retried every second (10 s if it answers without a tape); one that comes up takes over from the
// boot fallback, and SD_DOWN failed reads in a row drop it back to retrying.
#define SD_RETRY_US   1000000u
#define SD_NOTAPE_US 10000000u
#define SD_DOWN      8
static uint32_t sd_retry_at, sd_fails_run;
static bool sd_fallback;                // src is 1 only because no card was up at boot

static void sd_service(void) {
    if (!ctl.sd_ok && !ctl.sd_init && (int32_t)(time_us_32() - sd_retry_at) >= 0) {
        ctl.sd_init = 1;
        sd_retry_at = time_us_32() + SD_RETRY_US;
    }
    if (ctl.sd_init) {
        ctl.sd_ok = 0;
        if (sd_store) store_abort();
        else while (sd_reading >= 0 && sd_read_poll() == 0) tight_loop_contents();
        sd_reading = -1;
        ctl.sd_ok = sd_tape_init();
        ctl.sd_init = 0;
        sd_fails_run = 0;
        if (!ctl.sd_ok && sd_card_up) sd_retry_at = time_us_32() + SD_NOTAPE_US;
        if (ctl.sd_ok && sd_fallback && ctl.src == 1) ctl.src = 3;
        if (ctl.sd_ok) sd_fallback = false;
    }
    if (ctl.sd_test && sd_reading < 0) {
        ctl.sd_test_bad = sd_status_test(ctl.sd_test);
        ctl.sd_test = 0;
    }
    ctl.sd_reads = sd_stats.reads;
    ctl.sd_crc = sd_stats.crc_errors;
    ctl.sd_timeouts = sd_stats.timeouts;
    ctl.sd_cmd_err = sd_stats.cmd_errors;
    ctl.sd_stage = sd_stats.stage;
    ctl.sd_resp = sd_stats.resp;
    ctl.sd_fail = sd_stats.fail;
    ctl.st_assets = store_stats.assets;
    ctl.st_entries = store_stats.entries;
    ctl.st_dead = store_stats.dead;
    ctl.st_reads = store_stats.reads;
    ctl.st_fails = store_stats.fails;
    if (!ctl.sd_ok) return;
    if (sd_reading >= 0) {
        int r = sd_store ? store_compose(SD_SLOT(sd_reading), sd_reading_f) : sd_read_poll();
        if (r == 0) return;
        if (r > 0) {
            __dmb();
            sd_slot_f[sd_reading] = sd_reading_f;
            sd_fails_run = 0;
        } else if (++sd_fails_run >= SD_DOWN) {
            ctl.sd_ok = 0;
            sd_retry_at = time_us_32() + SD_RETRY_US;
        }
        sd_reading = -1;
        return;
    }
    uint32_t want = sd_want;
    for (uint32_t t = want; t < sd_end() && t - want < (uint32_t)sd_slots; t++) {
        if (sd_slot_of(t) >= 0) continue;
        int v = sd_victim(want);
        if (v < 0) return;
        sd_slot_f[v] = SD_NONE;
        __dmb();
        if (v == sd_show || v == sd_hold) return;
        if (sd_store || sd_read_start(sdt.first_sector + t * sdt.field_sectors, sdt.field_sectors, SD_SLOT(v))) {
            sd_reading = v;
            sd_reading_f = t;
        }
        return;
    }
}

// boot2 runs the flash at clk_sys / 2: keep the SDK's 150 MHz SCK and sample timing at 315 MHz.
static void flash_timing(void) {
    hw_write_masked(&qmi_hw->m[0].timing, 4u << QMI_M0_TIMING_CLKDIV_LSB | 4u << QMI_M0_TIMING_RXDELAY_LSB,
                    QMI_M0_TIMING_CLKDIV_BITS | QMI_M0_TIMING_RXDELAY_BITS);
}

// VHS channel model, in IRE * 256: one-pole luma filter per pixel (samples interpolated between),
// c_poles one-pole chroma filters per pixel pair with noise ahead of them, a chroma delay, then the
// deck's average of this line's chroma with the previous one of the field. Filter state starts each
// line at blanking. Core 1 models the first SPLIT pairs of a line, core 0 the rest, starting WARM
// pairs early so its filters have settled; they forget in about 7 pairs. Core 0 also turns ctl into
// coefficients, which core 1 picks up at each frame start.
typedef struct {
    int32_t ay, ac, y_noise, c_noise, delay, poles;
    bool comb;
} vhs_t;
#define PAIRS (W / 2)
#define SPLIT 100
#define WARM 24
#define DELAY_MAX 8                     // pairs; must stay well under WARM
#define NOISE_MAX 200                   // 20 IRE: keeps every sample inside code_lut
#define LUT_LO (96 * 16)                // code_lut covers -96 to +192 IRE in 1/16 IRE
static vhs_t vp, vp_next[2];
static volatile uint32_t vp_pub;        // vp_next index core 0 last completed, + 1
static uint8_t code_lut[288 * 16];
static int8_t  ntab[4096];
static int32_t cu[2][PAIRS], cv[2][PAIRS];  // per core: chroma before the delay
static int32_t pu[PAIRS], pv[PAIRS];        // previous line's delayed chroma, for the deck comb
static uint32_t rng = 0x12345678;

static uint16_t *volatile job_px;
static volatile int job_lp;
static volatile uint32_t job_nofs, job_seq, job_done;
static int32_t ire_of_code[64];         // IRE * 256

static inline uint32_t xs32(void) {
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    return rng;
}

#define CODE(ire256) code_lut[((ire256) >> 4) + LUT_LO]

static void vhs_setup(void) {
    for (int i = 0; i < (int)sizeof code_lut; i++)
        code_lut[i] = ire_code((i - LUT_LO) / 16.0f);
    for (int i = 0; i < (int)sizeof ntab; i++)
        ntab[i] = (int8_t)xs32();
    for (int c = 0; c < 64; c++)
        ire_of_code[c] = (int32_t)lroundf((c * 180.0f / 63.0f - 40.0f) * 256.0f);
}

static vhs_t vhs_coeffs(void) {
    vhs_t v;
    int n = ctl.c_poles == 1 ? 1 : 2;
    float fy = ctl.y_bw_khz * 1e3f, fc = ctl.c_bw_khz * 1e3f / sqrtf(powf(2.0f, 1.0f / n) - 1.0f);
    v.ay = (int32_t)(32768.0f * (1.0f - expf(-2.0f * (float)M_PI * fy / (FS_HZ / 2))));
    v.ac = (int32_t)(32768.0f * (1.0f - expf(-2.0f * (float)M_PI * fc / (FS_HZ / 4))));
    v.poles = n;
    v.delay = MIN((int32_t)lroundf(ctl.c_delay_ns * 1e-9f * FS_HZ / 4), DELAY_MAX);
    v.comb = ctl.c_comb != 0;
    v.y_noise = (int32_t)MIN(ctl.y_noise, NOISE_MAX) * 256 / 10;
    v.c_noise = (int32_t)MIN(ctl.c_noise, NOISE_MAX) * 256 / 10;
    return v;
}

static void vhs_clear_prev(void) {
    memset(pu, 0, sizeof pu);
    memset(pv, 0, sizeof pv);
}

// Pairs k_start..PAIRS-1 or ..k_end-1 run the filters; output starts at k_out.
static void vhs_span(int core, uint16_t *px, int lp, int k_start, int k_out, int k_end, uint32_t nofs) {
    const int32_t ay = vp.ay, ac = vp.ac, yn = vp.y_noise, cn = vp.c_noise, d = vp.delay;
    const bool two = vp.poles == 2, comb = vp.comb;
    int32_t *cuc = cu[core], *cvc = cv[core];
    int32_t y = 0, u1 = 0, u2 = 0, v1 = 0, v2 = 0;
    for (int k = k_start; k < k_end; k++) {
        const int32_t *a = yuv[idx[2 * k]], *b = yuv[idx[2 * k + 1]];
        const int8_t *n = &ntab[(nofs + 4 * (uint32_t)k) & (sizeof ntab - 4)];
        u1 += (ac * (((a[1] + b[1]) >> 1) + (n[0] * cn >> 7) - u1)) >> 15;
        v1 += (ac * (((a[2] + b[2]) >> 1) + (n[1] * cn >> 7) - v1)) >> 15;
        if (two) {
            u2 += (ac * (u1 - u2)) >> 15;
            v2 += (ac * (v1 - v2)) >> 15;
        } else {
            u2 = u1;
            v2 = v1;
        }
        cuc[k] = u2;
        cvc[k] = v2;
        int32_t y0 = y;
        y += (ay * (a[0] - y)) >> 15;
        int32_t ya = y;
        y += (ay * (b[0] - y)) >> 15;
        if (k < k_out) continue;

        int32_t u = k - d >= k_start ? cuc[k - d] : 0, v = k - d >= k_start ? cvc[k - d] : 0;
        if (comb) {
            int32_t uo = (u + pu[k]) >> 1, vo = (v + pv[k]) >> 1;
            pu[k] = u;
            pv[k] = v;
            u = uo;
            v = vo;
        }
        if (lp) {
            u = -u;
            v = -v;
        }
        int32_t na = n[2] * yn >> 7, nb = n[3] * yn >> 7;
        px[2 * k] = CODE(((y0 + ya) >> 1) + na + v) | CODE(ya + na + u) << 8;
        px[2 * k + 1] = CODE(((ya + y) >> 1) + nb - v) | CODE(y + nb - u) << 8;
    }
}

// Core 1's share starts at pair k_out (warming up from k_out - WARM); core 0's runs meanwhile, until
// out_vhs_end.
static void out_vhs_begin(uint16_t *px, int lp, int k_out) {
    uint32_t nofs = xs32() & ~3u;
    job_px = px;
    job_lp = lp;
    job_nofs = nofs;
    __dmb();
    job_seq = job_seq + 1;
    __sev();
    vhs_span(1, px, lp, MAX(k_out - WARM, 0), k_out, SPLIT, nofs);
}

static void out_vhs_end(void) {
    while (job_done != job_seq)
        tight_loop_contents();
    __dmb();
}

// Controller: the deck's front panel. The test-pattern settings are SWD-only (scripts/gcv2_ctl.sh).
static void pad_actions(uint16_t pressed) {
    static const struct { uint16_t bit; uint8_t code; } keys[] = {
        {PAD_START, VCR_POWER}, {PAD_A, VCR_PLAY}, {PAD_B, VCR_STOP}, {PAD_CLEFT, VCR_REW},
        {PAD_CRIGHT, VCR_FF}, {PAD_Z, VCR_PAUSE}, {PAD_L, VCR_EJECT}, {PAD_R, VCR_INSERT},
    };
    for (unsigned k = 0; k < sizeof keys / sizeof keys[0]; k++)
        if (pressed & keys[k].bit) vcr_post(keys[k].code);
}

// Runs from RAM like everything else, and only while the deck shows no tape, so nothing reads flash.
static void save_tape_state(void) {
    static vcr_save_t s;
    static uint8_t page[FLASH_PAGE_SIZE];
    if (!vcr_save_due(&s)) return;
    memset(page, 0xFF, sizeof page);
    memcpy(page, &s, sizeof s);
    flash_range_erase(SAVE_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(SAVE_OFFSET, page, FLASH_PAGE_SIZE);
    flash_timing();                     // boot2's setup comes back with the flash write
    ctl.vcr_saves = ctl.vcr_saves + 1;
}

static void core0_vhs_worker(void) {
    uint32_t next_coeffs = 0, i = 0;
    uint16_t held = 0, now_held;
    unsigned connected;
    for (;;) {
        if (job_seq != job_done) {
            __dmb();
            vhs_span(0, job_px, job_lp, SPLIT - WARM, SPLIT, PAIRS, job_nofs);
            __dmb();
            job_done = job_seq;
            continue;
        }
        if (pad_service(&now_held, &connected))
            pad_actions(now_held & ~held);
        held = now_held;
        ctl.pad_buttons = held;
        ctl.pad_connected = connected;
        ctl.pad_edges = pad_edges();
        vcr_ir_service();
        save_tape_state();
        sd_service();
        ctl.ir_frames = vcr_ir_frames;
        ctl.ir_bad = vcr_ir_bad;
        ctl.ir_last = vcr_ir_last;
        if ((int32_t)(time_us_32() - next_coeffs) >= 0) {
            vp_next[i] = vhs_coeffs();
            __dmb();
            vp_pub = i + 1;
            i ^= 1;
            next_coeffs = time_us_32() + 20000;
        }
    }
}

typedef struct {
    int h, H;                           // this line's start and the field's length, in half-lines from the field start
    vcr_field_t vf;                     // the deck's state for this field
    uint32_t nf;                        // fields sent
    uint32_t vbi_from, vbi_keep, tag_keep;
    int vbi_min, vbi_max;
    int rows, t_log2, lay, guard_px, guard_rows, hs_lines, hs_shift;
    bool il, plain, vhs, clips, data, tag_on, sd;
    const uint8_t *sd_rec;              // the SD field shown; NULL: black
    int dw, drows, dk, d_skew, d_burst, d_drop_left;
    uint32_t d_drop, d_s;               // d_s: data packets sent
    uint8_t d_black, d_white;
    int32_t d_ay;                       // luma one-pole per pair of DAC samples, Q15
    uint32_t cf;                        // clip field shown by the current field
    int aud;
    uint32_t aud_lin;
    int32_t aud_mid16, aud_amp16, aud_lead;
    int aud_n, aud_x0;                  // audio cells, and where the picture starts (0 without audio)
    uint32_t fc;
    uint32_t gl;                        // global line count: its parity is the line's subcarrier phase
    uint32_t lf;                        // LTC frame being sent
    int ltc_line, ltc_len;              // lines into the word, and its length
    int ltc_h, ltc_lvl;                 // last half-bit sent, and the level after it (+-1)
    uint8_t ltc_word[10];
    bool vitc_on, ptag_on;
    uint8_t vitc[VITC_BITS + 1];        // this field's VITC bits, then a 0
    uint8_t ptag[PTAG_CELLS];           // this field's position tag cells
    int32_t ltc_amp, left_amp;
    uint32_t left_ph, left_inc;
    int32_t a_rel;                      // next audio frame's time after this line's start, half sys clocks
    uint32_t a_n;                       // next audio frame
} gen_t;

static uint32_t field_cycles, pic_cycles, pic_lines;
static bool last_picture;

static uint32_t a_ring[A_RING] __attribute__((aligned(A_RING * 4)));  // (R << 16) | L

// SMPTE 12M at 30 fps non-drop; bit i is w[i >> 3] >> (i & 7) & 1, sent LSB first. Digit k sits in
// bits 8k..8k+3 and user nibble k in 8k+4..8k+7.
static void ltc_make(uint8_t w[10], uint32_t frame, uint32_t user) {
    uint32_t f = frame % 30, s = frame / 30 % 60, m = frame / 1800 % 60, h = frame / 108000 % 24;
    const uint32_t d[8] = {f % 10, f / 10, s % 10, s / 10, m % 10, m / 10, h % 10, h / 10};
    int ones = 0;
    for (int k = 0; k < 8; k++) {
        w[k] = (uint8_t)(d[k] | (user >> 4 * k & 15) << 4);
        ones += __builtin_popcount(w[k]);
    }
    w[8] = 0xFC;                        // sync word 0011111111111101
    w[9] = 0xBF;
    if ((ones + 13) & 1) w[3] |= 0x08;  // bit 27: an even count of ones, so every word starts alike
}

// SMPTE 12M VITC: 9 groups of sync 1 0 and 8 bits LSB first, the LTC's data bits with bit 27 as the
// field mark (1 in the frame's second field); group 9 is the CRC of x^8 + 1 over all 90 bits.
static void vitc_make(uint8_t b[VITC_BITS + 1], uint32_t frame, int field, uint32_t user) {
    uint8_t w[10], crc = 0;
    ltc_make(w, frame, user);
    w[3] = (uint8_t)((w[3] & ~0x08) | (field ? 0x08 : 0));
    for (int g = 0; g < 9; g++) {
        b[10 * g] = 1;
        b[10 * g + 1] = 0;
        for (int i = 0; i < 8; i++) b[10 * g + 2 + i] = g < 8 ? w[g] >> i & 1 : 0;
    }
    for (int k = 0; k < 82; k++) crc ^= (uint8_t)(b[k] << (k & 7));
    for (int k = 82; k < VITC_BITS; k++) b[k] = crc >> (k & 7) & 1;
    b[VITC_BITS] = 0;
}

// Position tag: the low 16 bits of the VITC position (2 x frame + field mark), LSB
// first in two bytes, then CRC-8/SMBUS (poly 0x07) of the bytes. A paused or cueing deck keeps only
// pieces of each line, so it is short, repeated along the line and on eight lines.
static void ptag_make(uint8_t c[PTAG_CELLS], uint32_t pos) {
    uint8_t d[3] = {(uint8_t)pos, (uint8_t)(pos >> 8), 0};
    for (int i = 0; i < 2; i++) {
        d[2] ^= d[i];
        for (int k = 0; k < 8; k++) d[2] = (uint8_t)(d[2] & 0x80 ? d[2] << 1 ^ 0x07 : d[2] << 1);
    }
    for (int g = 0; g < 3; g++) {
        c[10 * g] = 1;
        c[10 * g + 1] = 0;
        for (int i = 0; i < 8; i++) c[10 * g + 2 + i] = d[g] >> i & 1;
    }
    c[30] = c[31] = 0;
}

// Each cell a flat PTAG_CELL samples at 0 or 80 IRE; the first sample after a change sits halfway.
static void ptag_line(uint8_t *s, const uint8_t c[PTAG_CELLS]) {
    int lo = ire_code(0.0f), hi = ire_code(80.0f), mid = (lo + hi + 1) / 2;
    s += PTAG_X0;
    for (int n = 0, prev = 0; n < PTAG_COPIES * PTAG_CELLS; n++) {
        int b = c[n % PTAG_CELLS];
        memset(s, b ? hi : lo, PTAG_CELL);
        if (b != prev) s[0] = (uint8_t)mid;
        prev = b;
        s += PTAG_CELL;
    }
}

// Bit k spans 910 units from 910 k, sample i 115 from 115 i; levels 0 and 80 IRE, eased by [1 2 1] over
// each sample's coverage. Cells are ~8 samples, so only the three samples around a 0/1 edge are eased.
static void vitc_line(uint8_t *s, const uint8_t b[VITC_BITS + 1]) {
    static uint16_t at[VITC_BITS + 1];  // sample holding bit k's start
    static uint8_t pre[VITC_BITS + 1];  // of its 115 units, those still in bit k - 1
    if (!at[1])
        for (int k = 0; k <= VITC_BITS; k++) {
            at[k] = (uint16_t)(910 * k / 115);
            pre[k] = (uint8_t)(910 * k - 115 * at[k]);
        }
    int lo = ire_code(0.0f), hi = ire_code(80.0f);
    s += VITC_X0;
    for (int k = 0; k < VITC_BITS; k++) memset(s + at[k], b[k] ? hi : lo, (size_t)(at[k + 1] - at[k]));
    s[0] = (uint8_t)(lo + ((hi - lo) * 345 + 230) / 460);   // bit 0 is always 1; the blanking before it stays
    for (int k = 1; k <= VITC_BITS; k++) {
        if (b[k - 1] == b[k]) continue;
        int i = at[k], A = 115 * b[k - 1], B = 115 * b[k], c = pre[k] * b[k - 1] + (115 - pre[k]) * b[k];
        s[i - 1] = (uint8_t)(lo + ((hi - lo) * (3 * A + c) + 230) / 460);
        s[i] = (uint8_t)(lo + ((hi - lo) * (A + 2 * c + B) + 230) / 460);
        s[i + 1] = (uint8_t)(lo + ((hi - lo) * (c + 3 * B) + 230) / 460);
    }
}

// The audio frames falling in this line's time, into the DMA ring. The LTC word spans its video frame,
// so a biphase-mark half-bit is ltc_len lines / 160.
static void audio_line(gen_t *g) {
    const int32_t len = g->ltc_len * 1001;  // LINE_T / 40 units per line: pos * 4 / len is the half-bit
    while (g->a_rel < LINE_T) {
        int32_t pos = g->ltc_line * LINE_T + g->a_rel;
        int h = MIN(pos * 4 / len, 159);
        bool edge = false;
        while (g->ltc_h < h) {
            int j = ++g->ltc_h;
            if (!(j & 1) || (g->ltc_word[j >> 4] >> ((j >> 1) & 7) & 1)) {
                g->ltc_lvl = -g->ltc_lvl;
                edge = true;
            }
        }
        int16_t r = edge ? 0 : (int16_t)(g->ltc_lvl * g->ltc_amp);
        int16_t l = (int16_t)(sine[g->left_ph >> 20] * g->left_amp >> 15);
        g->left_ph += g->left_inc;
        a_ring[g->a_n++ % A_RING] = (uint32_t)(uint16_t)r << 16 | (uint16_t)l;
        g->a_rel += A_T;
    }
    g->a_rel -= LINE_T;
}

// Data tape: per cell width 4-7 the blob header gives the packet count, the image's
// CRC-32 and the session id.
#define DATA_MAGIC 0x44534856           // "VHSD"
#define DATA_HDR 256
#define DATA_LEAD 16
#define DATA_SPAN 720
#define DATA_RUNIN 5                    // 4 white cells, 1 black
#define DATA_B_MAX 21
#define DATA_W_MIN 4
#define DATA_W_MAX 7
#define DATA_K_MAX 8
#define IDX_HEADER 0xFFF0               // + K - 1
#define IDX_PARITY 0xFF00               // + group
typedef struct {
    uint32_t magic, version, len, sum, n[4], crc[4];
    uint8_t session[4];
    uint32_t leader;                    // nonzero: the game's leader in fields (datatape.py --leader), else ctl.leader
    uint32_t programme;                 // bit 31: ctl.src after the leader is the low bits (datatape.py --programme)
} data_hdr_t;
static const uint8_t *const data_flash = (const uint8_t *)(XIP_BASE + DATA_OFFSET);
static data_hdr_t data;

static uint32_t tape_leader(void) {
    return ctl.data_ok && data.leader ? data.leader : ctl.leader;
}
static uint8_t data_pk[DATA_B_MAX], data_par[DATA_K_MAX][DATA_B_MAX];
#define DNOISE 1024
static int16_t dnoise[DNOISE];          // ntab scaled by y_noise, IRE * 256
static int32_t dnoise_yn = -1;
static uint8_t dkey[ROWS][DATA_B_MAX];  // scrambler keystream per row, MSB first
static uint16_t crc_tab[256];

static int data_line_bytes(int w) { return (DATA_SPAN / w - DATA_RUNIN) / 8; }

static bool data_init(void) {
    memcpy(&data, data_flash, sizeof data);
    if (data.magic != DATA_MAGIC || data.version != 1 || data.len == 0 ||
        DATA_OFFSET + DATA_HDR + (uint64_t)data.len > SAVE_OFFSET)
        return false;
    for (int k = 0; k < 4; k++) {
        uint32_t d = (uint32_t)data_line_bytes(DATA_W_MIN + k) - 4;
        if (data.n[k] != (data.len + d - 1) / d || data.n[k] > 32768) return false;
    }
    for (int r = 0; r < ROWS; r++) {
        uint32_t lf = (uint32_t)r << 8 | (~(uint32_t)r & 0xFF);
        for (int i = 0; i < DATA_B_MAX; i++) {
            uint8_t v = 0;
            for (int b = 7; b >= 0; b--) {
                uint32_t key = lf & 1;
                lf = lf >> 1 ^ (key ? 0xB400 : 0);
                v |= (uint8_t)(key << b);
            }
            dkey[r][i] = v;
        }
    }
    for (int i = 0; i < 256; i++) {
        uint16_t c = (uint16_t)(i << 8);
        for (int k = 0; k < 8; k++) c = c & 0x8000 ? (uint16_t)(c << 1 ^ 0x1021) : (uint16_t)(c << 1);
        crc_tab[i] = c;
    }
    return flash_sum(data_flash + DATA_HDR, (data.len + 3) / 4) == data.sum;
}

static uint16_t crc16(const uint8_t *p, int n) {
    uint16_t c = 0xFFFF;
    while (n--) c = (uint16_t)(c << 8 ^ crc_tab[(c >> 8 ^ *p++) & 0xFF]);
    return c;
}

// One data-tape row (datatape.py Encoder) into the active samples s, then the test
// impairments.
static void data_line(gen_t *g, uint8_t *s, int row) {
    int w = g->dw, k = w - DATA_W_MIN, B = data_line_bytes(w), D = B - 4, last = g->drows - g->dk - 1;
    uint8_t *pk = data_pk, hi = g->d_white, lo = g->d_black;
    memset(s, lo, 2 * W);
    if (row >= g->drows) return;

    uint16_t idx;
    memset(pk + 2, 0, (size_t)D);
    if (row == 0) {
        idx = (uint16_t)(IDX_HEADER + g->dk - 1);
        uint32_t n = data.n[k], crc = data.crc[k];
        pk[2] = data.session[k];
        pk[3] = (uint8_t)g->drows;
        pk[4] = (uint8_t)n;
        pk[5] = (uint8_t)(n >> 8);
        memcpy(pk + 6, &crc, 4);
        memset(data_par, 0, sizeof data_par);
    } else if (row > last) {
        int p = row - last - 1;
        idx = (uint16_t)(IDX_PARITY + p);
        memcpy(pk + 2, data_par[p], (size_t)D);
    } else {
        idx = (uint16_t)(g->d_s % data.n[k]);
        g->d_s++;
        uint32_t off = (uint32_t)idx * (uint32_t)D;
        if (off < data.len) memcpy(pk + 2, data_flash + DATA_HDR + off, MIN((uint32_t)D, data.len - off));
        uint8_t *par = data_par[(row - 1) % g->dk];
        for (int i = 0; i < D; i++) par[i] ^= pk[2 + i];
    }
    pk[0] = (uint8_t)(idx >> 8);
    pk[1] = (uint8_t)idx;
    uint16_t c = crc16(pk, B - 2);
    pk[B - 2] = (uint8_t)(c >> 8);
    pk[B - 1] = (uint8_t)c;

    // Run-in 1 1 1 1 0, then the scrambled bits; a cell is 4-7 samples, two overlapping word stores.
    int lead = DATA_LEAD + (g->d_skew ? (int)(xs32() % (uint32_t)(2 * g->d_skew + 1)) - g->d_skew : 0);
    uint8_t *p = s + lead;
    const uint32_t v4[2] = {lo * 0x01010101u, hi * 0x01010101u};
    for (int i = 0; i < DATA_RUNIN; i++, p += w) {
        memcpy(p, &v4[i < 4], 4);
        memcpy(p + w - 4, &v4[i < 4], 4);
    }
    const uint8_t *key = dkey[row];
    for (int i = 0; i < B; i++) {
        uint32_t v = pk[i] ^ key[i];
        for (int b = 7; b >= 0; b--, p += w) {
            const uint32_t *cv = &v4[(v >> b) & 1];
            memcpy(p, cv, 4);
            memcpy(p + w - 4, cv, 4);
        }
    }
    if (g->vhs) {                       // the channel model, per pair of samples to fit the line time
        const int32_t a = g->d_ay, l0 = ire_of_code[lo], l1 = ire_of_code[hi];
        const int32_t lev[3] = {l0, (l0 + l1) >> 1, l1};
        const uint32_t o = xs32();
        uint16_t *s2 = (uint16_t *)s;   // ACTIVE_START is even
        int32_t y = l0;
        for (int i = 0; i < W; i++) {
            uint32_t v = s2[i];
            y += (a * (lev[((v & 0xFF) == hi) + ((v >> 8) == hi)] - y)) >> 15;
            s2[i] = (uint16_t)(CODE(y + dnoise[(o + (uint32_t)i) & (DNOISE - 1)]) * 0x0101u);
        }
    }

    if (g->d_drop_left > 0 || (g->d_drop && xs32() % 1000 < g->d_drop)) {
        g->d_drop_left = g->d_drop_left > 0 ? g->d_drop_left - 1 : g->d_burst - 1;
        int a = 16 + (int)(xs32() % 584), len = 60 + (int)(xs32() % 300);
        memset(s + a, xs32() & 1 ? hi : lo, (size_t)MIN(len, 2 * W - a));
    }
}

// Pause and cue reproduce nothing of the top content lines and only a piece of the rest, now and then a
// whole line: the rest of the line sits at blanking.
static void vbi_damage(gen_t *g, uint8_t *s, int lc) {
    const int a0 = BURST_START + BURST_LEN + 8, a1 = SPL - 16;
    const uint8_t blank = ire_code(0.0f);
    if (lc < (int)g->vbi_from) {
        memset(s + a0, blank, a1 - a0);
        return;
    }
    if (xs32() % 1000 < (lc == TAG_LINE_F1 ? g->tag_keep : g->vbi_keep)) return;
    int len = g->vbi_min + (int)(xs32() % (uint32_t)(g->vbi_max - g->vbi_min + 1));
    int at = a0 + (int)(xs32() % (uint32_t)(a1 - a0 - len + 1));
    memset(s + a0, blank, (size_t)(at - a0));
    memset(s + at + len, blank, (size_t)(a1 - at - len));
}

static uint8_t bar_noise[2048];         // cue noise bars: luma only, 10-70 IRE

// Latches ctl, steps the deck and sets up the field's tag, timecode and data from its tape position.
static void field_start(gen_t *g, int phase) {
    g->il = ctl.interlaced != 0;
    if (ctl.ltc_seek) {                 // the tape, and with it the tag count and the clips, moves to the LTC frame
        vcr_seek(2 * (ctl.ltc_seek - 1));
        ctl.ltc_seek = 0;
    }
    uint32_t cmd = ctl.vcr_cmd;
    if (cmd) {
        ctl.vcr_cmd = 0;
        if (ctl.vcr) vcr_command((int)cmd - 1);
    }
    vcr_fast_wind = ctl.vcr_fast != 0;
    vcr_field(&g->vf, phase, g->il, ctl.vcr == 0, ctl.vcr_len);
    g->H = g->vf.H;
    uint32_t pos = g->vf.pos;
    ctl.vcr_state = (uint32_t)vcr_state();
    ctl.vcr_pos = pos;
    ctl.vcr_cass = (uint32_t)vcr_cassette();
    bool movie = g->vf.cass == CASS_MOVIE;  // someone's recording: the clips with nothing of ours on them

    {                                   // ctl, latched once per field
        g->data = !movie && ctl.data_ok != 0 && (ctl.src == 2 || pos < tape_leader());  // the leader: the game image at the tape's start
        g->clips = (movie || ctl.src == 1) && ctl.clips_ok != 0 && !g->data;
        g->sd = !movie && ctl.src == 3 && !g->data;
        bool sdt_ok = g->sd && ctl.sd_ok;
        g->tag_on = !movie;
        g->t_log2 = g->clips ? (int)clip.t_log2 : sdt_ok ? (int)sdt.t_log2 : g->data || g->sd ? 0
                  : (int)MIN(ctl.t_log2, T_LOG2_MAX);
        g->lay = g->clips ? (int)clip.layout : sdt_ok ? (int)sdt.layout : g->data || g->sd ? LAYOUT_FULL
               : ctl.layout < LAYOUT_COUNT ? (int)ctl.layout : LAYOUT_FULL;
        g->rows = g->clips ? (int)clip.rows : sdt_ok ? (int)sdt.rows : ROWS - (int)MIN(ctl.hs_blank, HS_BLANK_MAX);
        g->plain = ctl.plain != 0;
        g->guard_px = (int)ctl.guard_px;
        g->guard_rows = (int)ctl.guard_rows;
        g->vhs = ctl.vhs != 0 && !g->clips && !g->sd;
        g->hs_lines = (int)ctl.hs_lines;
        g->hs_shift = (int)ctl.hs_shift_px & ~1;
        g->aud = movie ? AUD_OFF : g->clips || sdt_ok ? AUD_CLIP : g->data || g->sd ? AUD_OFF
               : ctl.aud <= AUD_SILENT ? (int)ctl.aud : AUD_OFF;
        if (g->data) {
            g->dw = (int)MIN(MAX(ctl.data_width, DATA_W_MIN), DATA_W_MAX);
            g->dk = (int)MIN(MAX(ctl.data_parity, 1u), (uint32_t)DATA_K_MAX);
            g->drows = (int)MIN(MAX(ctl.data_rows, (uint32_t)g->dk + 2), (uint32_t)ROWS);
            g->d_black = ire_code(MIN(ctl.data_black, 1000u) / 10.0f);
            g->d_white = ire_code(MIN(ctl.data_white, 1000u) / 10.0f);
            g->d_drop = ctl.data_drop;
            g->d_burst = (int)MIN(MAX(ctl.data_burst, 1u), 64u);
            g->d_skew = (int)MIN(ctl.data_skew, 12u);
            g->d_ay = (int32_t)(32768.0f * (1.0f - expf(-2.0f * (float)M_PI * ctl.y_bw_khz * 1e3f / (FS_HZ / 2))));
            ctl.data_packets = data.n[g->dw - DATA_W_MIN];
        }
        if (g->data && g->vhs && vp.y_noise != dnoise_yn) {
            dnoise_yn = vp.y_noise;
            for (int i = 0; i < DNOISE; i++) dnoise[i] = (int16_t)(ntab[i] * dnoise_yn >> 7);
        }
        g->aud_amp16 = (int32_t)MIN(ctl.aud_level, 400) * 56 / 100;
        g->aud_mid16 = (int32_t)MIN(ctl.aud_mid16, 1000u);
        g->aud_lead = (int32_t)MIN(ctl.aud_lead, 8u);
        g->aud_n = MIN((1 << g->t_log2) * layout_regions[g->lay], AUD_CELLS);
        g->aud_x0 = g->aud != AUD_OFF ? AUD_W(g->aud_n) : 0;
        if (ctl.aud_lin != g->aud_lin) {
            g->aud_lin = ctl.aud_lin;
            aud_lev_setup(g->aud_lin != 0);
        }
        float aud_fs = g->rows * FS_HZ / SPL / (g->il ? 262.5f : 262.0f);
        for (int k = 0; k < AUD_CELLS; k++)
            aud_inc[k] = (uint32_t)(aud_hz[k] / aud_fs * 4294967296.0f);
        uint32_t pub = vp_pub;
        if (pub) vp = vp_next[pub - 1];
        g->vitc_on = !movie && ctl.vitc != 0;
        g->ptag_on = !movie && ctl.ptag != 0;
        g->ltc_amp = !movie && ctl.ltc ? (int32_t)MIN(ctl.ltc_amp, 32767u) : 0;
        g->left_amp = ctl.left_hz ? (int32_t)MIN(ctl.left_amp, 32767u) : 0;
        g->left_inc = (uint32_t)(MIN(ctl.left_hz, 20000u) / 48000.0f * 4294967296.0f);
        ctl.active_t_log2 = g->t_log2;
        ctl.active_layout = g->lay;
        ctl.active_interlaced = g->il;
        g->vbi_from = ctl.vbi_from;
        g->vbi_keep = ctl.vbi_keep;
        g->tag_keep = ctl.tag_keep;
        g->vbi_max = (int)MIN(MAX(ctl.vbi_max, 1u), 600u);
        g->vbi_min = (int)MIN(ctl.vbi_min, (uint32_t)g->vbi_max);
    }

    ctl.fields = ++g->nf;
    ctl.cycles_max = field_cycles;
    ctl.cycles_avg = pic_lines ? pic_cycles / pic_lines : 0;
    field_cycles = pic_cycles = pic_lines = 0;
    g->fc = pos;
    if (g->clips) g->cf = pos % clip.fields;
    uint32_t lead = tape_leader();
    uint32_t sf = g->sd && ctl.sd_ok && pos >= lead ? pos - lead : SD_NONE;
    g->sd_rec = sf < sd_end() ? sd_take(sf) : NULL;
    if (sf >= sd_end())                 // during the leader, the card's first fields are read ahead
        sd_want = ctl.src == 3 && ctl.sd_ok && pos < lead ? 0 : SD_NONE;
    if (g->data) g->d_s = pos * (uint32_t)(g->drows - 1 - g->dk);
    if (!(pos & 1)) {
        g->lf = pos / 2;
        ltc_make(g->ltc_word, g->lf, ctl.ltc_user);
        ctl.ltc_frame = g->lf;
        g->ltc_line = 0;
        g->ltc_len = g->il ? 525 : 524;
        g->ltc_h = -1;
    }
    if (!g->vf.hifi) g->ltc_amp = g->left_amp = 0;
    if (g->vitc_on) vitc_make(g->vitc, pos / 2, (int)(pos & 1), ctl.ltc_user);
    if (g->ptag_on) ptag_make(g->ptag, pos);
}

static void render_next(gen_t *g, uint16_t *dst) {
    if (g->h >= g->H) {
        g->h -= g->H;
        field_start(g, 0);
    }
    int h = g->h, lp = g->gl & 1;
    bool mid = h + 1 >= g->H;           // the next field starts halfway along this line
    char k0 = slot_kind(h, true), k1 = mid ? slot_kind(0, false) : slot_kind(h + 1, false);
    int lc = (h >> 1) - g->vf.shift, row = lc - ROW0_F1;
    bool normal = k0 == 'N' && k1 == '0' && lc >= 0;

    if (g->vf.video == VID_NONE) {
        memset(dst, 0, sizeof tmpl[0][0]);
        normal = false;
    } else {
        memcpy(dst, tmpl[line_type(k0, k1)][lp], sizeof tmpl[0][0]);
    }
    if (g->vf.video == VID_BLUE) {
        if (normal && row >= 0 && row < ROWS) {
            fill(0, W, C_BLUE);
            out_direct(dst + ACTIVE_START / 2, lp, 0);
        }
        normal = false;
    }
    bool picture = normal;
    const uint8_t *rec = NULL;
    if (!normal) {
        if (g->vhs) vhs_clear_prev();
    } else if (lc == TAG_LINE_F1 && g->tag_on) {
        draw_tag(tag_word(g->t_log2, g->lay, g->il, g->fc));
    } else if (g->vitc_on && (lc == VITC_L1 || lc == VITC_L2)) {
        vitc_line((uint8_t *)dst, g->vitc);
        picture = false;
        if (g->vhs) vhs_clear_prev();
    } else if (g->ptag_on && lc < 32 && (PTAG_LINES >> lc & 1)) {
        ptag_line((uint8_t *)dst, g->ptag);
        picture = false;
        if (g->vhs) vhs_clear_prev();
    } else if (g->data && row >= 0 && row < ROWS) {
        data_line(g, (uint8_t *)dst + ACTIVE_START, row);
        ctl.data_sent = g->d_s;
        picture = false;
    } else if (row >= 0 && row < g->rows && g->clips) {
        rec = clip_pair(g->cf * CLIP_PAIRS + (uint32_t)(row >> 1));
    } else if (row >= 0 && row < g->rows && g->sd) {
        if (g->sd_rec) rec = g->sd_rec + (uint32_t)(row >> 1) * sdt.pair_bytes;
        else fill(0, W, C_BLACK);
    } else if (row >= 0 && row < g->rows) {
        span_t sp[2];
        int x0 = g->aud_x0, xm = (x0 + W) / 2;
        int n = layout_row(g->lay, (int)(g->fc & ((1u << g->t_log2) - 1)), row, x0, g->rows, sp);
        for (int i = 0; i < n; i++)
            draw_stream(&sp[i], g->fc >> g->t_log2, g->plain);
        if (g->lay == LAYOUT_HALVES || g->lay == LAYOUT_QUADS)
            fill(xm - g->guard_px, xm + g->guard_px, C_GREY);
        if (g->lay == LAYOUT_QUADS && row >= ROWS / 2 - g->guard_rows && row < ROWS / 2 + g->guard_rows)
            fill(x0, W, C_GREY);
        if (g->vhs && row >= ROWS - g->hs_lines && g->hs_shift > 0 && g->hs_shift < W - x0) {
            memmove(idx + x0 + g->hs_shift, idx + x0, (size_t)(W - x0 - g->hs_shift));
            fill(x0, x0 + g->hs_shift, C_BLACK);
        }
    } else {
        picture = false;
        if (g->vhs) vhs_clear_prev();
    }
    if (picture) {
        bool aud = g->aud != AUD_OFF && row >= 0 && row < g->rows;
        int x0 = aud ? g->aud_x0 : 0;   // the audio cells cover pixels below x0
        if (g->vhs) out_vhs_begin(dst + ACTIVE_START / 2, lp, x0 / 2);
        else if (rec && g->sd && sdt.version == 2)
            out_clip6(dst + ACTIVE_START / 2, x0, rec, row & 1, (int)sdt.width, (int)sdt.aud_n, sd_clut[lp],
                      sd_clut[lp ^ 1]);
        else if (rec && g->sd)
            out_clip(dst + ACTIVE_START / 2, x0, rec, row & 1, (int)sdt.width, (int)sdt.aud_n, sd_clut[lp],
                     sd_clut[lp ^ 1]);
        else if (rec)
            out_clip(dst + ACTIVE_START / 2, x0, rec, row & 1, (int)clip.width, (int)clip.aud_n, clut[lp], clut[lp ^ 1]);
        else out_direct(dst + ACTIVE_START / 2, lp, x0);
        if (aud) {
            uint32_t t0 = systick_hw->cvr;
            int mode = g->aud == AUD_CLIP && !rec ? AUD_SILENT : g->aud;
            draw_audio((uint8_t *)dst + ACTIVE_START, g->aud_n, mode, g->aud_mid16, g->aud_amp16,
                       g->aud_lead, rec ? (const int8_t *)rec + (row & 1) * g->aud_n : NULL);
            ctl.aud_cycles = (t0 - systick_hw->cvr) & 0xFFFFFF;
            ctl.aud_lines = ctl.aud_lines + 1;
        }
        if (g->vhs) out_vhs_end();
    }
    if (normal && g->vf.trick == TRICK_CUE && row >= 0 && row < ROWS && (row + (int)g->vf.t) % 80 < 10)
        memcpy((uint8_t *)dst + ACTIVE_START, bar_noise + (xs32() & 1023), 2 * W);
    else if (normal && g->vf.trick != TRICK_NONE && lc < ROW0_F1)
        vbi_damage(g, (uint8_t *)dst, lc);
    last_picture = picture;
    audio_line(g);
    g->ltc_line++;

    g->gl++;
    g->h += 2;
    if (mid) {
        g->h -= g->H;
        field_start(g, 1);
    }
}

static void core1_main(void) {
    static gen_t g = {.aud_lin = (uint32_t)-1, .ltc_len = 525, .ltc_lvl = 1};
    systick_hw->rvr = 0xFFFFFF;
    systick_hw->cvr = 0;
    systick_hw->csr = 5;                // processor clock, no interrupt
    for (;;) {
        while ((int32_t)(rendered - (queued + RING - 2)) > 0)
            tight_loop_contents();
        uint32_t t0 = systick_hw->cvr;
        render_next(&g, ring[rendered % RING]);
        uint32_t dt = (t0 - systick_hw->cvr) & 0xFFFFFF;
        if (dt > field_cycles) field_cycles = dt;
        if (last_picture) {
            pic_cycles += dt;
            pic_lines++;
        }
        __dmb();
        rendered = rendered + 1;
    }
}

// Channels A and B chain to each other; when one finishes, the other is playing the next line, so the
// finished one is re-aimed at the line after that with a whole line of slack.
static void __isr dma_irq(void) {
    uint32_t ints = dma_hw->ints0 & (1u << dma_a | 1u << dma_b);
    dma_hw->ints0 = ints;
    for (uint ch = 0; ch < NUM_DMA_CHANNELS; ch++) {
        if (!(ints & 1u << ch)) continue;
        uint32_t next = queued + 1;
        queued = next;
        if ((int32_t)(rendered - next) <= 0) ctl.underruns = ctl.underruns + 1;
        dma_channel_set_read_addr(ch, ring[next % RING], false);
    }
}

static void dac_pio_init(PIO pio, uint sm) {
    uint off = pio_add_program(pio, &tvout_program);
    for (uint i = 0; i < DAC_BITS; i++) {
        pio_gpio_init(pio, DAC_BASE_PIN + i);
        gpio_set_drive_strength(DAC_BASE_PIN + i, GPIO_DRIVE_STRENGTH_12MA);
    }
    pio_sm_set_consecutive_pindirs(pio, sm, DAC_BASE_PIN, DAC_BITS, true);

    pio_sm_config c = tvout_program_get_default_config(off);
    sm_config_set_out_pins(&c, DAC_BASE_PIN, DAC_BITS);
    sm_config_set_out_shift(&c, true, true, 16);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv_int_frac8(&c, PIO_DIV, 0);
    pio_sm_init(pio, sm, off, &c);
}

static void i2s_init(PIO pio, uint sm) {
    uint off = pio_add_program(pio, &i2s_program);
    for (uint i = 0; i < 3; i++) pio_gpio_init(pio, I2S_DIN_PIN + i);
    pio_sm_set_consecutive_pindirs(pio, sm, I2S_DIN_PIN, 3, true);

    pio_sm_config c = i2s_program_get_default_config(off);
    sm_config_set_out_pins(&c, I2S_DIN_PIN, 1);
    sm_config_set_sideset_pins(&c, I2S_BCK_PIN);
    sm_config_set_out_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv_int_frac8(&c, I2S_DIV_INT, I2S_DIV_FRAC);
    pio_sm_init(pio, sm, off + i2s_offset_entry_point, &c);

    uint ch = dma_claim_unused_channel(true);
    dma_channel_config dc = dma_channel_get_default_config(ch);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, true);
    channel_config_set_write_increment(&dc, false);
    channel_config_set_ring(&dc, false, 10);    // log2 of sizeof a_ring
    channel_config_set_dreq(&dc, pio_get_dreq(pio, sm, true));
    dma_channel_configure(ch, &dc, &pio->txf[sm], a_ring, dma_encode_endless_transfer_count(), true);
}

static void line_channel(uint ch, uint chain, PIO pio, uint sm, const void *first) {
    dma_channel_config dc = dma_channel_get_default_config(ch);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_16);
    channel_config_set_read_increment(&dc, true);
    channel_config_set_write_increment(&dc, false);
    channel_config_set_dreq(&dc, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&dc, chain);
    channel_config_set_high_priority(&dc, true);
    dma_channel_configure(ch, &dc, &pio->txf[sm], first, LINE_HW, false);
}

int main(void) {
    flash_timing();
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    sleep_ms(3);
    set_sys_clock_khz(SYS_KHZ, true);

    pal_set(C_BLACK, 0, 0, 0);
    pal_set(C_WHITE, 1, 1, 1);
    pal_set(C_GREY, 0.5f, 0.5f, 0.5f);
    pal_set(C_BLUE, 0, 0, 0.75f);       // the stopped deck's blue-back, without its OSD
    for (int i = 0; i < NSTREAMS; i++) {
        float a = stream_deg(i) * (float)M_PI / 180.0f;
        pal_set_yuv(C_STREAM0 + i, 35.0f + 6.0f * (i >> 2), 38.0f * cosf(a), 38.0f * sinf(a));
        pal_set_yuv(C_MEAS0 + i, 50.0f, 30.0f * cosf(a), 30.0f * sinf(a));
    }
    build_templates();
    vhs_setup();
    for (int i = 0; i < (int)sizeof bar_noise; i++)
        bar_noise[i] = ire_code(40.0f + 30.0f * (int8_t)xs32() / 128.0f);
    for (int i = 0; i < 4096; i++)
        sine[i] = (int16_t)lroundf(32767.0f * sinf(2.0f * (float)M_PI * i / 4096));
    aud_white = ire_code(100.0f);
    ctl.clips_ok = clips_init();
    ctl.clip_fields = ctl.clips_ok ? clip.fields : 0;
    ctl.data_ok = data_init();
    ctl.sd_ok = sd_tape_init();
    if (ctl.data_ok && data.programme >> 31) ctl.src = data.programme & 3;
    else if (ctl.sd_ok) ctl.src = 3;    // a card with a tape on it is the game tape's programme
    else sd_fallback = ctl.src == 1;
    sd_retry_at = time_us_32() + SD_RETRY_US;
    static const unsigned pad_pins[PAD_PORTS] = {40, 41};
    pad_init(pad_pins);
    vcr_ir_init(43);                    // IR receiver on the P2 controller header's spare pin
    vcr_restore((const vcr_save_t *)(XIP_BASE + SAVE_OFFSET));

    multicore_launch_core1(core1_main);
    while (rendered < RING - 1)
        tight_loop_contents();

    PIO pio = pio0;
    uint sm = 0;
    dac_pio_init(pio, sm);
    dma_a = dma_claim_unused_channel(true);
    dma_b = dma_claim_unused_channel(true);
    line_channel(dma_a, dma_b, pio, sm, ring[0]);
    line_channel(dma_b, dma_a, pio, sm, ring[1]);
    dma_hw->ints0 = 1u << dma_a | 1u << dma_b;
    dma_set_irq0_channel_mask_enabled(1u << dma_a | 1u << dma_b, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq);
    irq_set_priority(DMA_IRQ_0, 0);
    irq_set_enabled(DMA_IRQ_0, true);

    // Audio frame n then plays at n * A_T and line gl at gl * LINE_T: audio_line's clock, with no drift.
    i2s_init(pio, 1);
    dma_channel_start(dma_a);
    pio_enable_sm_mask_in_sync(pio, 1u << sm | 1u << 1);
    core0_vhs_worker();
}
