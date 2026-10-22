// Each field's records composed from the card's assets as the flash schedule places them:
// audio through a cache of LINE_SECT-sector runs, video one frame read per region.
#include "tape_store.h"

#include <string.h>
#include "pico/stdlib.h"
#include "sd.h"

#define ROWS 240
#define PAIRS (ROWS / 2)
#define ASSETS_MAX 256
#define ENTRIES_MAX 1024
#define DIR_PER_SECT 8
#define LINE_SECT 4
#define LINE_BYTES (LINE_SECT * 512)
#define LINES 24
#define FRAME_SECT_MAX 52               // a T=2 halves region: 172 x 236
#define NONE UINT32_MAX

enum { K_AUDIO, K_VIDEO };
enum { L_FULL, L_HALVES, L_QUADS, L_LINES };
static const uint8_t regions[4] = {1, 2, 4, 2};

typedef struct {
    uint32_t magic, version, n_assets, dir_first;
} store_hdr_t;

typedef struct {
    char name[24];
    uint32_t kind, first, units;        // units: samples, or frames of stride sectors
    uint32_t unit_bytes, stride, w, h;
    uint32_t pad[3];
} dir_entry_t;
_Static_assert(sizeof(dir_entry_t) == 64, "tools/tapestore.py DIR_ENTRY");

typedef struct {
    uint32_t magic, version, t_log2, layout, width, rows, aud_n, n_entries;
    int32_t y_tab[16], u_tab[16], v_tab[16];
} sched_hdr_t;

typedef struct {
    char name[24];
    uint8_t kind, target, pad[2];       // target: audio cell, or video stream
    uint32_t first, end;                // SD fields
    int32_t offset;                     // asset unit at field first, row 0
    uint32_t win0, win1;                // units played (win1 0: all); outside them silence or black
    uint32_t loop0, loop1;              // loop1 nonzero: units from loop1 on repeat [loop0, loop1)
} sched_entry_t;
_Static_assert(sizeof(sched_entry_t) == 56, "tools/tapestore.py SCHED_ENTRY");

typedef struct {
    uint32_t first, units, stride, w, h;
    uint8_t kind;
} asset_t;

store_stats_t store_stats;

static const sched_hdr_t *const sched = (const sched_hdr_t *)(XIP_BASE + SCHED_OFFSET);
static const sched_entry_t *entries;
static uint32_t n_entries;
static int16_t res[ENTRIES_MAX];
static asset_t assets[ASSETS_MAX];
static store_geom_t geom;
static uint32_t pair_bytes, reg_w, reg_h;

static uint8_t dir_raw[SD_RAW_BLOCK] __attribute__((aligned(4)));
static uint8_t line_raw[LINES][LINE_SECT * SD_RAW_BLOCK] __attribute__((aligned(4)));
static uint32_t line_key[LINES], line_gen[LINES];
static uint8_t frame_raw[FRAME_SECT_MAX * SD_RAW_BLOCK] __attribute__((aligned(4)));
static uint32_t frame_asset = NONE, frame_unit;

static struct {
    uint8_t *rec;
    uint32_t f, gen;
    int phase, region;                  // 0 audio lines, 1 video regions, 2 audio samples
    int pending;                        // line slot being read, LINES the frame, -1 none
    uint32_t pending_key;
} cs = {.pending = -1};

static void region_at(int q, uint32_t *x0, uint32_t *r0) {
    *x0 = geom.layout == L_QUADS ? (q & 1) * reg_w : geom.layout == L_HALVES ? q * reg_w : 0;
    *r0 = geom.layout == L_QUADS ? (q >> 1) * reg_h : 0;
}

bool store_init(const uint8_t *sector0, store_geom_t *out) {
    store_hdr_t h;
    memcpy(&h, sector0, sizeof h);
    store_abort();
    if (h.magic != STORE_MAGIC || h.version != 1 || h.n_assets > ASSETS_MAX || h.dir_first == 0) return false;
    if (sched->magic != SCHED_MAGIC || sched->version != 1 || sched->n_entries > ENTRIES_MAX ||
        sched->layout >= L_LINES || sched->t_log2 > 2 || sched->aud_n > 8 || sched->rows == 0 || sched->rows > ROWS)
        return false;
    geom = (store_geom_t){sched->t_log2, sched->layout, sched->width, sched->rows, sched->aud_n};
    memcpy(geom.y_tab, sched->y_tab, sizeof geom.y_tab);
    memcpy(geom.u_tab, sched->u_tab, sizeof geom.u_tab);
    memcpy(geom.v_tab, sched->v_tab, sizeof geom.v_tab);
    entries = (const sched_entry_t *)(sched + 1);
    n_entries = sched->n_entries;
    pair_bytes = 2 * geom.aud_n + geom.width + geom.width / 4;
    reg_w = geom.layout == L_FULL ? geom.width : geom.width / 2;
    reg_h = geom.layout == L_QUADS ? ROWS / 2 : ROWS;

    for (uint32_t e = 0; e < n_entries; e++) res[e] = -1;
    for (uint32_t a = 0; a < h.n_assets; a++) {
        if (a % DIR_PER_SECT == 0 && !sd_read(h.dir_first + a / DIR_PER_SECT, 1, dir_raw)) return false;
        const dir_entry_t *d = (const dir_entry_t *)dir_raw + a % DIR_PER_SECT;
        assets[a] = (asset_t){d->first, d->units, d->stride, d->w, d->h, (uint8_t)d->kind};
        for (uint32_t e = 0; e < n_entries; e++)
            if (res[e] < 0 && strncmp(entries[e].name, d->name, sizeof d->name) == 0) res[e] = (int16_t)a;
    }
    store_stats = (store_stats_t){.assets = h.n_assets, .entries = n_entries};
    for (uint32_t e = 0; e < n_entries; e++) {
        const sched_entry_t *s = &entries[e];
        const asset_t *a = res[e] >= 0 ? &assets[res[e]] : NULL;
        if (a && (a->kind != s->kind || (s->kind == K_AUDIO && s->target >= geom.aud_n) ||
                  (s->kind == K_VIDEO && (a->w != reg_w || a->h > reg_h || a->stride > FRAME_SECT_MAX ||
                                          s->target >= (1u << geom.t_log2) * regions[geom.layout]))))
            res[e] = -1;
        if (res[e] < 0) store_stats.dead++;
    }
    for (int i = 0; i < LINES; i++) line_key[i] = NONE;
    frame_asset = NONE;
    cs.rec = NULL;
    *out = geom;
    return true;
}

void store_abort(void) {
    if (cs.pending >= 0)
        while (sd_read_poll() == 0) tight_loop_contents();
    cs.pending = -1;
    cs.rec = NULL;
}

// The asset unit entry e plays at u units past its offset, or false for silence / black.
static bool unit_at(uint32_t e, int64_t u, uint32_t *out) {
    const sched_entry_t *s = &entries[e];
    const asset_t *a = &assets[res[e]];
    if (s->loop1 > s->loop0 && u >= (int64_t)s->loop1) u = s->loop0 + (u - s->loop0) % (s->loop1 - s->loop0);
    uint32_t w1 = s->win1 ? s->win1 : a->units;
    if (u < (int64_t)s->win0 || u >= (int64_t)w1 || u >= (int64_t)a->units) return false;
    *out = (uint32_t)u;
    return true;
}

static bool active(uint32_t e, uint32_t f, uint8_t kind) {
    const sched_entry_t *s = &entries[e];
    return res[e] >= 0 && s->kind == kind && f >= s->first && f < s->end;
}

static int line_find(uint32_t key) {
    for (int i = 0; i < LINES; i++)
        if (line_key[i] == key) return i;
    return -1;
}

static bool start_read(int slot, uint32_t sector, uint32_t n, uint8_t *raw) {
    if (!sd_read_start(sector, n, raw)) return false;
    cs.pending = slot;
    cs.pending_key = sector;
    store_stats.reads++;
    return true;
}

// Phase 0: every audio run this field plays cached, stamped with this compose's generation. 0 while reading.
static int audio_lines(void) {
    for (uint32_t e = 0; e < n_entries; e++) {
        if (!active(e, cs.f, K_AUDIO)) continue;
        const asset_t *a = &assets[res[e]];
        int64_t base = entries[e].offset + (int64_t)(cs.f - entries[e].first) * geom.rows;
        uint32_t last = NONE, u;
        for (uint32_t r = 0; r < geom.rows; r++) {
            if (!unit_at(e, base + r, &u)) continue;
            uint32_t key = a->first + u / LINE_BYTES * LINE_SECT;
            if (key == last) continue;
            last = key;
            int i = line_find(key);
            if (i >= 0) {
                line_gen[i] = cs.gen;
                continue;
            }
            int v = -1;
            for (int k = 0; k < LINES; k++)
                if (line_gen[k] != cs.gen && (v < 0 || line_key[k] == NONE || line_gen[k] < line_gen[v])) v = k;
            if (v < 0) return -1;
            store_stats.line_misses++;
            line_key[v] = NONE;
            return start_read(v, key, LINE_SECT, line_raw[v]) ? 0 : -1;
        }
    }
    return 1;
}

// Phase 1: each region's stream from its frame. 0 while reading a frame.
static int video_regions(void) {
    uint32_t nreg = regions[geom.layout], phase = cs.f & ((1u << geom.t_log2) - 1);
    for (; cs.region < (int)nreg; cs.region++) {
        uint32_t stream = phase * nreg + (uint32_t)cs.region, u = 0;
        int32_t e = -1;
        for (uint32_t k = 0; k < n_entries && e < 0; k++) {
            if (!active(k, cs.f, K_VIDEO) || entries[k].target != stream) continue;
            int64_t n = (int64_t)(cs.f >> geom.t_log2) - (int64_t)(entries[k].first >> geom.t_log2);
            if (unit_at(k, entries[k].offset + n, &u)) e = (int32_t)k;
        }
        if (e < 0) continue;
        const asset_t *a = &assets[res[e]];
        if (frame_asset != (uint32_t)res[e] || frame_unit != u) {
            frame_asset = NONE;
            store_stats.frame_reads++;
            if (!start_read(LINES, a->first + u * a->stride, a->stride, frame_raw)) return -1;
            cs.pending_key = (uint32_t)res[e];
            frame_unit = u;
            return 0;
        }
        uint32_t x0, r0, lw = reg_w / 2, cw = reg_w / 4;
        region_at(cs.region, &x0, &r0);
        for (uint32_t r = 0; r < a->h; r++) {
            uint32_t row = r0 + r;
            memcpy(cs.rec + (row >> 1) * pair_bytes + 2 * geom.aud_n + (row & 1) * (geom.width / 2) + x0 / 2,
                   frame_raw + r * lw, lw);
        }
        const uint8_t *c = frame_raw + a->h * lw;
        for (uint32_t p = 0; p < a->h / 2; p++)
            memcpy(cs.rec + (r0 / 2 + p) * pair_bytes + 2 * geom.aud_n + geom.width + x0 / 4, c + p * cw, cw);
    }
    return 1;
}

// Phase 2: the samples, all cached by phase 0.
static void audio_samples(void) {
    for (uint32_t e = 0; e < n_entries; e++) {
        if (!active(e, cs.f, K_AUDIO)) continue;
        const asset_t *a = &assets[res[e]];
        int64_t base = entries[e].offset + (int64_t)(cs.f - entries[e].first) * geom.rows;
        uint32_t cell = entries[e].target, last = NONE, u;
        const uint8_t *line = NULL;
        for (uint32_t r = 0; r < geom.rows; r++) {
            if (!unit_at(e, base + r, &u)) continue;
            uint32_t key = a->first + u / LINE_BYTES * LINE_SECT;
            if (key != last) {
                int i = line_find(key);
                line = i >= 0 ? line_raw[i] : NULL;
                last = key;
            }
            if (line) cs.rec[(r >> 1) * pair_bytes + (r & 1) * geom.aud_n + cell] = line[u % LINE_BYTES];
        }
    }
}

int store_compose(uint8_t *rec, uint32_t f) {
    if (cs.pending >= 0) {
        int r = sd_read_poll();
        if (r == 0) return 0;
        int slot = cs.pending;
        cs.pending = -1;
        if (r < 0) {
            store_stats.fails++;
            return -1;
        }
        if (slot < LINES) {
            line_key[slot] = cs.pending_key;
            line_gen[slot] = cs.gen;
        } else {
            frame_asset = cs.pending_key;
        }
    }
    if (cs.rec != rec || cs.f != f) {
        cs.rec = rec;
        cs.f = f;
        cs.gen++;
        cs.phase = 0;
        cs.region = 0;
        for (uint32_t p = 0; p < PAIRS; p++) {
            uint8_t *q = rec + p * pair_bytes;
            memset(q, 0, 2 * geom.aud_n + geom.width);
            memset(q + 2 * geom.aud_n + geom.width, 0x77, geom.width / 4);    // neutral chroma: black
        }
    }
    int r;
    if (cs.phase == 0) {
        if ((r = audio_lines()) <= 0) return r;
        cs.phase = 1;
    }
    if (cs.phase == 1) {
        if ((r = video_regions()) <= 0) return r;
        cs.phase = 2;
    }
    audio_samples();
    cs.rec = NULL;
    return 1;
}
